/* General Use Includes */
#include "processPacket.h"
#include <cstring>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>

//STATIC INITS
pthread_mutex_t      PacketProcessor::Mutex = PTHREAD_MUTEX_INITIALIZER; //Non recursive
PacketGroup_t        PacketProcessor::PacketsBuffer[2]; //Init Double Buffer
RadarPublisher *     PacketProcessor::Publisher = NULL; //Publisher object for callback
bool                 PacketProcessor::publish = false; //Publishing flag
static PacketGroup_t EmptyPackets; //Local static only, Cleanup struct
uint32_t PacketProcessor::curNearTimeStamp = 0, PacketProcessor::curFarTimeStamp = 0;
uint8_t  PacketProcessor::curNearIdx = 0,       PacketProcessor::curFarIdx = 0;
uint8_t  PacketProcessor::radarID = 0; //Needed by compiler, overwritten in init

//Some local functions
bool loadRDIMessageFromPacket(ars430_ros_publisher::RadarPacket* newMsg, const ars430_ros_publisher::RadarPacket::ConstPtr& oldMsg);

/* Initialization Processor
* -- Not using normal constructor b/c all methods are static
*/
uint8_t PacketProcessor::initializePacketProcessor(uint8_t newRadarID, RadarPublisher* newPublisher) {
    pthread_mutex_lock(&Mutex);
    
    if (clearAllPackets()) { //Wipe internal struct
        pthread_mutex_unlock(&Mutex);
        return INIT_FAIL;
    }

    radarID = newRadarID;
    Publisher = newPublisher;
    //All packet structs should be 0'd by default
    pthread_mutex_unlock(&Mutex);

    return SUCCESS;
}

uint8_t PacketProcessor::processRDIMsg(const ars430_ros_publisher::RadarPacket::ConstPtr& packet) {
    if ( packet->Detections.size() == 0 ){
        return NO_DETECTIONS;
    }

    //Only mutex here b/c not using the Packets struct yet
    pthread_mutex_lock(&Mutex);

    // DOUBLE BUFFER GROUPING
    if(packet->EventID == FAR1 || packet->EventID == FAR0) {
        if (curFarTimeStamp == 0){ //Init Case
            curFarTimeStamp = packet->TimeStamp;
        } else if (curFarTimeStamp != packet->TimeStamp) { //New timestamp
            curFarIdx = (curFarIdx + 1) % 2;

            if (curFarIdx == curNearIdx) { //Indexes are the same
                // We just got a new TS for both near & far, so we should publish the old buffer
                publish = true;
            }
        }

        PacketGroup_t * curGroup = &PacketsBuffer[curFarIdx]; //Tmp to make code easier to read
        // Load message into current buffer
        if (loadRDIMessageFromPacket(&curGroup->farPackets[curGroup->numFarPackets], packet)) {
            curGroup->numFarPackets++;
        }

        if (publish) {
            uint8_t err = publishPackets((curFarIdx+1)%2); //Publish the previous buffer idx
            publish = false;
            pthread_mutex_unlock(&Mutex);
            return err;
        }
    }

    else if (packet->EventID == NEAR0 || packet->EventID == NEAR1 || packet->EventID == NEAR2) {
        if (curNearTimeStamp == 0){ //Init Case
            curNearTimeStamp = packet->TimeStamp;
        } else if (curNearTimeStamp != packet->TimeStamp) { //New timestamp
            curNearIdx = (curNearIdx + 1) % 2;

            if (curNearIdx == curFarIdx) { //Indexes are the same
                // We just got a new TS for both near & far, so we should publish the old buffer
                publish = true;
            }
        }

        PacketGroup_t* curGroup = &PacketsBuffer[curNearIdx]; //Tmp to make code easier to read
        // Load message into current buffer
        if (loadRDIMessageFromPacket(&curGroup->nearPackets[curGroup->numNearPackets], packet)) {
            curGroup->numNearPackets++;
        }

        if (publish) {
            uint8_t err = publishPackets((curNearIdx+1)%2); //Publish the previous buffer idx
            publish = false;
            pthread_mutex_unlock(&Mutex);
            return err;
        }
    }
    else {
        pthread_mutex_unlock(&Mutex);
        return BAD_EVENT_ID;
    }

    pthread_mutex_unlock(&Mutex);
    return SUCCESS;
}

/* Publish Packets
* -- Send our packet struct to the ROS publisher
* -- Clear our intial struct & reset
* -- Only called ffrom synchronized methods, dont need to mutex
*/
uint8_t PacketProcessor::publishPackets(uint8_t idx) {
    if (Publisher == NULL) { //Publisher not set up
        //Check if we can still clear the packets
        if (clearPackets(idx)) { //Everything failed
            return NO_PUB_CLR_FAIL;
        }
        return NO_PUBLISHER; //Only null publisher, cleared ok
    } else if (Publisher->pubCallback(&PacketsBuffer[idx])) {
        return PUBLISH_FAIL;
    }

    if (clearPackets(idx)) {
        return CLEAR_FAIL;
    }
    return SUCCESS;
}

/* Clear Packet Struct
* -- Empty one internal packet struct
* -- Only called form synchronized methods, dont need to mutex
*/
uint8_t PacketProcessor::clearPackets(uint8_t idx) {
    if (idx < 2) {
        PacketsBuffer[idx] = EmptyPackets; //Replace w/ empty struct
        PacketsBuffer[idx].numFarPackets = 0;
        PacketsBuffer[idx].numNearPackets = 0;
        return SUCCESS;
    }
    return CLEAR_FAIL;
}

/* Clear All Packet Buffers
* -- Empty both packet structs
* -- Only called form synchronized methods, dont need to mutex
*/
uint8_t PacketProcessor::clearAllPackets() {
    clearPackets(0);
    clearPackets(1);

    return SUCCESS;
}

bool loadRDIMessageFromPacket(ars430_ros_publisher::RadarPacket* newMsg, const ars430_ros_publisher::RadarPacket::ConstPtr& oldMsg) { 
    newMsg->EventID                    = oldMsg->EventID;
    newMsg->TimeStamp                  = oldMsg->TimeStamp;
    newMsg->MeasurementCounter         = oldMsg->MeasurementCounter;
    newMsg->Vambig                     = oldMsg->Vambig;
    newMsg->CenterFrequency            = oldMsg->CenterFrequency;
    newMsg->Detections.clear();

    for(uint8_t i = 0; i < oldMsg->Detections.size(); i++) {

        // TODO: Figure out an SNR threshold value that actually works here.
        if (oldMsg->Detections[i].SNR < SNR_THRESHOLD) { // Too much noise; drop detection.
            continue;
        } else if (abs(oldMsg->Detections[i].VrelRad) < VELOCITY_LOWER_THRESHOLD) {
            continue;
        } else if (oldMsg->Detections[i].posX > DISTANCE_MAX_THRESHOLD) { // need to do trig
            continue;
        } else if (oldMsg->Detections[i].posX < DISTANCE_MIN_THRESHOLD) {
            continue;
        }

        ars430_ros_publisher::RadarDetection data;

        // move check prob to parser.cpp

        data.AzAng        = oldMsg->Detections[i].AzAng;
        data.RCS          = oldMsg->Detections[i].RCS;
        data.AzAngVar     = oldMsg->Detections[i].AzAngVar;

        data.VrelRad      = oldMsg->Detections[i].VrelRad;
        data.ElAng        = 0; //oldMsg->Detections[i].f_ElAng; //Told to ignore by continental
        data.RangeVar     = oldMsg->Detections[i].RangeVar;
        data.VrelRadVar   = oldMsg->Detections[i].VrelRadVar;
        data.ElAngVar     = oldMsg->Detections[i].ElAngVar;
        data.SNR          = oldMsg->Detections[i].SNR;

        // VEEEERY SIMPLIFIED, probably will be more complex/accurate than this
        data.posX = oldMsg->Detections[i].posX;
        data.posY = -1 * data.posX * tan(data.AzAng);  //Flip y axis
        data.posZ = data.posX * tan(data.ElAng);

        newMsg->Detections.push_back(data);
    }

    // Return true if there is at least one detection in newMsg after filtering
    return (newMsg->Detections.size() > 0);
}

/* Print to console the currently selected buffer
    For debugging
*/
void PacketProcessor::printPacketGroup(uint8_t idx) {
    pthread_mutex_lock(&Mutex);
    if (idx >= 2) {
        printf("Improper Idx Given: %u\r\n", idx);
        return;
    }
    PacketGroup_t * Packets = &PacketsBuffer[idx];

    for (uint8_t j = 0; j < Packets->numFarPackets; j++) {
        printf("\nPROC:Far Packet: %d, len: %u\n", j, Packets->farPackets[j].Detections.size());
        for(uint8_t i = 0; i < Packets->farPackets[j].Detections.size(); i++) {
            printf("PROC:RDI Idx: %d \n"    , i);
            printf("PROC:posX %f \n"        , Packets->farPackets[j].Detections[i].posX);
            printf("PROC:posY %f \n"        , Packets->farPackets[j].Detections[i].posY);
            printf("PROC:posZ %f \n"        , Packets->farPackets[j].Detections[i].posZ);
            printf("PROC:VrelRad %f \n"     , Packets->farPackets[j].Detections[i].VrelRad);
            printf("PROC:AzAng %f \n"       , Packets->farPackets[j].Detections[i].AzAng);
            printf("PROC:ElAng %f \n"       , Packets->farPackets[j].Detections[i].ElAng);
            printf("PROC:RCS %f \n"         , Packets->farPackets[j].Detections[i].RCS);
            printf("PROC:RangeVar %f \n"    , Packets->farPackets[j].Detections[i].RangeVar);
            printf("PROC:VrelRadVar %f \n"  , Packets->farPackets[j].Detections[i].VrelRadVar);
            printf("PROC:AzAngVar %f \n"    , Packets->farPackets[j].Detections[i].AzAngVar);
            printf("PROC:ElAngVar %f \n"    , Packets->farPackets[j].Detections[i].ElAngVar);
            printf("PROC:SNR %f \n"         , Packets->farPackets[j].Detections[i].SNR); 
            printf("\n"); 
        }
    }

    for (uint8_t j = 0; j < Packets->numNearPackets; j++) {
        printf("PROC:Near Packet: %d, len: %u\n", j, Packets->nearPackets[j].Detections.size());
        for(uint8_t i = 0; i < Packets->nearPackets[j].Detections.size(); i++) {
            printf("PROC:RDI Idx: %d \n"    , i);
            printf("PROC:posX %f \n"        , Packets->nearPackets[j].Detections[i].posX);
            printf("PROC:posY %f \n"        , Packets->nearPackets[j].Detections[i].posY);
            printf("PROC:posZ %f \n"        , Packets->nearPackets[j].Detections[i].posZ);
            printf("PROC:VrelRad %f \n"     , Packets->nearPackets[j].Detections[i].VrelRad);
            printf("PROC:AzAng %f \n"       , Packets->nearPackets[j].Detections[i].AzAng);
            printf("PROC:ElAng %f \n"       , Packets->nearPackets[j].Detections[i].ElAng);
            printf("PROC:RCS %f \n"         , Packets->nearPackets[j].Detections[i].RCS);
            printf("PROC:RangeVar %f \n"    , Packets->nearPackets[j].Detections[i].RangeVar);
            printf("PROC:VrelRadVar %f \n"  , Packets->nearPackets[j].Detections[i].VrelRadVar);
            printf("PROC:AzAngVar %f \n"    , Packets->nearPackets[j].Detections[i].AzAngVar);
            printf("PROC:ElAngVar %f \n"    , Packets->nearPackets[j].Detections[i].ElAngVar);
            printf("PROC:SNR %f \n"         , Packets->nearPackets[j].Detections[i].SNR); 
            printf("\n"); 
        }
    }

    pthread_mutex_lock(&Mutex);
}
