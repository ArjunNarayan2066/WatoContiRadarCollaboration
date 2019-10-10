/* General Use Includes */
#include "processPacket.h"
#include <cstring>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>

//Processing Rules: Written out in better detail in the spec doc
//Spec Doc: https://docs.google.com/document/d/13W7H_Z5pgCx8rVgnv7u1MaKYaGQvnPspWVuuTmzJgUA/edit

//#define PROCESS_SS_PACKET //Perception doesn't need it

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
void loadRDIMessageFromPacket(ars430_ros_publisher::RadarPacket* newMsg, const ars430_ros_publisher::RadarPacket::ConstPtr& oldMsg);
void loadSSMessage(ars430_ros_publisher::SensorStatus* msg, SSPacket_t* packet);

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
        loadRDIMessageFromPacket(&curGroup->farPackets[curGroup->numFarPackets], packet);
        curGroup->numFarPackets++;

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
        loadRDIMessageFromPacket(&curGroup->nearPackets[curGroup->numNearPackets], packet);
        curGroup->numNearPackets++;

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

/* Process Sensor Status Packet
* -- Add packet to our internal struct
* -- Call our publisher to publish all packets since we have a group
* -- Publish as soon as we have the sensor status packet
*/
uint8_t PacketProcessor::processSSPacket(SSPacket_t* packet) {
    // TODO: Raise an alarm on the dashboard with error info.
    if (packet->Defective && DEFECTIVE_HW) {
        return SS_DEFECTIVE_HW;
    } else if (packet->BadSupplyVolt && BAD_VOLTAGE) {
        return SS_BAD_VOLT;
    } else if (packet->BadTemp && BAD_TEMP) {
        return SS_BAD_TEMP;
    } else if (packet->GmMissing && GM_MISSING) {
        return SS_GM_MISSING;
    } else if (packet->TxPowerStatus && POWER_REDUCED) {
        return SS_PWR_REDUCED;
    }
    pthread_mutex_lock(&Mutex);

    //TODO: Implement Error Checking Here...

#if PROCESS_SS_PACKET
    loadSSMessage(&Packets.sensorStatusMsg, packet);    
#endif
    
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

void loadRDIMessageFromPacket(ars430_ros_publisher::RadarPacket* newMsg, const ars430_ros_publisher::RadarPacket::ConstPtr& oldMsg) {
    newMsg->EventID                    = oldMsg->EventID;
    newMsg->TimeStamp                  = oldMsg->TimeStamp;
    newMsg->MeasurementCounter         = oldMsg->MeasurementCounter;
    newMsg->Vambig                     = oldMsg->Vambig;
    newMsg->CenterFrequency            = oldMsg->CenterFrequency;

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
}

/* Load SS ROS Message
* -- Local Function, not in class definition
*/
void loadSSMessage(ars430_ros_publisher::SensorStatus* msg, SSPacket_t* packet) {
    msg->PartNumber             = packet->PartNumber;
    msg->AssemblyPartNumber     = packet->AssemblyPartNumber;
    msg->SWPartNumber           = packet->SWPartNumber;
    for (uint8_t i = 0; i < SENSOR_SERIAL_NUM_LEN; i++) {
        msg->SerialNumber[i]    = packet->SerialNumber[i]; //Should work for a boost::array object
    }
    msg->BLVersion              = packet->BLVersion;
    msg->SWVersion              = packet->SWVersion;
    msg->UTCTimeStamp           = packet->UTCTimeStamp;
    msg->TimeStamp              = packet->TimeStamp;
    msg->SurfaceDamping         = packet->SurfaceDamping;
    msg->OpState                = packet->OpState;
    msg->CurrentFarCF           = packet->CurrentFarCF;
    msg->CurrentNearCF          = packet->CurrentNearCF;
    msg->Defective              = packet->Defective;
    msg->BadSupplyVolt          = packet->BadSupplyVolt;
    msg->BadTemp                = packet->BadTemp;
    msg->GmMissing              = packet->GmMissing;
    msg->TxPowerStatus          = packet->TxPowerStatus;
    msg->MaximumRangeFar        = packet->MaxRangeFar;
    msg->MaximumRangeNear       = packet->MaxRangeNear;
}

/* Set ROS Publisher Callback
*/
void PacketProcessor::setPublisherCallback(RadarPublisher* newPublisher) {
    pthread_mutex_lock(&Mutex);
    Publisher = newPublisher;
    pthread_mutex_unlock(&Mutex);
}

/* Set the current radarID
*/
void PacketProcessor::setRadarID(uint8_t newRadarID) {
    pthread_mutex_lock(&Mutex);
    radarID = newRadarID;
    pthread_mutex_unlock(&Mutex);
}

/* Print the currently selected buffer
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
