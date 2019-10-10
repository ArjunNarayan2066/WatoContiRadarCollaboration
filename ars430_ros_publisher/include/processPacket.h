#ifndef PROCESS_PACKET_H
#define PROCESS_PACKET_H

/* File: processPacket.h
 * 
 * Groups incoming packets into buffer based on timestamp & type
 * Performs simple filtering & thresholding on simeple packet contents
 */

/* General Use Includes */
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>
#include "parser.h"
#include "radarPublisher.h"

//Return Values
#define SUCCESS             0
#define NO_DETECTIONS       1
#define PUBLISH_FAIL        2
#define CLEAR_FAIL          3
#define BAD_EVENT_ID        4
#define NO_PUBLISHER        5
#define INIT_FAIL           6
#define BAD_PORT            7
#define BAD_SERVICE_ID      8
#define NO_PUBLISHER        9
#define NO_PUB_CLR_FAIL     10
#define FALSE_DETECTION_1   11
#define FALSE_DETECTION_2   12
#define FALSE_DETECTION_3   13
#define TOO_MUCH_NOISE      14
#define SS_DEFECTIVE_HW     15
#define SS_BAD_VOLT         16
#define SS_BAD_TEMP         17
#define SS_GM_MISSING       18
#define SS_PWR_REDUCED      19
#define NO_PROCESS          20

//Filter Thresholds
#define SNR_THRESHOLD            3 //dBr
#define VELOCITY_LOWER_THRESHOLD 0.00 // m/s
#define RCS_THRESHOLD           -70 //(dBm)^2
#define DISTANCE_MAX_THRESHOLD   70 //m
#define DISTANCE_MIN_THRESHOLD   0.25 //m

class PacketProcessor {
    private:
        static class RadarPublisher*    Publisher; //Publisher Object for Callback
        static pthread_mutex_t          Mutex; //Mutex for synchronization
        static PacketGroup_t            PacketsBuffer[2]; //Double buffer
        static uint8_t                  curNearIdx, curFarIdx; //Double buffer indexes
        static uint32_t                 curNearTimeStamp, curFarTimeStamp; //Time Stamps
        static uint8_t                  radarID;
        static bool                     publish;

        static uint8_t clearPackets(uint8_t idx); //Don't expose these
        static uint8_t clearAllPackets();
        static uint8_t publishPackets(uint8_t idx);
        
    public:
        static uint8_t initializePacketProcessor(uint8_t newRadarID, RadarPublisher* newPublisher);
        static uint8_t processRDIMsg(const ars430_ros_publisher::RadarPacket::ConstPtr& packet);

        static void    setPublisherCallback(RadarPublisher* newPublisher);
        static void    setRadarID(uint8_t newRadarID);

        /* Print the index buffer chosen */
        static void    printPacketGroup(uint8_t idx);
};

#endif /* PROCESS_PACKET_H */
