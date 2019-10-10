#ifndef RADAR_PUBLISHER_H
#define RADAR_PUBLISHER_H

/* File: radarPublisher.h
 * 
 * Publishes the radar data
 */

#include <ros/ros.h>
#include <sys/types.h>
#include <sys/prctl.h>
#include "processPacket.h"

 //msgs
 #include "ars430_ros_publisher/RadarPacket.h"
 #include "ars430_ros_publisher/RadarDetection.h"
 #include "ars430_ros_publisher/SensorStatus.h"

class RadarPublisher {
	private:
		ros::NodeHandle nh_; //Node Handle

		// Publisher Objects
		ros::Publisher packet_pub_;

		//Publisher Functions
		void publishRadarPacketMsg(ars430_ros_publisher::RadarPacket& radar_packet_msg);

	public:
		RadarPublisher(ros::NodeHandle new_nh_, uint8_t ID);
		uint8_t setupProcessor(uint8_t ID);
		uint8_t pubCallback(PacketGroup_t* Packets);
};

#endif
