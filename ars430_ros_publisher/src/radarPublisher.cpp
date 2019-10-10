#include "radarPublisher.h"
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>

//msgs
#include "std_msgs/String.h"
#include "ars430_ros_publisher/RadarDetection.h" 
#include "ars430_ros_publisher/RadarPacket.h"
#include "ars430_ros_publisher/SensorStatus.h"

//#define PRINT_PUBLISHER

RadarPublisher::RadarPublisher (ros::NodeHandle nh_, uint8_t ID)
{
  packet_pub_  = nh_.advertise<ars430_ros_publisher::RadarPacket>(std::string("filtered_radar_packet_") + std::to_string(ID), 50);
}

uint8_t RadarPublisher::setupProcessor(uint8_t ID) 
{
  uint8_t err = PacketProcessor::initializePacketProcessor(ID, this);
  if (err) {
    printf("Failed to Start Processor: %d\r\n", err);
    return err; //Processor Failed to Init
  }
  return SUCCESS;
}

void RadarPublisher::publishRadarPacketMsg(ars430_ros_publisher::RadarPacket& radar_packet_msg) 
{
  return packet_pub_.publish(radar_packet_msg);
}


uint8_t RadarPublisher::pubCallback(PacketGroup_t* Packets) {   //call upon the appropriate publish function
  for (uint8_t i = 0; i < Packets->numFarPackets; i++) {
    this->publishRadarPacketMsg(Packets->farPackets[i]);
  }

  for (uint8_t i = 0; i < Packets->numNearPackets; i++) {
    this->publishRadarPacketMsg(Packets->nearPackets[i]);
  }

  return SUCCESS;
}

