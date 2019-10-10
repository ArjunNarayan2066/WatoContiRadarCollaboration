#include <ros/ros.h>
#include <getopt.h>
#include "radarPublisher.h"
#include "processPacket.h"

extern int opterr;

void printUsage(char buff[]) {
  printf("\nUsage: %s [-h] [-i ID]\r\n", buff);
  printf("\ti: radar id (int)\r\n");
}

void radarCallback(const ars430_ros_publisher::RadarPacket::ConstPtr& msg) {
  PacketProcessor::processRDIMsg(msg);
}

int main(int argc, char** argv)
{
  opterr = 0;
  ros::init(argc, argv, "radar_processor");
  ROS_INFO("Initialize radar processor");
  int ID = 0, c;//Radar Default

  // Get the command line option, if any
  while ((c = getopt (argc, argv, "+hi:")) != -1)
  { //Pass remainder of arguments to the sniffer
    switch (c)
    {
      case 'h':
        printUsage(argv[0]);
        exit(0);
        break;
      case 'i':
        ID = atoi(optarg);
        printf("ID: %d\r\n", ID);
        break;
    }
  }

  if (ID == 0) {
    printf("Bad ID or no ID given\r\n");
    return 0;
  }

  if (argc <= 1) {
      ROS_INFO("Insufficient args\r\n");
      printUsage(argv[0]);
      return 0;
  }
  
  ros::NodeHandle nh;
  RadarPublisher rp(nh, ID); //Send as int (radar ID)
  ros::Subscriber radarUnfilteredSub;

  
  if (!rp.setupProcessor(ID)) {
    radarUnfilteredSub = nh.subscribe("unfiltered_radar_packet_" + std::to_string(ID), 100, radarCallback);
    ros::spin();
    
  } else {
    ROS_INFO("Failed to setup processor\r\n");
  }

  return 0;
}
