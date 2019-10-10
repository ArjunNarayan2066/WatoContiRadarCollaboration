# Radar Publisher

ROS Publisher node to listen in and process packets.

Simplified Packet Sniffer program to read from Radar devices. Implemented using the libpcap packet sniffer. Code format follow http://wiki.ros.org/CppStyleGuide

## Built by ROS
### Dependencies
Run the following in the terminal to install the packet capture library we use
1. `sudo apt-get install libpcap0.8-dev` 

## Software Architecture

The radar driver system will take in raw UDP packets over ethernet from the radar, and output clean & clear
data for perception in the form of ROS messages. This is currently accomplished using 3 individual nodes & topics.
```
Nodes & Topics:
1. radar_publisher -> This will take in the raw radar data & convert to ROS messages. This does no processing, filtering or grouping. It is simply meant to convert the data types to something ROS can understand & manipulate
    
    Publishes To: `unfiltered_radar_packet_ID`

2. radar_processor -> This will take in the radar ROS messages & covert the raw data into clean data for perception.
This first step is to group the data by timestamp to get a proper sense of the order of the data. Then we run some basic thresholding to eliminate the obviously unnecessary packets. The next step is to perform whatever processing/filtering is required to clean the data. Then output
    
    Subscribes To: `unfiltered_radar_packet_ID`
    Publishes To: `filtered_radar_packet_ID`

3. radar_visualizer -> This will take the clean ROS Radar messages and convert to the SensorMsgs/PointCloud2 ROS Message type. This is so that the rviz visualizer we have can properly visualize the data.
    
    Subscribes To: `filtered_radar_packet_ID`
    Publishes To: `radar_pointcloud_ID`
```


## Run

First, create a new directory to contain your catkin workspace. Place embedded_actuation_and_controls in a folder called *src*. Your directory structure should look something like this:
`./workspace/src/embedded_actuation_and_controls/`

Next, run `catkin_make` in your workspace root to initialize your workspace. Once it has initialized with no errors, source the setup.bash file into your environment
```
~/wato/workspace source devel/setup.bash
```
Note if you are using a different shell (.sh or .zsh), there will be a corresponding setup file. If you run into message dependency errors, clone the ros_msgs repo in the same directory and build again

### Run CLI
Run by specifying an interface and a Berkeley packet filter expression. Assuming
the radars are on ethernet and interface enp3s0, port 31122, and ID 1, the arguments would be:
```
rosrun ars430_ros_publisher radar_publisher -i 1 -e enp3s0 -p 31122 -c 1
```
The -i is radar id, -p for the input port, -e for the ethernet interface. -f is 
an optional string represeting an additonal filter string options. -c is to indicate live capture.


### Launch single radar
```
roslaunch ars430_ros_publisher visualizer.launch id:=1 port:=31122 iface:='enp3s0' capture:=1
```
The id arg sets the name of the node, and the port & iface args set the port & interface. The capture arg tells the driver if you using live radar data or recorded pcap data. ID and Capture default to 1.

### Launch single radar offline from pcap (deprecated)
```
roslaunch ars430_ros_publisher visualizer.launch id:=1 port:=31122 iface:='enp3s0' capture:=0 cpath:="/abs/dir/to/pcap/file.pcap"
```
Capture must be set to 0 and cpath used to direct to the intended pcap file.

### Launch single radar offline from rosbag

This method only runs the radar_processor. Rosbags are recorded from output of the `/unfiltered_radar_packet` topic from the `radar_publisher` node. Generally this method is used when testing different filtering algorithms, and so only the `radar_processor` node is run.

```bash
# Run ROSBAG wherever it is
rosbag play -lq rosbag.bag # -lq means loop & quiet

...

# Run processor node
rosrun ars430_ros_publisher radar_processor -i 1 # Assuming id = 1
```

**Find iface from typing `ifconfig` into terminal (first ethernet interface, starts with 'e').**

The [filter] argument is optional, represents any additional packet string settings. It
can be used as follows:
```
roslaunch ars430_ros_publisher single_radar.launch ... filter="'more settings'"
```
The filter string must be surrounded by double quotes ("") and single quotes ('') in
either order. This is just however ROS implemented their args I guess.

### NOTE:
If you are getting permission errors, run the following first on the linux terminal:
```
sudo setcap 'cap_net_raw=pe' devel/lib/ars430_ros_publisher/radar_publisher
```
This will enable the radar_publisher executable (node) to have sufficient permissions
to listen to packets.

### Run Multiple Radars
```
roslaunch ars430_ros_publisher all_radar.launch --screen
```
Optional `--screen` to see output
(Comment publisher and rviz out of visualizer.launch first)

## Visualizer
The visualizer is run using the radar_visualizer node in the same package.

1. Run the radar_publisher node as normal (offline or live)
2. Run the visualizer node:
```
rosrun ars430_ros_publisher radar_visualizer -i 1 -t unfiltered_radar_packet_1
    The topic name is needed. This lets the visualizer show whatever data we want, that being from the unfiltered or filtered topic. 
```
3. Run rviz. There is a predefined configuration file to set up the cloud. This is at: 
catkin_ws/src/embedded_actuation_and_controls/ars430_ros_publisher/radarPointCloud.rviz
```
rosrun rviz rviz -d radarPointCloud.rviz
```
4. Ensure you have a PointCloud2 display object setup in the left hand panel, and that the topic 
used is /radar_pointcloud. To ensure PointCloud2 packets are being published run the following:
```
rostopic echo /radar_pointcloud_1
```

Launch just one publisher and visualizer through launch file:
```
roslaunch ars430_ros_publisher visualizer.launch id:=2 port:=31122 iface:=enp3s0 capture:=1
```

### Fall 2019 - Members

Arjun Narayan

Steven Tuer

Mitchell Hoyt

Saeejith Nair

Jonathan Thomas

Vivek Lasi

