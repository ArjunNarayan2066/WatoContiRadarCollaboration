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


## Setup

First, create a new directory to contain your catkin workspace. Place a copy of this repo in a folder called *src*. Your directory structure should look something like this:
`./catkin_ws/src/WatoContiRadarCollaboration/`

Next, run `catkin_make` in your workspace root to initialize your workspace. Once it has initialized with no errors, source the setup.bash file into your environment
```
cd ~/catkin_ws; source devel/setup.bash
```

### Launch Radar with Rosbag

This method only runs the `radar_processor` and `radar_visualizer`. Rosbags are recorded from output of the `/unfiltered_radar_packet_ID` topic from the `radar_publisher` node. Generally this method is used when testing different filtering algorithms, and so only the `radar_processor` and `radar_visualizer` nodes are needed.

```
roslaunch ars430_ros_publisher radarROSbag.launch id:=1 bagPath:= /path/to/rosbag/file/including/filename

ex. roslaunch ars430_ros_publisher radarROSbag.launch id:=1 bagPath:= /home/wato/rosbags/rosbag1.bag
```
The id arg sets the name of the topic, in our case the bags were recorded with ID = 1 so this will always need to be set to 1. The bagPath allows the playing of the rosbag

### Launch Rosbag Independantly

In the case where the visualizer is not needed the rosbag node and `radar_processor` nodes can be run manually.

To play the rosbag:
```bash
# Run ROSBAG wherever it is
rosbag play -lq rosbag.bag # -lq means loop & quiet
...
# Run processor node
rosrun ars430_ros_publisher radar_processor -i 1 # Assuming id = 1
```

### Notes
RQT is a helpful tool part of the ros package that has many plugins for monitoring topics, printing to the console, visualizing the node structure and flow of the topics between them, etc.

It can be run from any roscore session by entering `rqt` in a seperate terminal

