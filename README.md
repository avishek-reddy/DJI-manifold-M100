# DJI-manifold-M100
DJI M100 Manifold with Guidance system

ROS version: indigo

Takes the feed from all the 5 stereo cameras from guidance and transmits it compressed over external WiFi module connected to the Manifold along with all the flight data.

Also includes the flight control nodes for DJI M100 as well as the sensor data filter for all sensors.

Visual SLAM is done via RTABMAP running on a ground station laptop recieving the data from Manifold via WiFi(router is attached to the manifold with a custonm battery pack) and a 3D model of the environment is generated

Sensor fusion nodes are running on the ground station using 2 IMU's and visual odometry data to get a more accurate odometry data.
