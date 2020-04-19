# DJI-manifold-M100
DJI M100 Manifold with Guidance system

ROS version: indigo

Takes the feed from all the 5 stereo cameras from guidance and transmits it over external WiFi module connected to the Manifold.

Also includes the flight control for Manifold on DJI M100 as well as the sensor calibration and sensor fusion.

Visual SLAM is done via RTABMAP on a laptop recieving the data from Manifold via WiFi and a 3D model of the environment is created.
