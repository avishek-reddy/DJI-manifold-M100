#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>

//Global variables
#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

//Callback functions
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

void battery_state_callback(const sensor_msgs::BatteryState::ConstPtr& msg);

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);

void height_above_takeoff_callback(const std_msgs::Float32::ConstPtr& msg);

//Helper functions
bool obtain_control();

bool takeoff_land(int task);

bool M100monitoredTakeoff();

bool M100monitoredLanding();

#endif // FLIGHT_CONTROL_H
