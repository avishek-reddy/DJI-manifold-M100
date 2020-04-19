//DJI SDK includes
#include "flight_control.h"
#include "dji_sdk/dji_sdk.h"

//ROS services
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;

//ROS publishers
ros::Publisher ctrlGenericPub;
ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlVelYawratePub;
ros::Publisher ctrlRollPitchYawZPub;


// global variables for subscribed topics
const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
geometry_msgs::Quaternion current_atti;


// Callback Functions
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

//Helper functions


bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }
  ROS_INFO("Obtained control");
  return true;
}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }
  return true;
}


bool M100monitoredLanding()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND))
  {
    ROS_ERROR("Landing failed.");
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  //Checking if landed in 4 secs
  while (ros::Time::now() - start_time < ros::Duration(4))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
  
  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_FINISHED_LANDING)
  {
    ROS_ERROR("Landing failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful landing !");
    ros::spinOnce();
  }

  return true;
}




/////////////////////  MAIN  ////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flight_control_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);

  // Publish the control signal
  ctrlGenericPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  ctrlVelYawratePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
  ctrlRollPitchYawZPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10);
  
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  
  bool obtain_control_result = obtain_control();
  bool takeoff_result, landing_result;

  
  //Main steps function calls
  
  //STEP 1: Take off
  takeoff_result = M100monitoredTakeoff();

  //STEP 2: Waiting
  ros::Duration(2).sleep();  
  
  
  
  //STEP 3: Landing
  landing_result = M100monitoredLanding();

  ros::spin();
  return 0;
}
