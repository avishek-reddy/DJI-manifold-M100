//DJI SDK includes
#include "flight_control.h"
#include "dji_sdk/dji_sdk.h"


//ROS services
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;


//ROS publishers
ros::Publisher ctrlGenericPub;
ros::Publisher ctrlRollPitchYawZPub;


// global variables for subscribed topics
const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

geometry_msgs::Quaternion current_attitude;
float current_battery;
sensor_msgs::Imu current_imu;
uint8_t current_flight_status = 255;
float current_height_above_takeoff  = 255;


// Callback Functions
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_attitude = msg->quaternion;
}

void battery_state_callback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
  current_battery = msg->percentage;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  current_flight_status = msg->data;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  current_imu = *msg;
}

void height_above_takeoff_callback(const std_msgs::Float32::ConstPtr& msg)
{
  current_height_above_takeoff = msg->data;
}

//Helper functions

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("Failed to obtain control of the aircraft");
    return false;
  }
  ROS_INFO("Obtained control of aircraft");
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
  ROS_INFO("Aircraft taking off.");
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(current_flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR)
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

  // If M100 didn't land in 6 secs, fail
  while (ros::Time::now() - start_time < ros::Duration(6))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
	
  ROS_INFO("Successful landing.");  
  return true;
}




/////////////////////  MAIN  ////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flight_control_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber batteryStateSub = nh.subscribe("dji_sdk/battery_state", 10, &battery_state_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber imuSub = nh.subscribe("dji_sdk/imu", 10, &imu_callback);
  ros::Subscriber heightAboveTakeoffSub = nh.subscribe("dji_sdk/height_above_takeoff", 10, &height_above_takeoff_callback);

  // Publish the control signal
  ctrlGenericPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  ctrlRollPitchYawZPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10);
  
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  
  bool obtain_control_result = false , takeoff_result = false , landing_result = false;
  ros::Rate loop_rate(50);
  
  // Wait till battery levels are received
  while(current_battery == 0)
  {
    ros::spinOnce();
  }


  // obtain control
  obtain_control_result = obtain_control();
  
  // Take off 
  if(current_battery < 30.00)
  {
    ROS_ERROR("Battery below 30 percent, operation terminated");
  }
  else if(!obtain_control_result)
  {
    ROS_ERROR("Failed to obtain aircraft control");
  }
  else
  {
    takeoff_result = M100monitoredTakeoff();
  }
  ros::Duration(2).sleep();

  //temp
  static ros::Time start_time = ros::Time::now();

  // infinite loop
  while(ros::ok && takeoff_result)
  {
    
    // Monitor battery percentage > 15%
    if(current_battery < 15)
    {
      ROS_ERROR("Battery below 15 percent, landing");
      break;
    }

    ROS_INFO("Start - hover ");
    while(ros::ok && (ros::Time::now() - start_time < ros::Duration(3)))
    {
      uint8_t flag = (DJISDK::VERTICAL_POSITION |
                  DJISDK::HORIZONTAL_ANGLE |
                  DJISDK::YAW_RATE |
                  DJISDK::HORIZONTAL_BODY |
                  DJISDK::STABLE_ENABLE);

      sensor_msgs::Joy ctrlCmds;
      ctrlCmds.axes.push_back(0); 	//X
      ctrlCmds.axes.push_back(0);	//Y
      ctrlCmds.axes.push_back(1.2);	//Z
      ctrlCmds.axes.push_back(0);	//Yaw
      ctrlCmds.axes.push_back(flag);	//flag
      ctrlGenericPub.publish(ctrlCmds);
      loop_rate.sleep();
    }
    ROS_INFO("End - hover");


    ROS_INFO("Start - Move left ");
    // Move forward 
    while(ros::ok && (ros::Time::now() - start_time < ros::Duration(4)))
    {
      uint8_t flag = (DJISDK::VERTICAL_POSITION |
                  DJISDK::HORIZONTAL_ANGLE |
                  DJISDK::YAW_RATE |
                  DJISDK::HORIZONTAL_BODY |
                  DJISDK::STABLE_ENABLE);

      sensor_msgs::Joy ctrlCmds;
      ctrlCmds.axes.push_back(-0.02); 	//X
      ctrlCmds.axes.push_back(0);	//Y
      ctrlCmds.axes.push_back(1.2);	//Z
      ctrlCmds.axes.push_back(0);	//Yaw
      ctrlCmds.axes.push_back(flag);	//flag
      ctrlGenericPub.publish(ctrlCmds);
      loop_rate.sleep();
    }
    ROS_INFO("End - Move left");


    ROS_INFO("Start - hover ");
    while(ros::ok && (ros::Time::now() - start_time < ros::Duration(7)))
    {
      uint8_t flag = (DJISDK::VERTICAL_POSITION |
                  DJISDK::HORIZONTAL_ANGLE |
                  DJISDK::YAW_RATE |
                  DJISDK::HORIZONTAL_BODY |
                  DJISDK::STABLE_ENABLE);

      sensor_msgs::Joy ctrlCmds;
      ctrlCmds.axes.push_back(0); 	//X
      ctrlCmds.axes.push_back(0);	//Y
      ctrlCmds.axes.push_back(1.2);	//Z
      ctrlCmds.axes.push_back(0);	//Yaw
      ctrlCmds.axes.push_back(flag);	//flag
      ctrlGenericPub.publish(ctrlCmds);
      loop_rate.sleep();
    }
    ROS_INFO("End - hover");

    ROS_INFO("Start - move left");
    while(ros::ok && (ros::Time::now() - start_time < ros::Duration(8)))
    {
      uint8_t flag = (DJISDK::VERTICAL_POSITION |
                  DJISDK::HORIZONTAL_ANGLE |
                  DJISDK::YAW_RATE |
                  DJISDK::HORIZONTAL_BODY |
                  DJISDK::STABLE_ENABLE);

      sensor_msgs::Joy ctrlCmds;
      ctrlCmds.axes.push_back(-0.02); 	//X
      ctrlCmds.axes.push_back(0);	//Y
      ctrlCmds.axes.push_back(1.2);	//Z
      ctrlCmds.axes.push_back(0);	//Yaw
      ctrlCmds.axes.push_back(flag);	//flag
      ctrlGenericPub.publish(ctrlCmds);
      loop_rate.sleep();
    }
    ROS_INFO("End - move left");
   
    break;
    /*
    //temp
    if(ros::Time::now() - start_time > ros::Duration(5))
    {
      ROS_INFO("5 secs done");
      break;
    }
    */
  }  
  /*
  static ros::Time start_time = ros::Time::now();

  ros::Rate r(50);
  
  while(ros::ok)
  {
    sensor_msgs::Joy ctrlCmds;

    ctrlCmds.axes.push_back(0);
    ctrlCmds.axes.push_back(0);
    ctrlCmds.axes.push_back(1.2);
    ctrlCmds.axes.push_back(0);

    ctrlRollPitchYawZPub.publish(ctrlCmds);
    
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    
    r.sleep();
    
    if(elapsed_time > ros::Duration(2))
    break;
  }
  */
  
  ros::Duration(2).sleep();
  //STEP 3: Landing
  landing_result = M100monitoredLanding();

  ros::spin();
  return 0;
}
