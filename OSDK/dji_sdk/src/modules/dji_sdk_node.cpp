/** @file dji_sdk_node.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  Implementation of the initialization functions of DJISDKNode
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

using namespace DJI::OSDK;

DJISDKNode::DJISDKNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : telemetry_from_fc(USE_BROADCAST),
    R_FLU2FRD(tf::Matrix3x3(1,  0,  0, 0, -1,  0, 0,  0, -1)),
    R_ENU2NED(tf::Matrix3x3(0,  1,  0, 1,  0,  0, 0,  0, -1)),
    curr_align_state(UNALIGNED)
{
  nh_private.param("serial_name",   serial_device, std::string("/dev/ttyUSB0"));
  nh_private.param("baud_rate",     baud_rate, 921600);
  nh_private.param("app_id",        app_id,    123456);
  nh_private.param("app_version",   app_version, 1);
  nh_private.param("enc_key",       enc_key, std::string("abcd1234"));
  nh_private.param("drone_version", drone_version, std::string("M100")); // choose M100 as default
  nh_private.param("gravity_const", gravity_const, 9.801);
  nh_private.param("align_time",    align_time_with_FC, false);
  nh_private.param("use_broadcast", user_select_broadcast, false);

  //! Default values for local Position
  local_pos_ref_latitude  = 0;
  local_pos_ref_longitude = 0;
  local_pos_ref_altitude  = 0;
  local_pos_ref_set       = false;

  //! RTK support check
  rtkSupport = false;

  // @todo need some error handling for init functions
  //! @note parsing launch file to get environment parameters
  if (!initVehicle(nh_private))
  {
    ROS_ERROR("Vehicle initialization failed");
  }

  else
  {
    if (!initServices(nh))
    {
      ROS_ERROR("initServices failed");
    }

    if (!initFlightControl(nh))
    {
      ROS_ERROR("initFlightControl failed");
    }

    if (!initSubscriber(nh))
    {
      ROS_ERROR("initSubscriber failed");
    }

    if (!initPublisher(nh))
    {
      ROS_ERROR("initPublisher failed");
    }
  }
}

DJISDKNode::~DJISDKNode()
{
  if(!isM100())
  {
    cleanUpSubscribeFromFC();
  }
  if (vehicle)
  {
    delete vehicle;
  }
}

bool
DJISDKNode::initVehicle(ros::NodeHandle& nh_private)
{
  bool threadSupport = true;
  bool enable_advanced_sensing = false;



  //! @note currently does not work without thread support
  vehicle = new Vehicle(serial_device.c_str(), baud_rate, threadSupport, enable_advanced_sensing);

  /*!
   * @note activate the drone for the user at the beginning
   *        user can also call it as a service
   *        this has been tested by giving wrong appID in launch file
   */
  if (ACK::getError(this->activate(this->app_id, this->enc_key)))
  {
    ROS_ERROR("drone activation error");
    return false;
  }
  ROS_INFO("drone activated");

  // This version of ROS Node works for:
  //    1. A3/N3/M600 with latest FW
  //    2. M100 with FW version M100_31
  if(vehicle->getFwVersion() > INVALID_VERSION
      && vehicle->getFwVersion() < mandatoryVersionBase
      && (!isM100()))
  {
    return false;
  }


  if (NULL != vehicle->subscribe && (!user_select_broadcast))
  {
    telemetry_from_fc = USE_SUBSCRIBE;
  }

  return true;
}

// clang-format off
bool DJISDKNode::initServices(ros::NodeHandle& nh) {
  // Common to A3/N3 and M100
  drone_activation_server   = nh.advertiseService("dji_sdk/activation",                     &DJISDKNode::droneActivationCallback,        this);
  drone_arm_server          = nh.advertiseService("dji_sdk/drone_arm_control",              &DJISDKNode::droneArmCallback,               this);
  drone_task_server         = nh.advertiseService("dji_sdk/drone_task_control",             &DJISDKNode::droneTaskCallback,              this);
  sdk_ctrlAuthority_server  = nh.advertiseService("dji_sdk/sdk_control_authority",          &DJISDKNode::sdkCtrlAuthorityCallback,       this);
  camera_action_server      = nh.advertiseService("dji_sdk/camera_action",                  &DJISDKNode::cameraActionCallback,           this);
  query_version_server      = nh.advertiseService("dji_sdk/query_drone_version",            &DJISDKNode::queryVersionCallback,           this);
  local_pos_ref_server      = nh.advertiseService("dji_sdk/set_local_pos_ref",              &DJISDKNode::setLocalPosRefCallback,         this);

  return true;
}
// clang-format on

bool
DJISDKNode::initFlightControl(ros::NodeHandle& nh)
{
  flight_control_sub = nh.subscribe<sensor_msgs::Joy>(
    "dji_sdk/flight_control_setpoint_generic", 10, 
    &DJISDKNode::flightControlSetpointCallback,   this);

  flight_control_position_yaw_sub =
    nh.subscribe<sensor_msgs::Joy>(
      "dji_sdk/flight_control_setpoint_ENUposition_yaw", 10,
      &DJISDKNode::flightControlPxPyPzYawCallback, this);

  flight_control_velocity_yawrate_sub =
    nh.subscribe<sensor_msgs::Joy>(
      "dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10,
      &DJISDKNode::flightControlVxVyVzYawrateCallback, this);

  flight_control_rollpitch_yawrate_vertpos_sub =
    nh.subscribe<sensor_msgs::Joy>(
      "dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10,
      &DJISDKNode::flightControlRollPitchPzYawrateCallback, this);

  return true;
}

bool DJISDKNode::isM100()
{
  return(vehicle->isM100());
}


ACK::ErrorCode
DJISDKNode::activate(int l_app_id, std::string l_enc_key)
{
  usleep(1000000);
  Vehicle::ActivateData testActivateData;
  char                  app_key[65];
  testActivateData.encKey = app_key;
  strcpy(testActivateData.encKey, l_enc_key.c_str());
  testActivateData.ID = l_app_id;

  ROS_DEBUG("called vehicle->activate(&testActivateData, WAIT_TIMEOUT)");
  return vehicle->activate(&testActivateData, WAIT_TIMEOUT);
}

bool
DJISDKNode::initSubscriber(ros::NodeHandle& nh)
{
  gimbal_angle_cmd_subscriber = nh.subscribe<dji_sdk::Gimbal>(
    "dji_sdk/gimbal_angle_cmd", 10, &DJISDKNode::gimbalAngleCtrlCallback, this);
  gimbal_speed_cmd_subscriber = nh.subscribe<geometry_msgs::Vector3Stamped>(
    "dji_sdk/gimbal_speed_cmd", 10, &DJISDKNode::gimbalSpeedCtrlCallback, this);
  return true;
}

bool
DJISDKNode::initPublisher(ros::NodeHandle& nh)
{
  rc_publisher = nh.advertise<sensor_msgs::Joy>("dji_sdk/rc", 10);

  attitude_publisher =
    nh.advertise<geometry_msgs::QuaternionStamped>("dji_sdk/attitude", 10);

  battery_state_publisher =
    nh.advertise<sensor_msgs::BatteryState>("dji_sdk/battery_state",10);

  /*!
   * - Fused attitude (duplicated from attitude topic)
   * - Raw linear acceleration (body frame: FLU, m/s^2)
   *       Z value is +9.8 when placed on level ground statically
   * - Raw angular velocity (body frame: FLU, rad/s^2)
   */
  imu_publisher = nh.advertise<sensor_msgs::Imu>("dji_sdk/imu", 10);

  // Refer to dji_sdk.h for different enums for M100 and A3/N3
  flight_status_publisher =
    nh.advertise<std_msgs::UInt8>("dji_sdk/flight_status", 10);

  /*!
   * gps_health needs to be greater than 3 for gps_position and velocity topics
   * to be trusted
   */
  gps_health_publisher =
    nh.advertise<std_msgs::UInt8>("dji_sdk/gps_health", 10);

  /*!
   * NavSatFix specs:
   *   Latitude [degrees]. Positive is north of equator; negative is south.
   *   Longitude [degrees]. Positive is east of prime meridian; negative is
   * west.
   *   Altitude [m]. Positive is above the WGS 84 ellipsoid
   */
  gps_position_publisher =
    nh.advertise<sensor_msgs::NavSatFix>("dji_sdk/gps_position", 10);

  /*!
   *   x [m]. Positive along navigation frame x axis
   *   y [m]. Positive along navigation frame y axis
   *   z [m]. Positive is down
   *   For details about navigation frame, please see telemetry documentation in API reference
  */
  vo_position_publisher =
          nh.advertise<dji_sdk::VOPosition>("dji_sdk/vo_position", 10);
  /*!
   * Height above home altitude. It is valid only after drone
   * is armed.
   */
  height_publisher =
    nh.advertise<std_msgs::Float32>("dji_sdk/height_above_takeoff", 10);

  velocity_publisher =
    nh.advertise<geometry_msgs::Vector3Stamped>("dji_sdk/velocity", 10);


  // TODO: documentation and proper frame id
  gimbal_angle_publisher =
    nh.advertise<geometry_msgs::Vector3Stamped>("dji_sdk/gimbal_angle", 10);

  local_position_publisher =
      nh.advertise<geometry_msgs::PointStamped>("dji_sdk/local_position", 10);

  local_frame_ref_publisher =
      nh.advertise<sensor_msgs::NavSatFix>("dji_sdk/local_frame_ref", 10, true);


  if (telemetry_from_fc == USE_BROADCAST)
  {
    ACK::ErrorCode broadcast_set_freq_ack;
    ROS_INFO("Use legacy data broadcast to get telemetry data!");

    uint8_t defaultFreq[16];

    if(isM100())
    {
      setUpM100DefaultFreq(defaultFreq);
    }
   

    broadcast_set_freq_ack =
      vehicle->broadcast->setBroadcastFreq(defaultFreq, WAIT_TIMEOUT);
//      vehicle->broadcast->setBroadcastFreqDefaults(WAIT_TIMEOUT);

    if (ACK::getError(broadcast_set_freq_ack))
    {
      ACK::getErrorCodeMessage(broadcast_set_freq_ack, __func__);
      return false;
    }
    // register a callback function whenever a broadcast data is in
    vehicle->broadcast->setUserBroadcastCallback(
      &DJISDKNode::SDKBroadcastCallback, this);
  }
  else if (telemetry_from_fc == USE_SUBSCRIBE)
  {
    ROS_INFO("Use data subscription to get telemetry data!");
    if(!align_time_with_FC)
    {
      ROS_INFO("align_time_with_FC set to false. We will use ros time to time stamp messages!");
    }
    else
    {
      ROS_INFO("align_time_with_FC set to true. We will time stamp messages based on flight controller time!");
    }
    

    if (!initDataSubscribeFromFC(nh))
    {
      return false;
    }
  }
  return true;
}

bool
DJISDKNode::initDataSubscribeFromFC(ros::NodeHandle& nh)
{
  ACK::ErrorCode ack = vehicle->subscribe->verify(WAIT_TIMEOUT);
  if (ACK::getError(ack))
  {
    return false;
  }

  std::vector<Telemetry::TopicName> topicList100Hz;
  topicList100Hz.push_back(Telemetry::TOPIC_QUATERNION);
  topicList100Hz.push_back(Telemetry::TOPIC_ACCELERATION_GROUND);
  topicList100Hz.push_back(Telemetry::TOPIC_ANGULAR_RATE_FUSIONED);

  int nTopic100Hz    = topicList100Hz.size();
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_100HZ, nTopic100Hz,
                                                   topicList100Hz.data(), 1, 100))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_100HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_100HZ, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 100Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
              PACKAGE_ID_100HZ, publish100HzData, this);
    }
  }

  std::vector<Telemetry::TopicName> topicList50Hz;
  // 50 Hz package from FC
  topicList50Hz.push_back(Telemetry::TOPIC_GPS_FUSED);
  topicList50Hz.push_back(Telemetry::TOPIC_ALTITUDE_FUSIONED);
  topicList50Hz.push_back(Telemetry::TOPIC_HEIGHT_FUSION);
  topicList50Hz.push_back(Telemetry::TOPIC_STATUS_FLIGHT);
  topicList50Hz.push_back(Telemetry::TOPIC_STATUS_DISPLAYMODE);
  topicList50Hz.push_back(Telemetry::TOPIC_GIMBAL_ANGLES);
  topicList50Hz.push_back(Telemetry::TOPIC_GIMBAL_STATUS);
  topicList50Hz.push_back(Telemetry::TOPIC_RC);
  topicList50Hz.push_back(Telemetry::TOPIC_VELOCITY);
  topicList50Hz.push_back(Telemetry::TOPIC_GPS_CONTROL_LEVEL);

  if(vehicle->getFwVersion() > versionBase33)
  {
    topicList50Hz.push_back(Telemetry::TOPIC_POSITION_VO);
    topicList50Hz.push_back(Telemetry::TOPIC_RC_WITH_FLAG_DATA);
    topicList50Hz.push_back(Telemetry::TOPIC_FLIGHT_ANOMALY);

    // A3 and N3 has access to more buttons on RC
    std::string hardwareVersion(vehicle->getHwVersion());
    if( (hardwareVersion == std::string(Version::N3)) || hardwareVersion == std::string(Version::A3))
    {
      topicList50Hz.push_back(Telemetry::TOPIC_RC_FULL_RAW_DATA);      
    }

    // Advertise rc connection status only if this topic is supported by FW
    rc_connection_status_publisher =
            nh.advertise<std_msgs::UInt8>("dji_sdk/rc_connection_status", 10);

    flight_anomaly_publisher =
            nh.advertise<dji_sdk::FlightAnomaly>("dji_sdk/flight_anomaly", 10);
  }

  int nTopic50Hz    = topicList50Hz.size();
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_50HZ, nTopic50Hz,
                                                   topicList50Hz.data(), 1, 50))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_50HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_50HZ, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 50Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
              PACKAGE_ID_50HZ, publish50HzData, (UserData) this);
    }
  }

  //! Check if RTK is supported in the FC
  Telemetry::TopicName topicRTKSupport[] =
  {
    Telemetry::TOPIC_RTK_POSITION
  };


  std::vector<Telemetry::TopicName> topicList5hz;
  topicList5hz.push_back(Telemetry::TOPIC_GPS_DATE);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_TIME);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_POSITION);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_VELOCITY);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_DETAILS);
  topicList5hz.push_back(Telemetry::TOPIC_BATTERY_INFO);


  int nTopic5hz    = topicList5hz.size();
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_5HZ, nTopic5hz,
                                                   topicList5hz.data(), 1, 5))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_5HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_5HZ, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 5hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
              PACKAGE_ID_5HZ, publish5HzData, (UserData) this);
    }
  }


  ros::Duration(1).sleep();
  return true;
}

void
DJISDKNode::cleanUpSubscribeFromFC()
{
  vehicle->subscribe->removePackage(0, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(1, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(2, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(3, WAIT_TIMEOUT);
}

bool DJISDKNode::validateSerialDevice(LinuxSerialDevice* serialDevice)
{
  static const int BUFFER_SIZE = 2048;
  //! Check the serial channel for data
  uint8_t buf[BUFFER_SIZE];
  if (!serialDevice->setSerialPureTimedRead())
  {
    ROS_ERROR("Failed to set up port for timed read.\n");
    return (false);
  };
  usleep(100000);
  if(serialDevice->serialRead(buf, BUFFER_SIZE))
  {
    ROS_INFO("Succeeded to read from serial device");
  }
  else
  {
    ROS_ERROR("Failed to read from serial device. The Onboard SDK is not communicating with your drone.");
    return (false);
  }

  // All the tests passed and the serial device is properly set up
  serialDevice->unsetSerialPureTimedRead();
  return (true);
}

void
DJISDKNode::setUpM100DefaultFreq(uint8_t freq[16])
{
  freq[0]  = DataBroadcast::FREQ_100HZ;
  freq[1]  = DataBroadcast::FREQ_100HZ;
  freq[2]  = DataBroadcast::FREQ_100HZ;
  freq[3]  = DataBroadcast::FREQ_50HZ;
  freq[4]  = DataBroadcast::FREQ_100HZ;
  freq[5]  = DataBroadcast::FREQ_50HZ;
  freq[6]  = DataBroadcast::FREQ_10HZ;
  freq[7]  = DataBroadcast::FREQ_50HZ;
  freq[8]  = DataBroadcast::FREQ_50HZ;
  freq[9]  = DataBroadcast::FREQ_50HZ;
  freq[10] = DataBroadcast::FREQ_10HZ;
  freq[11] = DataBroadcast::FREQ_10HZ;
}


void DJISDKNode::gpsConvertENU(double &ENU_x, double &ENU_y,
                                 double gps_t_lon, double gps_t_lat,
                                 double gps_r_lon, double gps_r_lat)
{
  double d_lon = gps_t_lon - gps_r_lon;
  double d_lat = gps_t_lat - gps_r_lat;
  ENU_y = DEG2RAD(d_lat) * C_EARTH;
  ENU_x = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(gps_t_lat));
};
