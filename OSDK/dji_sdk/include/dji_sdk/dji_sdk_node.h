/** @file dji_sdk_node.h
 *  @version 3.7
 *  @date July, 2018
 *
 *  @brief
 *  A ROS wrapper to interact with DJI onboard SDK
 *
 *  @copyright 2018 DJI. All rights reserved.
 *
 */

#ifndef DJI_SDK_NODE_MAIN_H
#define DJI_SDK_NODE_MAIN_H

//! ROS
#include <ros/ros.h>
#include <tf/tf.h>

//! ROS standard msgs
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

//! msgs
#include <dji_sdk/Gimbal.h>
#include <dji_sdk/FlightAnomaly.h>
#include <dji_sdk/VOPosition.h>


//! service headers
#include <dji_sdk/Activation.h>
#include <dji_sdk/CameraAction.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/QueryDroneVersion.h>


//! SDK library
#include <djiosdk/dji_vehicle.hpp>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

using namespace DJI::OSDK;

class DJISDKNode
{
public:
  DJISDKNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~DJISDKNode();

  enum TELEMETRY_TYPE
  {
    USE_BROADCAST = 0,
    USE_SUBSCRIBE = 1
  };

  enum
  {
    PACKAGE_ID_5HZ   = 0,
    PACKAGE_ID_50HZ  = 1,
    PACKAGE_ID_100HZ = 2,
    PACKAGE_ID_400HZ = 3
  };

private:
  bool initVehicle(ros::NodeHandle& nh_private);
  bool initServices(ros::NodeHandle& nh);
  bool initFlightControl(ros::NodeHandle& nh);
  bool initSubscriber(ros::NodeHandle& nh);
  bool initPublisher(ros::NodeHandle& nh);
  bool initActions(ros::NodeHandle& nh);
  bool initDataSubscribeFromFC(ros::NodeHandle& nh);
  void cleanUpSubscribeFromFC();
  bool validateSerialDevice(LinuxSerialDevice* serialDevice);
  bool isM100();

  /*!
   * @note this function exists here instead of inside the callback function
   *        due to the usages, i.e. we not only provide service call but also
   *        call it for the user when this node was instantiated
   *        we cannot call a service without serviceClient, which is in another
   * node
   */
  ACK::ErrorCode activate(int l_app_id, std::string l_enc_key);

  //! flight control subscriber callbacks
  void flightControlSetpointCallback(
    const sensor_msgs::Joy::ConstPtr& pMsg);

  void flightControlPxPyPzYawCallback(
    const sensor_msgs::Joy::ConstPtr& pMsg);

  void flightControlVxVyVzYawrateCallback(
    const sensor_msgs::Joy::ConstPtr& pMsg);

  void flightControlRollPitchPzYawrateCallback(
    const sensor_msgs::Joy::ConstPtr& pMsg);

  //! general subscriber callbacks
  void gimbalAngleCtrlCallback(const dji_sdk::Gimbal::ConstPtr& msg);
  void gimbalSpeedCtrlCallback(
    const geometry_msgs::Vector3Stamped::ConstPtr& msg);

  //! general service callbacks
  bool droneActivationCallback(dji_sdk::Activation::Request&  request,
                               dji_sdk::Activation::Response& response);
  bool sdkCtrlAuthorityCallback(
    dji_sdk::SDKControlAuthority::Request&  request,
    dji_sdk::SDKControlAuthority::Response& response);

  bool setLocalPosRefCallback(
      dji_sdk::SetLocalPosRef::Request&  request,
      dji_sdk::SetLocalPosRef::Response& response);
  //! control service callbacks
  bool droneArmCallback(dji_sdk::DroneArmControl::Request&  request,
                        dji_sdk::DroneArmControl::Response& response);
  bool droneTaskCallback(dji_sdk::DroneTaskControl::Request&  request,
                         dji_sdk::DroneTaskControl::Response& response);


  //! Query Drone FW version
  bool queryVersionCallback(dji_sdk::QueryDroneVersion::Request& request,
                            dji_sdk::QueryDroneVersion::Response& response);

  bool cameraActionCallback(dji_sdk::CameraAction::Request&  request,
                            dji_sdk::CameraAction::Response& response);
  


  //! data broadcast callback
  void dataBroadcastCallback();

  static void SDKBroadcastCallback(Vehicle*            vehicle,
                                   RecvContainer       recvFrame,
                                   DJI::OSDK::UserData userData);

  static void publish5HzData(Vehicle*            vehicle,
                              RecvContainer       recvFrame,
                              DJI::OSDK::UserData userData);

  static void publish50HzData(Vehicle*            vehicle,
                              RecvContainer       recvFrame,
                              DJI::OSDK::UserData userData);

  static void publish100HzData(Vehicle*            vehicle,
                               RecvContainer       recvFrame,
                               DJI::OSDK::UserData userData);


private:
  //! OSDK core
  Vehicle* vehicle;
  //! general service servers
  ros::ServiceServer drone_activation_server;
  ros::ServiceServer sdk_ctrlAuthority_server;
  ros::ServiceServer camera_action_server;
  //! flight control service servers
  ros::ServiceServer drone_arm_server;
  ros::ServiceServer drone_task_server;
  //! Query FW version of FC
  ros::ServiceServer query_version_server;
  //! Set Local position reference
  ros::ServiceServer local_pos_ref_server;
  //! flight control subscribers
  ros::Subscriber flight_control_sub;
  ros::Subscriber flight_control_position_yaw_sub;
  ros::Subscriber flight_control_velocity_yawrate_sub;
  ros::Subscriber flight_control_rollpitch_yawrate_vertpos_sub;
  //! general subscribers
  ros::Subscriber gimbal_angle_cmd_subscriber;
  ros::Subscriber gimbal_speed_cmd_subscriber;
  //! telemetry data publisher
  ros::Publisher attitude_publisher;
  ros::Publisher angularRate_publisher;
  ros::Publisher acceleration_publisher;
  ros::Publisher battery_state_publisher;
  ros::Publisher trigger_publisher;
  ros::Publisher imu_publisher;
  ros::Publisher flight_status_publisher;
  ros::Publisher gps_health_publisher;
  ros::Publisher gps_position_publisher;
  ros::Publisher vo_position_publisher;
  ros::Publisher height_publisher;
  ros::Publisher velocity_publisher;
  ros::Publisher gimbal_angle_publisher;
  ros::Publisher rc_publisher;
  ros::Publisher rc_connection_status_publisher;
  ros::Publisher flight_anomaly_publisher;
  //! Local Position Publisher (Publishes local position in ENU frame)
  ros::Publisher local_position_publisher;
  ros::Publisher local_frame_ref_publisher;

  //! constant
  const int WAIT_TIMEOUT           = 10;
  const int MAX_SUBSCRIBE_PACKAGES = 5;
  const int INVALID_VERSION        = 0;

  //! configurations
  int         app_id;
  std::string enc_key;
  std::string drone_version;
  std::string serial_device;
  int         baud_rate;
  int         app_version;
  std::string app_bundle_id; // reserved
  int         uart_or_usb;
  double      gravity_const;

  //! use broadcast or subscription to get telemetry data
  TELEMETRY_TYPE telemetry_from_fc;
  bool stereo_subscription_success = false;
  bool stereo_vga_subscription_success = false;
  bool user_select_broadcast;
  const tf::Matrix3x3 R_FLU2FRD;
  const tf::Matrix3x3 R_ENU2NED;

  void flightControl(uint8_t flag, float32_t xSP, float32_t ySP, float32_t zSP, float32_t yawSP);

  enum AlignState
  {
    UNALIGNED,
    ALIGNING,
    ALIGNED
  };

  AlignState curr_align_state;

  static int constexpr STABLE_ALIGNMENT_COUNT = 400;
  static double constexpr TIME_DIFF_CHECK = 0.008;
  static double constexpr TIME_DIFF_ALERT = 0.020;

  ros::Time base_time;

  bool align_time_with_FC;

  bool local_pos_ref_set;

  void alignRosTimeWithFlightController(ros::Time now_time, uint32_t tick);
  void setUpM100DefaultFreq(uint8_t freq[16]);
  void gpsConvertENU(double &ENU_x, double &ENU_y,
                     double gps_t_lon, double gps_t_lat,
                     double gps_r_lon, double gps_r_lat);

  double local_pos_ref_latitude, local_pos_ref_longitude, local_pos_ref_altitude;
  double current_gps_latitude, current_gps_longitude, current_gps_altitude;
  int current_gps_health;
  bool rtkSupport = false;
};

#endif // DJI_SDK_NODE_MAIN_H
