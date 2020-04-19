/** @file dji_sdk.h
 *  @version 3.7
 *  @date July, 2018
 *
 *  @brief
 *  Definitions and Enums for client code to use dji_sdk ros wrapper
 *
 *  @copyright 2018 DJI. All rights reserved.
 *
 */

#ifndef PROJECT_DJI_SDK_H_H
#define PROJECT_DJI_SDK_H_H
#include <djiosdk/dji_control.hpp>
#include <djiosdk/dji_status.hpp>
#include <djiosdk/dji_version.hpp>
namespace DJISDK {

/*!
 * This enum is used with service query_drone_version to
 * check if the drone is M100 or not. We only support
 * M100 with this particular FW version.
 */
enum DroneFirmwareVersion
{
  M100_31 = DJI::OSDK::Version::M100_31,
};

typedef enum AircraftVersion
{
  UNKNOWN,
  M100,
  M600,
  A3,
  N3,
  M210
} AircraftVersion;

enum FlightControlFlag
{
  HORIZONTAL_ANGLE         = DJI::OSDK::Control::HORIZONTAL_ANGLE,
  HORIZONTAL_VELOCITY      = DJI::OSDK::Control::HORIZONTAL_VELOCITY,
  HORIZONTAL_POSITION      = DJI::OSDK::Control::HORIZONTAL_POSITION,
  // Horizontal angular rate is supported only by A3/N3 based platform
  // and is NOT supported by M100
  HORIZONTAL_ANGULAR_RATE  = DJI::OSDK::Control::HORIZONTAL_ANGULAR_RATE,

  VERTICAL_VELOCITY = DJI::OSDK::Control::VERTICAL_VELOCITY,
  VERTICAL_POSITION = DJI::OSDK::Control::VERTICAL_POSITION,
  VERTICAL_THRUST   = DJI::OSDK::Control::VERTICAL_THRUST,

  YAW_ANGLE = DJI::OSDK::Control::YAW_ANGLE,
  YAW_RATE  = DJI::OSDK::Control::YAW_RATE,

  HORIZONTAL_GROUND = DJI::OSDK::Control::HORIZONTAL_GROUND,
  HORIZONTAL_BODY   = DJI::OSDK::Control::HORIZONTAL_BODY,

  STABLE_DISABLE = DJI::OSDK::Control::STABLE_DISABLE,
  STABLE_ENABLE  = DJI::OSDK::Control::STABLE_ENABLE
};


enum FlightStatus
{
  STATUS_STOPPED   = DJI::OSDK::VehicleStatus::FlightStatus::STOPED,
  STATUS_ON_GROUND = DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND,
  STATUS_IN_AIR    = DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR
};

enum M100FlightStatus
{
  M100_STATUS_ON_GROUND        = DJI::OSDK::VehicleStatus::M100FlightStatus::ON_GROUND_STANDBY,
  M100_STATUS_TAKINGOFF        = DJI::OSDK::VehicleStatus::M100FlightStatus::TAKEOFF,
  M100_STATUS_IN_AIR           = DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY,
  M100_STATUS_LANDING          = DJI::OSDK::VehicleStatus::M100FlightStatus::LANDING,
  M100_STATUS_FINISHED_LANDING = DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING
};

}


#endif //PROJECT_DJI_SDK_H_H
