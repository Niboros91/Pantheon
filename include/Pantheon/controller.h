/*
 * mission_controller_v2.h
 *
 *  Created on: Feb 6, 2018
 *      Author: nicolas
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_


// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt8.h>
#include "std_msgs/String.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <stdlib.h>
#include "dji_sdk/path.h"

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>

#include "std_msgs/String.h"
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream> //added
#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

/*!
 * @brief a bare bone state machine to track the stage of the mission
 */
class Mission
{
public:
  // The basic state transition flow is:
  // 0---> 1 ---> 2 ---> ... ---> N ---> 0
  // where state 0 means the mission is note started
  // and each state i is for the process of moving to a target point.
  int state;
    
  int inbound_counter;
  int outbound_counter;
  int break_counter;
  int picture_bound;

  float target_offset_x;
  float target_offset_y;
  float target_offset_z;
  float target_yaw;
  sensor_msgs::NavSatFix start_gps_location;
  geometry_msgs::Point start_local_position;
  std::vector<double> Offset_check;

  bool finished;
  bool Landing_result;
  bool passed;

  Mission() : state(0), inbound_counter(0), outbound_counter(0), picture_bound(0), break_counter(0), target_offset_x(0.0), target_offset_y(0.0), target_offset_z(0.0),
              finished(false), Landing_result(false),Offset_check(4,0)
  {
  }

  void step();
    
  void sensing();

  bool monitoredLanding();

  void setTarget(float x, float y, float z, float yaw)
  {
    target_offset_x = x;
    target_offset_y = y;
    target_offset_z = z;
    target_yaw      = yaw;
  }

  void reset()
  {
    inbound_counter = 0;
    outbound_counter = 0;
    break_counter = 0;
    picture_bound =0;
    finished = false;
    passed=false;
    

  }

};



//End of the class mission

void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin);
// More things have to be checked
//void lineOffset(float a,sensor_msgs::NavSatFix& start);

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

void pathCallback(const dji_sdk::path::ConstPtr& msg);

bool takeoff_land(int task);

bool obtain_control();

bool is_M100();

bool monitoredTakeoff();

//void path_callback();  Function callback for the path sent from matlab

bool set_local_position();

#endif /* CONTROLLER_H_ */
