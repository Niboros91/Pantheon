//
//  path_planner.h
//  
//
//  Created by Nicolás Bono Rosselló on 4/4/18.
//

#ifndef planner_h
#define planner_h

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>


#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#define C_PI (double)3.141592653589793

// We define our camera and coordinates class
class Camera {
public:
    int FOV_x, FOV_y,RES_x,RES_y;
};

class Coordinates {
public:
    float x, y;
};

class Dual{
public:
    int x, y;
};
/*!
 * @brief a bare bone state machine to track the stage of the mission
 */
float  resolution, altitude,  X_picture, Y_picture, Dist_picture_x, Dist_picture_y, x, y; 
int Total_pictures, Lines,sector_x, sector_y, change;
int num_pictures=0; 
int num_lines=0;





#endif /* planner_h */
