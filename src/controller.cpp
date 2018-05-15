/*
 * mission_controller_local_coordinates.cpp
 *
 *  Created on: Feb 6, 2018
 *      Author: nicolas
 */
/** @file controller.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "Pantheon/controller.h"
#include "dji_sdk/dji_sdk.h"
#include "dji_sdk/path.h"  //To include the msg created
#include "dji_sdk/picture.h"  //To include the msg created

std::vector<float> loc(2, 0);
std::vector<float> loc_pic(2, 0);

//Values that we get from the messages
std::vector<float> pos(0);
std::vector<float> line(0);
int points, lines;
bool check=false;
float altitude;

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlBrakePub;
ros::Publisher PicturePub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
sensor_msgs::Imu current_imu;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;
geometry_msgs::Vector3Stamped current_velocity;
dji_sdk::picture pic;
int i=0;
int j=0;
int state=0;
int counter=0;
int picture_counter=0;

Mission square_mission, picture_mission;

//Defintion of the message
	std_msgs::Float32MultiArray dat;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_controller");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber imuSub = nh.subscribe("dji_sdk/imu",10, &imu_callback);
  ros::Subscriber velocutySub = nh.subscribe("dji_sdk/velocity",10, &velocity_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  ros::Subscriber Path_planned = nh.subscribe("path_defined", 10, &pathCallback);
 

  // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);

  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  PicturePub = nh.advertise<dji_sdk::picture>("picture_taken", 10);

  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

	  ros::spin();

  return 0;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/
void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}
// On progress If there is a problem check variables of the operations
void
lineOffset(bool passed,
                         sensor_msgs::NavSatFix& current,
                         sensor_msgs::NavSatFix& origin, float target_offset_x, float target_offset_y)
{
    geometry_msgs::Vector3  deltaNed;
    double a, k, current_point, initial, tempo, x0, y0, diff;
    double deltaLon = current.longitude - origin.longitude;
    double deltaLat = current.latitude - origin.latitude;
    deltaNed.y = deltaLat * deg2rad * C_EARTH; //We can consider this the current position
    deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*current.latitude);
    //We already have the target position
    
    //We have to calculate it different depending on the way we go
    if (std::abs(deltaNed.y)>std::abs(deltaNed.x)){//Y is bigger so change of coordinates
        tempo=deltaNed.y;//We change the difference
        deltaNed.y=deltaNed.x;
        deltaNed.x=tempo;
        y0=  origin.longitude * deg2rad * C_EARTH * cos(deg2rad*origin.latitude);//Calculated origin
        x0= origin.latitude * deg2rad * C_EARTH;
        
        a=(target_offset_x-x0)/(target_offset_y-y0);//We calculate the slope
        k= a*target_offset_y+target_offset_x;//That depends on which is bigger x or y
       // double current_point= current_gps.longitude * deg2rad * C_EARTH; We have already calculated the current point
        current_point= a*deltaNed.y+x0;
        initial= a*y0+x0;
    }
    else{// Normal case
        x0=  origin.longitude * deg2rad * C_EARTH * cos(deg2rad*origin.latitude);
        y0=  origin.latitude * deg2rad * C_EARTH;
        a=(target_offset_y-y0)/(target_offset_x-x0);//We calculate the slope
        k= a*target_offset_x+target_offset_y;//That depends on which is bigger x or y
        current_point= a*deltaNed.x+y0;
        initial= a*x0+y0;
    }
    //Depending on the values of i0 and target? It will decrease always We can check this here or later
    if(k>initial){
    diff=k-current_point;
    }else{
    diff=current_point-k;
    }
    if(diff<-0.01){
        passed=true;
        ROS_INFO("We passed a picture %d ",picture_counter );
        
    }else{
        passed=false;
    }
}
////

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}


//Step mission
void Mission::step()
{
  static int info_counter = 0;
  geometry_msgs::Vector3     localOffset;

  float speedFactor         = 4;
  float yawThresholdInDeg   = 2;

  float xCmd, yCmd, zCmd;

  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

  double xOffsetRemaining = target_offset_x - localOffset.x;
  double yOffsetRemaining = target_offset_y - localOffset.y;
  double zOffsetRemaining = target_offset_z - localOffset.z;

  double yawDesiredRad     = deg2rad * target_yaw;
  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  double yawInRad          = toEulerAngle(current_atti).z;

  info_counter++;
  if(info_counter > 25)
  {
    info_counter = 0;
    ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
    ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);
  }
  if (abs(xOffsetRemaining) >= speedFactor)
    xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    xCmd = xOffsetRemaining;

  if (abs(yOffsetRemaining) >= speedFactor)
    yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    yCmd = yOffsetRemaining;

  zCmd = start_local_position.z + target_offset_z;


  /*!
   * @brief: if we already started breaking, keep break for 50 sample (1sec)
   *         and call it done, else we send normal command
   */

  if (break_counter > 50)//We could add a picture here
  {
  
    ROS_INFO("##### Route %d finished....", state);
      pic.gps_position = current_gps;
      pic.Attitude = current_atti;
      pic.velocity= current_velocity;
      pic.imu= current_imu;
      PicturePub.publish(pic);
	picture_mission.finished=1;
      ROS_INFO("##### Picture %d taken...", picture_counter);
    finished = true;
    return;
  }
  else if(break_counter > 0)
  {
    sensor_msgs::Joy controlVelYawRate;
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_GROUND   |
                DJISDK::STABLE_ENABLE);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(flag);

    ctrlBrakePub.publish(controlVelYawRate);
    break_counter++;
    return;
  }
  else //break_counter = 0, not in break stage
  {
    sensor_msgs::Joy controlPosYaw;


    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(yawDesiredRad);
    ctrlPosYawPub.publish(controlPosYaw);
  }

    //Here is where we start to counter for the break
  if (std::abs(xOffsetRemaining) < 0.1 &&
      std::abs(yOffsetRemaining) < 0.1 &&
      std::abs(zOffsetRemaining) < 0.1 &&
      std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
  {
    //! 1. We are within bounds; start incrementing our in-bound counter
    inbound_counter ++;
  }
  else
  {
    if (inbound_counter != 0)
    {
      //! 2. Start incrementing an out-of-bounds counter
      outbound_counter ++;
    }
  }

  //! 3. Reset withinBoundsCounter if necessary
  if (outbound_counter > 10)
  {
    ROS_INFO("##### Route %d: out of bounds, reset....", state);
    inbound_counter  = 0;
    outbound_counter = 0;
  }

  if (inbound_counter > 50)
  {
    ROS_INFO("##### Route %d start break....", state);
    break_counter = 1;
  }

}


//Picture mission
void Mission::sensing()
{
    geometry_msgs::Vector3     localOffset;
    float yawThresholdInDeg   = 2;
   // bool passed;
    float xCmd, yCmd, zCmd;
    
    localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);//MAybe we can use only one function for both
    lineOffset(passed,current_gps,start_gps_location,target_offset_x,target_offset_y);
    
    double xOffsetRemaining = target_offset_x - localOffset.x;
    double yOffsetRemaining = target_offset_y - localOffset.y;
    double zOffsetRemaining = target_offset_z - localOffset.z;

    double yawDesiredRad     = deg2rad * target_yaw;
    double yawThresholdInRad = deg2rad * yawThresholdInDeg;
    double yawInRad          = toEulerAngle(current_atti).z;
    
    if (break_counter == 1)
    {
        //Here we should send a rostopic
        //First we check only with the values that we know that we are getting

        pic.gps_position = current_gps;
        pic.Attitude = current_atti;
        pic.velocity= current_velocity;
        pic.imu= current_imu;
        PicturePub.publish(pic);
        
        ROS_INFO("##### Picture %d taken...", picture_counter);

        finished = true;
        return;
    }
    else if(break_counter > 0)
    {
        break_counter++;
        return;
    }

    //Here is where we start to counter for the break
    if (std::abs(xOffsetRemaining) < 0.1 &&
        std::abs(yOffsetRemaining) < 0.1 &&
        std::abs(zOffsetRemaining) < 0.1 &&
        std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)//SHOuld we consider this?
    {
        //! 1. We are within bounds; start incrementing our in-bound counter
        inbound_counter ++;
    }
    else
    {
        if (inbound_counter != 0)
        {
            //! 2. Start incrementing an out-of-bounds counter
            outbound_counter ++;
        }
    }
    
    //! 3. Reset withinBoundsCounter if necessary
    if (outbound_counter > 10)
    {
        ROS_INFO("##### Route %d: out of bounds, reset....", state);
        inbound_counter  = 0;
        outbound_counter = 0;
    }
    
    if (inbound_counter == 1)
    {
        //ROS_INFO("##### Route %d start break....", state);
        break_counter = 1;
    }
	
   if(picture_counter > 1){
	   if(passed==true){

	
			pic.gps_position = current_gps;
			pic.Attitude = current_atti;
			pic.velocity=current_velocity;
			pic.imu=current_imu;
			PicturePub.publish(pic);
			ROS_INFO("##### Picture %d taken FAIIIIIL...", picture_counter);
			finished = true;
			return;
		     }
	        
	  }
    
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

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    current_imu = *msg;// Because we want all the message
}

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    current_velocity = *msg;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;
if(check==true){	//We start only after reading the path message
  // Down sampled to 50Hz loop
  if(elapsed_time > ros::Duration(0.02))
  {
     //Update here the state of the i and j? Only i?
              if(square_mission.finished)
              {
                  i=i+2;
              }
      
      
              if(!picture_mission.finished)
              {
                  picture_mission.sensing();
              }
              else
              {
                  
                  if(square_mission.finished){
                      j=j+2;
                      picture_counter=picture_counter+1;
                  }
                  if(pos[j+2]==line[i] && pos[j+3]==line[i+1]){//Nos podemos meter en un lio aquí
                  //Esto se queda aquí hasta que acabamos la línea del mission sensing
                  
                  }else{
                   
                     // if(!square_mission.finished){
                          j=j+2;
                          picture_counter=picture_counter+1;
                     // }
                  picture_mission.reset();
                  picture_mission.start_gps_location = current_gps;
                  picture_mission.start_local_position = current_local_pos;
                  //We have to move the global coordinates into local ones
                  //Do we use the function to calculate that?
                  loc_pic[0]=pos[j]-pos[j-2];
                  loc_pic[1]=pos[j+1]-pos[j-1];
                  picture_mission.setTarget(loc_pic[0], loc_pic[1], 0, 60);//Here we have to add every point
                  picture_mission.state = picture_counter;
                  ROS_INFO("##### Start picture route %d ....", picture_mission.state);
                  
                  //As it is relative altitude we donn't have to change it in between points
                  }
              }
      
    	 switch(state)
    {
      case 0:
	if(!square_mission.finished)
			{
			  square_mission.step();
			}
	else
	{
	 if(counter==lines-1 or counter==lines){ //We have to improve that with something better
	  state=1;
	   break;
           }
		//i=i+2;
        counter=counter+1;
	
        //Do we add something here about the picture?
        
			  square_mission.reset();
			  square_mission.start_gps_location = current_gps;
			  square_mission.start_local_position = current_local_pos;
			//We have to move the global coordinates into local ones
                loc[0]=line[i]-line[i-2];
                loc[1]=line[i+1]-line[i-1];
			  square_mission.setTarget(loc[0], loc[1], 0, 60);//Here we have to add every point
              square_mission.state = counter;
			  ROS_INFO("##### Start route %d ....", square_mission.state);
			ROS_INFO("The picture state mission is %d ....", picture_mission.finished);

//As it is relative altitude we donn't have to change it in between points
			  
			
			}
	break;	
	//In case that we reach the last point we have to start the landing
	case 1:
		
			if(!square_mission.Landing_result)
			{
				square_mission.Landing_result=square_mission.monitoredLanding();
			}
			else
			{
			    ROS_INFO("Mission %d ##### Landed ....", square_mission.state);
			    square_mission.state = 0;
			    ros::shutdown();//Sure?

			}
			
			
	break;
		
	}
   } 
  }
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}



/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_AUTO_LANDING) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}

bool Mission::monitoredLanding()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();


  return true;
    }



bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}


//Callback for the path. It cannot be void it has to give back the values
void pathCallback(const dji_sdk::path::ConstPtr& msg)
{
    //Here we read and make the conversion
    
    //Here we get some characteristics from the message sent
    float dstride0 = msg->Pictures.layout.dim[0].stride;
    float dstride1 = msg->Pictures.layout.dim[1].stride;
    points = msg->numPoints;
    lines = msg->numLines;
	altitude=msg->altitude;
    int h = msg->Pictures.layout.dim[0].size;
    int w = msg->Pictures.layout.dim[1].size;
    float Arr_1[w][h];//This one needs to be declared outside to be global and to be able to use it

float dstride3 = msg->Lines.layout.dim[0].stride;
    float dstride4 = msg->Lines.layout.dim[1].stride;
    int f = msg->Lines.layout.dim[0].size;
    int g = msg->Lines.layout.dim[1].size;
    float Arr_2[f][g];//This one needs to be declared outside to be global and to be able to use it
    int i=0, j=0;
    int counter_1=0, counter_2=0;
    
    
    for(std::vector<float>::const_iterator it = msg->Pictures.data.begin(); it != msg->Pictures.data.end(); ++it)
    {
        if(counter_1%2==0){
            Arr_1[i][0] = *it;
            //We only increase i after the second one
            counter_1++;
	    pos.push_back(Arr_1[i][0]);	//We add the position to the global variable
        }else{
            Arr_1[i][1] = *it;
	    pos.push_back(Arr_1[i][1]);
            i++;
            counter_1++;
            
        }
    }

for(std::vector<float>::const_iterator it = msg->Lines.data.begin(); it != msg->Lines.data.end(); ++it)
    {
        if(counter_2%2==0){
            Arr_2[j][0] = *it;
            //We only increase j after the second one
            counter_2++;
	    line.push_back(Arr_2[j][0]);
        }else{
            Arr_2[j][1] = *it;
	    line.push_back(Arr_2[j][1]);
            j++;
            counter_2++;
            
        }
}
	
        //If something fails check without the rosspin

 bool obtain_control_result = obtain_control();
	  bool takeoff_result;
		float value=0;
	  
	    

	  if (!set_local_position()) // We need this for height
	  {
	    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
	   // return 1; //DOn't forget this!
	  }

	  if(is_M100())
	  {
	    ROS_INFO("M100 taking off!");

	  }
	  else
	  {
	    ROS_INFO("A3/N3 taking off!");
	    takeoff_result = monitoredTakeoff();
	  }

	  if(takeoff_result)//We send the first point to the mission controller                   
	  {
	    square_mission.reset();
	    square_mission.start_gps_location = current_gps;
	    square_mission.start_local_position = current_local_pos;
	    square_mission.setTarget(line[0], line[1], altitude, 60); //The first coordinate can be given like that
	      //The first coordinate can be given in global coordinates
	    square_mission.state = 1;
	    ROS_INFO("##### Start route %d ....", square_mission.state);
      /*   
        picture_mission.reset();
        picture_mission.start_gps_location = current_gps;
        picture_mission.start_local_position = current_local_pos;
        picture_mission.setTarget(pos[0], pos[1], altitude, 60); //The first coordinate can be given like that
          //The first coordinate can be given in global coordinates
        picture_mission.state = 1;
          ROS_INFO("##### Picture mission started %d ....", picture_mission.state);*/
	  }
	check=true;
        ros::spin();
    



}

