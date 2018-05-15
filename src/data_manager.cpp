#include "dji_sdk/picture.h"  //To include the msg created
#include <std_msgs/Float32MultiArray.h> //Just in case we also add the multiarray part
#include <std_msgs/MultiArrayDimension.h>
#include "ros/ros.h"
#include <time.h>
#include "std_msgs/String.h"
#include <stdlib.h>
#include <sstream>
#include <tinyxml.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

 int i = 0;

void pictureCallback(const dji_sdk::picture::ConstPtr& msg)
{

//We just enter when we get a message
    sensor_msgs::NavSatFix current_gps = msg->gps_position;
    sensor_msgs::Imu current_imu = msg->imu;
    geometry_msgs::Quaternion current_atti = msg->Attitude;
    geometry_msgs::Vector3 current_velocity = msg->velocity.vector
;
    ros::Time Time_stamp = msg->gps_position.header.stamp;
    
    //That is one way to do it
    std::string gps_longitude = boost::lexical_cast<std::string>(current_gps.longitude);
    std::string gps_latitude = boost::lexical_cast<std::string>(current_gps.latitude);
    std::string gps_altitude = boost::lexical_cast<std::string>(current_gps.altitude);
    
    std::string attitude_x = boost::lexical_cast<std::string>(current_atti.x);
    std::string attitude_y = boost::lexical_cast<std::string>(current_atti.y);
    std::string attitude_z = boost::lexical_cast<std::string>(current_atti.z);
    std::string attitude_w = boost::lexical_cast<std::string>(current_atti.w);
    
    std::string velocity_x = boost::lexical_cast<std::string>(current_velocity.x);
    std::string velocity_y = boost::lexical_cast<std::string>(current_velocity.y);
    std::string velocity_z = boost::lexical_cast<std::string>(current_velocity.z);
    
    std::string Time = boost::lexical_cast<std::string>(Time_stamp);
    
    //We try to use it like that. If it works we try later to use the messages directly

    
    std::string xml="/home/nico/Desktop/test.xml";
    TiXmlDocument doc(xml.c_str()); //We open an existing document
    if (doc.LoadFile()) {
        TiXmlHandle docHandle(&doc);
        TiXmlElement* fileLog = docHandle.FirstChild("FileLog").ToElement();
        
        if (fileLog) { //If the element filelog exists, we add a new one
            TiXmlElement recording("Picture");
            
            recording.SetAttribute("ID", i);
            recording.SetAttribute("gps_position_x", gps_longitude); //Check if there is a problem and we have to do it with other "instructions"
            recording.SetAttribute("gps_position_y", gps_latitude);
            recording.SetAttribute("gps_position_z", gps_altitude);
            
            recording.SetAttribute("attitude_x", attitude_x);
            recording.SetAttribute("attitude_y", attitude_y);
            recording.SetAttribute("attitude_z", attitude_z);
            recording.SetAttribute("attitude_w", attitude_w);
            
            recording.SetAttribute("Time", Time);
            fileLog->InsertEndChild(recording);
        }
    }
    
    else std::cout << "error loading file" << std::endl;
    
    if(doc.SaveFile(xml.c_str())){
        std::cout << "file saved succesfully.\n";
        
    }
    else std::cout << "error saving file" << std::endl;
    
    

    i++;
   
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listen");

  ros::NodeHandle n;

  
  ros::Subscriber sub = n.subscribe("picture_taken", 1000, pictureCallback);
  ros::spin();

  return 0;
}


