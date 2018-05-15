/*
 * planner.cpp
 *
 *  Created on: Feb 6, 2018
 *      Author: nicolas
 */
/** @file planner.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "Pantheon/planner.h"
#include "dji_sdk/path.h" 
#include "dji_sdk/dji_sdk.h"
// This part is for the message of the where we send the path planning done
#define H (2) 
#define W (4) //This has to be implemented on the message from matlab or in a different txt 


// Here we define the publishers or service clients
ros::ServiceClient set_local_pos_reference;
ros::Publisher path_planning;

//Defintion of the message that we are going to send
dji_sdk::path dat;

//We declare 3 variables of the cmaera class
Camera Thermal, Multi, Rgb;

//We declare 3 variables of the cmaera class
Coordinates points, Dif;
Dual pictures;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planner");
  ros::NodeHandle nh;

  // Subscribe to messages
 // ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);

  // Publish message of the calculated points
  path_planning = nh.advertise<dji_sdk::path>("path_defined", 10);

  // Basic services
//  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");

   //We have to check this to obtain the parameters
 
    //Main part
    //Camera characteristics should be written in a parameter service
    //Thermal camera
    nh.param("/Camera/Thermal/FOV/x", Thermal.FOV_x, 32); //It should be 32, 5000 is for the test
    nh.param("/Camera/Thermal/FOV/y", Thermal.FOV_y, 26);
    nh.param("/Camera/Thermal/RES/x", Thermal.RES_x, 640);
    nh.param("/Camera/Thermal/RES/y", Thermal.RES_y, 512);

    //ROS_INFO("The parameter used for the thermal camera is %d", Thermal.FOV_x);
   
    //Try also if it does work this way
    
    //Another way would be: nh.param("Thermal/FOV/y", Thermal.FOV_y, 26); camera also?
    
    //Multispectral camera
    nh.param("/Camera/Multispectral/FOV/x", Multi.FOV_x, 35);//CHange
    nh.param("/Camera/Multispectral/FOV/y", Multi.FOV_y, 24);
    nh.param("/Camera/Multispectral/RES/x", Multi.RES_x, 6000);
    nh.param("/Camera/Multispectral/RES/y", Multi.RES_y, 4000);
    
    //RGB camera
    nh.param("/Camera/Rgb/FOV/x", Rgb.FOV_x, 38);
    nh.param("/Camera/Rgb/FOV/y", Rgb.FOV_y, 30);
    nh.param("/Camera/Rgb/RES/x", Rgb.RES_x, 1280);
    nh.param("/Camera/Rgb/RES/y", Rgb.RES_y, 1024);
    
    
    //Inputs: Resolution (which it gives the altitude or we could have two functions and the user decides. But first only one way); Sector size
    
    // These values we have to see where to define them
    float Overlap=0.75;
    float Sec_margin=0.25;
    int Vel_free=5; //In meters per second
    float Vel_picture=2.5;
    int Margin=1; //Margin in meters for covering well the whole area
    
    //Here we try to get the resolution and sector values by the keyboard
    std::cout << "Please enter the desired resolution: ";
    std::cin >> resolution;
    std::cout << "Please enter the x length to cover: ";
    std::cin >> sector_x;
    std::cout << "Please enter the y length to cover: ";
    std::cin >> sector_y;
    



    //We calculate the actual field of view for that resolution and its altitude
    X_picture=(resolution*Thermal.RES_x)/100;
    Y_picture=(resolution*Thermal.RES_y)/100;
    //X_picture=(resolution*Thermal.RES.x)/100;
    //Y_picture=(resolution*Thermal.RES.y)/100;
   
    
    //Size of the pixel at the altitude given
    altitude=(X_picture/2)/(tan((Thermal.FOV_x*C_PI)/180));
    //altitude=(X_picture/2)/(tan((Thermal.FOV.x*C_PI)/180));
    
    // Now we have to see if x or y is bigger
    if (sector_x>sector_y){
        change=sector_x;
        sector_x=sector_y;
        sector_y=change;
    }
    
    Dist_picture_x= X_picture*(1-Overlap); //Distance in between pictures
    x=sector_x-X_picture/2;
    Lines=1+round(0.5 + x/Dist_picture_x); //Round up the number of lines
    Dist_picture_y=Y_picture*(1-Overlap);
    y=sector_y-Y_picture/2;

    //We calculate the points needed in x and y
    pictures.y=1+round(0.5+((y)/Dist_picture_y));
    pictures.x=Lines;
    //We define the initial points As here we jut want the points, no
    //represenation we have to add the part of the field
    
    points.y=Y_picture/4;
    points.x=X_picture/4;

    //  points.y=0;
      //points.x=0;
    
    if (((pictures.y-1)*Dist_picture_y)>y) {
        Dif.y = ((pictures.y-1)*Dist_picture_y)-y;
        points.y=points.y-Dif.y/2;
    }
    if (((pictures.x-1)*Dist_picture_x)>x){
        Dif.x=((pictures.x-1)*Dist_picture_x)-x;
    points.x=points.x-Dif.x/2;
    }
    Total_pictures= (pictures.x)*(pictures.y);
    //We check the calculated values
    ROS_INFO(" Number of lines %d ", Lines);
    ROS_INFO(" Number of pictures %d ", Total_pictures);
    
    // Here we define the matrix and the messages where to store the points and the the lines?
    //Definition of the size of the matrix
    //dat.pictures?
    dat.Pictures.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.Pictures.layout.dim.push_back(std_msgs::MultiArrayDimension());//We push twice because it is a two dimensions array
    dat.Pictures.layout.dim[0].label="Longitude";
    dat.Pictures.layout.dim[1].label="Latitude";
    dat.Pictures.layout.dim[0].size=H;
    dat.Pictures.layout.dim[1].size=Total_pictures;
    dat.Pictures.layout.dim[0].stride=H*Total_pictures;
    dat.Pictures.layout.dim[1].stride=Total_pictures;
    dat.Pictures.layout.data_offset=0;
    
    dat.Lines.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.Lines.layout.dim.push_back(std_msgs::MultiArrayDimension());//We push twice because it is a two dimensions array
    dat.Lines.layout.dim[0].label="Longitude";
    dat.Lines.layout.dim[1].label="Latitude";
    dat.Lines.layout.dim[0].size=H;
    dat.Lines.layout.dim[1].size=(Lines*2);
    dat.Lines.layout.dim[0].stride=H*(Lines*2);
    dat.Lines.layout.dim[1].stride=(Lines*2);
    dat.Lines.layout.data_offset=0;
    
    //We define the 2 vectors where to store the paths
    float Path[Total_pictures][2];
    float line_path[Lines*2][2];
    std::vector<float> vec1(Total_pictures*2, 0);
    std::vector<float> vec2(Lines*4, 0);
	
    
    //If there isn't the correct amount of pictures. Check the "for" loop
    for (int h=0;h<pictures.x;h++){
        
        if(h%2==0){
            
            for (int t=0; t<pictures.y; t++){
                Path[num_pictures][0]=points.x;
                Path[num_pictures][1]=points.y;

 		if (t==0 or t==(pictures.y-1)){
                    line_path[num_lines][0]=points.x;
                    line_path[num_lines][1]=points.y;
		    num_lines=num_lines+1; 	
                }
                num_pictures= num_pictures+1;
                points.y=points.y+Dist_picture_y;
                
               
            }
        
        points.y=points.y-Dist_picture_y;
        
        }else{
                
            for(int t=0;t<=pictures.y-1; t++){
                Path[num_pictures][0]=points.x;
                Path[num_pictures][1]=points.y;

		if (t==0 or t==(pictures.y-1)){
                    line_path[num_lines][0]=points.x;
                    line_path[num_lines][1]=points.y; 
		    num_lines=num_lines+1;
               }
                num_pictures= num_pictures+1;
               points.y=points.y-Dist_picture_y;
                
                
            }
        points.y=points.y+Dist_picture_y;
        }
        
       points.x=points.x+Dist_picture_x;
    }

////////

/*
  for (int h=0;h<pictures.x;h++){
	 for (int t=0; t<pictures.y; t++){
	      Path[(h*pictures.y)+t][0]=points.x;
	      if (t==0 or t==(pictures.y-1)){
                    line_path[num_lines][0]=points.x;
		    num_lines=num_lines+1;
                }	

	}

         points.x=points.x+Dist_picture_x;
    }

num_lines=0;

 for (int h=0;h<pictures.y;h++){
	 for (int t=0; t<pictures.x; t++){
	      Path[(h*pictures.x)+t][1]=points.y;
	      if (t==0 or t==(pictures.x-1)){
                   line_path[num_lines][1]=points.y;
		    num_lines=num_lines+1;
                }	
		 points.y=points.y+Dist_picture_y;
	}

        
    }*/

    // And now we have to send all this in the message
for (int i=0; i<Total_pictures; i++){
        for (int j=0; j<2; j++){
            vec1[i*2 + j] = Path[i][j];
        }
}

for (int i=0; i<(Lines*2); i++){
        for (int j=0; j<2; j++){
            vec2[i*2 + j] = line_path[i][j];
        }
}    

dat.numPoints=Total_pictures;// Like that or push back?
dat.numLines=Lines;
dat.Pictures.data=vec1;
dat.Lines.data=vec2;
dat.altitude=altitude;
dat.speed=Vel_picture;

path_planning.publish(dat);



    //Check how to change the calculations of the points in case than more than one

  ros::spin();

  return 0;
}

// Here some examples of services or callbacks that can be useful
// Function calculating characteristics
// Function calculating time and pictures








