#include "ros/ros.h"
#include "libRover.h"
#include <sensor_msgs/Joy.h>

Rover rover(false);

void joyCallback(const sensor_msgs::JoyConstPtr& joy)
{
  

  float v=joy->axes[1];
  float vth=joy->axes[2];
  float a1=joy->buttons[4];
  v= v*(a1+1)/2;
   
  int retCode = rover.setSpeedVO (v, vth);
  //cout << "RETCODE" << retCode << endl << endl;
    if (retCode != 0)
		{
     	// Failed to set the speed
		ROS_INFO("Failed to set speed of the rover");
  }

  
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "rover_joy");

  ros::NodeHandle n;

  int retCode = rover.init();
  if (retCode != 0)
		{
     	// Failed to init the rover
        ROS_INFO("Initialization Failed");
		return -1;
  }
  ros::Rate loop_rate(100);
  EScannerState state;
  retCode = rover.getScannerState(state);
  while (state != ESSIdle)
    {
        cout << '.' << flush;
        Time::sleep(1, 0);
        retCode = rover.getScannerState(state);
    }
  /*
  retCode = rover.setScannerAngle(M_PI/4.0);
  if (retCode != 0)
    {
        printError("setScannerAngle failed", retCode);
             
    }
  retCode = rover.sendScannerCommand(ESCStart);
  if (retCode != 0)
    {
        printError("Starting Scanner failed", retCode);
             
    }*/
  ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("joy", 1, joyCallback);
  while (ros::ok())
  {

     //ROS_INFO(sub);

       //retCode = rover.setScannerAngle(M_PI/3);
       if (retCode != 0)
       {
             printError("setScannerAngle failed", retCode);
             
       }
     ros::spinOnce();
     loop_rate.sleep();
 }
 return 0;
}

