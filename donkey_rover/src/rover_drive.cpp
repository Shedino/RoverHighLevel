#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include "std_msgs/Int32.h"
#include <math.h>
#include "libRover.h"
Rover rover(false);
EScannerState state;

void correctfactor(const geometry_msgs::Vector3::ConstPtr& vector)
{
  geometry_msgs::Vector3 new_correction = *vector;
  float msg = new_correction.z;
  int retCode;


  /*if( angle != dummyangle ){

      retCode = rover.setScannerAdjustment(angle);

      if (retCode != 0)
      {
      printError("Failed to set new angle", retCode);

      }
      printf("Angle is set to :  %f", angle);
      while (state != ESSIdle)
      {
          cout << '.' << flush;
          Time::sleep(1, 0);
          retCode = rover.getScannerState(state);
      }
      angle = dummyangle;
  }*/

  if (msg ==1 ){
      while (state != ESSIdle)
      {
       cout << '.' << flush;
       Time::sleep(1, 0);
       retCode = rover.getScannerState(state);
      }
      retCode = rover.sendScannerCommand(ESCDoHoming);
      if (retCode != 0)
      {
       printError("GoHome failed", retCode);

      }
  } else if(msg == 2)
  {
      while (state != ESSIdle)
      {
       cout << '.' << flush;
       Time::sleep(1, 0);
       retCode = rover.getScannerState(state);
      }
      retCode = rover.sendScannerCommand(ESCStart);
      if (retCode != 0)
      {
      printError("Starting failed", retCode);

      }
  } else if(msg == 3)
  {
      retCode = rover.sendScannerCommand(ESCStop);
      if (retCode != 0)
      {
       printError("Stopping failed", retCode);

      }
  } else if (msg == 4)
  {
      retCode = rover.setScannerAdjustment(new_correction.x);
      if (retCode != 0)
      {
          printError("setScannerAdjustment failed", retCode);

      }

      ROS_INFO("I heard");
      printf("Angle is set to :  %f", new_correction.x);
      while (state != ESSIdle)
      {
          cout << '.' << flush;
          Time::sleep(1, 0);
          retCode = rover.getScannerState(state);
      }

  }else
  {
      //printf(" \n Command is not recognized, recognized commands are GoHome, Start and Stop : \n");

  }

}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "rover_drive");

 
  ros::NodeHandle n;


  ros::Subscriber sub3 = n.subscribe("c1", 10, correctfactor);

  int retCode = rover.init();
  if (retCode != 0)
        {
        // Failed to init the rover
        return -1;
  }

  while (state != ESSIdle)
  {
      cout << '.' << flush;
      Time::sleep(1, 0);
      retCode = rover.getScannerState(state);
  }

  retCode = rover.setScannerPeriod(30);
  if (retCode != 0)
  {
      printError("setScannerPeriod failed", retCode);
      return -1;
  }
  while (state != ESSIdle)
  {
      cout << '.' << flush;
      Time::sleep(1, 0);
      retCode = rover.getScannerState(state);
  }

  /*retCode = rover.setScannerAdjustment(1.57);
  while (state != ESSIdle)
  {
      cout << '.' << flush;
      Time::sleep(1, 0);
      retCode = rover.getScannerState(state);
  }*/
  ros::spin();

  return 0;
}
