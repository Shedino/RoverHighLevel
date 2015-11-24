#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "libRover.h"
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <custom_msgs/velocityEstimate.h>

Rover rover(false);
float C1 = 1;
float C2 = 1;
float VX = 0.0;
float VY = 0.0;
void imuspeed(const custom_msgs::velocityEstimate::ConstPtr& speed){
    custom_msgs::velocityEstimate s = *speed;
    VX = s.velE;
    VY = s.velN;
}

void correctfactor(const geometry_msgs::Vector3::ConstPtr& vector)
{
  geometry_msgs::Vector3 new_correction = *vector;
  C1 = new_correction.x;
  C2 = new_correction.y;
}
void donkeyMover(const geometry_msgs::Twist::ConstPtr& vel)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  geometry_msgs::Twist new_vel = *vel;
  float v = sqrt (new_vel.linear.x * new_vel.linear.x + new_vel.linear.y * new_vel.linear.y);
  float vth = new_vel.angular.z;
  //printf("Sending linear velocity of %f and angular velocity of %f \n",v,vth);
  int retCode = rover.setSpeedVO (v, vth);
    if (retCode != 0)
        {
        // Failed to set the speed
        ROS_INFO("Failed to set speed of the rover");
  }

}
void joyCallback(const sensor_msgs::JoyConstPtr& joy)
{
  

  float v=joy->axes[1];
  float vth=joy->axes[2];
  float a1=joy->buttons[4];
  //int a2=joy->buttons[5];
  v= v*(a1+1)/2;

  int retCode = rover.setSpeedVO (v, vth);
  //cout << "RETCODE" << retCode << endl << endl;
    if (retCode != 0)
		{
     	// Failed to set the speed
		ROS_INFO("Failed to set speed of the rover");
  }
  /*while(a2==1)
      {
       rover.disableTracks();
  }
  rover.enableTracks();
*/
  
}
int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("twist", 100);
  tf::TransformBroadcaster odom_broadcaster;
  

  float x = 0.0;
  float y = 0.0;
  float th = 0.0;
  float v;
  float syncf = 100.0;


  /* ROVER INITIALIZATION */
  
  
  
  int retCode = rover.init();
  if (retCode != 0)
		{
     	// Failed to init the rover
		return -1;
  }
    EScannerState state;
    retCode = rover.getScannerState(state);
    while (state != ESSIdle)
    {
        cout << '.' << flush;
        Time::sleep(1, 0);
        retCode = rover.getScannerState(state);
    }

  retCode = rover.setSyncFreq(syncf);
  if (retCode != 0)
        {
        // Failed to set sync frequency
        printf("Failed to set sync frequency to %f",syncf);
        return -1;
  }
  float vx = 0.1;
  float vy = -0.1;
  float vth;
  float v_left;
  float v_left2;
  float v_right;
  float v_right2;
  int count = 0;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
//printf(" speed is %f ",vx);
  ros::Rate r(100.0);
  ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("joy", 1, joyCallback);
  ros::Subscriber sub2 = n.subscribe("cmd_vel", 1, donkeyMover);
  ros::Subscriber sub3 = n.subscribe("c1", 10, correctfactor);  
  ros::Subscriber sub4 = n.subscribe("mti/filter/velocity", 10, imuspeed);
  while(n.ok()){
    
    Time timestamp;
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    // Reading speed of the rover
    retCode = rover.getSpeedInMPerS(timestamp, v_left, v_right, v_left2, v_right2);

    if (retCode != 0)
        {
        // Failed to read the rover speed
        ROS_INFO("Failed to read the Righ_left speed");
        return -1;
    }

    retCode = rover.getSpeedVO(timestamp, v, vth);
    //printf(" retCode  is %d, speed is  %f \n ",retCode,v);
    if (retCode != 0)
		{
     	// Failed to read the rover speed
		ROS_INFO("Failed to read the rover speed");
		return -1;
    }
    /*v2 = (v_left + v_right)/2;
    vth2 = (v_left - v_right)/0.793;*/
    if ((count>50) && ((v > 0.01) || (vth > 0.01) || (vth < -0.01)))
        {
        printf("Front left speed is   %f   while rear left speed is   %f   \n", v_left, v_left2);
        printf("Front right speed is   %f   while rear right speed is   %f   \n .. \n .. \n .. \n", v_right, v_right2);

    }

    //Apply correction on Odometry data
    v = v * C1;
    vth= vth * C2;
    // Correction applied
    vx = v * cos(th);    
    vy = v * sin(th);
    // IMS based Speeds
    //vx = VX;
    //vy = VY;


    //compute odometry in a typical way given the velocities of the robot
    float dt = (current_time - last_time).toSec();
    float delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    float delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    float delta_th = vth * dt;
//DEBUG
    delta_x = vx * dt;
    delta_y = vy * dt;
// DEBUG END
    x += delta_x;
    y += delta_y;
    th += delta_th;


    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    geometry_msgs::Twist tw;

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    tw.linear.x = VX;
    tw.linear.y = VY;

    //publish the message
    odom_pub.publish(odom);
    twist_pub.publish(tw);

    last_time = current_time;
    r.sleep();
    count ++;
  }
}
