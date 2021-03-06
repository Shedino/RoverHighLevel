#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
float roll_inc;
float sgn = 0.01;
float sgn_last = 0.01;
float last = 0;
float last_time = 0;
bool flaghorizon = true;

void correctfactor(const geometry_msgs::Vector3::ConstPtr& vector)
{
  geometry_msgs::Vector3 new_correction = *vector;
  if (new_correction.z==3) flaghorizon = false;
  else flaghorizon = true;
}
void scanfunc(const geometry_msgs::Vector3::ConstPtr& vec){
    geometry_msgs::Vector3 new_data = *vec;
    float dx = new_data.x - last;
    last = new_data.x;
    if (dx > 0.0 && dx < 5000){
        sgn = 0.01;
    } else if (dx < 0.0 && dx > -5000){
        sgn = -0.01;
    } else {
        sgn = 0.0;
    }
    /*if ((sgn + sgn_last) == 0){

        //ROS_INFO("Check Point");
        if(roll > M_PI/2){
            ROS_INFO("Check Point 180");
            //roll = M_PI;
        } else {
            ROS_INFO("Check Point 0 ");
            //roll = 0.0;
        }

    }
    sgn_last = sgn;
    if ((new_data.z - last_time) > 0.01){ 

        roll = sgn*new_data.y*M_PI/24178.12 + roll;
        last_time = new_data.z;
    }*/
    roll_inc = sgn*new_data.y*M_PI/24178.12;


}


int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);
  ros::Subscriber sub = n.subscribe("scanner_data", 1, scanfunc);
  tf::TransformBroadcaster broadcaster;
  int count = 0;
  float roll_min = 0.0;
  float roll = M_PI/2;
  float roll_last = roll;
  bool rollflag = true; // Flag of roll_min, works only once in the loop
  bool swipeflag = true; //Flag of Swipe, works one a swipe
  ros::Subscriber sub2 = n.subscribe("c1", 10, correctfactor);
  tf::Quaternion fix_quart2;  
  fix_quart2.setRPY(M_PI,0.54 , 0.0);
  while(n.ok()){

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.493, 0.0, 0.0)), ros::Time::now(),"base_link","base_laser")
    );


    
    // Calculating Roll angle


    if (count == 50){
        count =0;
        //ROS_INFO("The roll angle is %f  \n", roll);
        ROS_INFO("  ******     Roll min is    %f   ****** ", roll_min);
    }
    roll_last = roll;
    roll = roll + roll_inc;
    if (roll < 0.6){  // Not tested
               if((roll > roll_last) && swipeflag ){
                   if (rollflag) { // TO be executed once
                       roll_min = roll_last;
                       rollflag = false;
                   }
                   roll = roll_min + roll_inc;
                   swipeflag = false;
               }
    }
    if (roll > 2.7){
        swipeflag = true;
    }
    if (flaghorizon) roll = M_PI/2;
    ROS_INFO("The roll angle is %f  \n", roll);
    // Broadcasting base_scanner - base_laser
    tf::Quaternion scan_quart;
    scan_quart.setRPY(roll + M_PI/2 ,0.0 , 0.0);
    broadcaster.sendTransform(
      tf::StampedTransform(

        tf::Transform(scan_quart, tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(),"base_laser","base_scanner")
    );
    broadcaster.sendTransform(
      tf::StampedTransform(
       tf::Transform( fix_quart2,tf::Vector3(0.0 , -0.26 , 0.0)), ros::Time::now(),"base_scanner","openni_depth_frame")
    );
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(),"openni_depth_frame","camera_link")
    );

    ros::Duration(0.02).sleep();
    ros::spinOnce();
    count ++;
    r.sleep();
  }
}

//        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.164)),
//       ros::Time::now(),"base_footprint", "base_link")
