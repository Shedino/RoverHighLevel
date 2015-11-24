#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    tf::Quaternion fix_quart;
    tf::Quaternion fix_quart2;
    fix_quart.setRPY(M_PI ,0.0 , 0.0);
    fix_quart2.setRPY(M_PI,0.54 , 0.0);
    broadcaster.sendTransform(
      tf::StampedTransform(
       tf::Transform(fix_quart, tf::Vector3(0.493, 0.0, 0.0)), ros::Time::now(),"base_link","base_laser")
    );
    broadcaster.sendTransform(
      tf::StampedTransform(
       tf::Transform( fix_quart2,tf::Vector3(0.0 , -0.26 , 0.0)), ros::Time::now(),"base_laser","openni_depth_frame")
    );
    /*
    broadcaster.sendTransform(
      tf::StampedTransform(
        //tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.493, 0.0, 0.16)), ros::Time::now(),"base_link","base_laser")
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.164)), ros::Time::now(),"base_footprint","base_link")
    );
    */
    r.sleep();
  }
}

//        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.164)),
//       ros::Time::now(),"base_footprint", "base_link")
