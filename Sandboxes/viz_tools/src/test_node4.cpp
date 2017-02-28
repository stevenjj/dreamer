#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
// #include <geometry_msgs/Twist.h>
// #include <turtlesim/Spawn.h>

#include "marker_pub/ros_pub.h"


ros_pub* rp_;
sejong::MarkerChannel mc_eye_left;
sejong::MarkerChannel mc_eye_right;
sejong::MarkerChannel mc_head;



sejong::Vect3      pos_eye_right;
sejong::Quaternion quat_eye_right;

sejong::Vect3      pos_eye_left;
sejong::Quaternion quat_eye_left;

sejong::Vect3      pos_head;
sejong::Quaternion quat_head;

int count = 0;



int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;
  rp_ = ros_pub::get_ros_pub();

  // ros::service::waitForService("spawn");
  // ros::ServiceClient spawner =
  //   node.serviceClient<turtlesim::Spawn>("spawn");
  // turtlesim::Spawn turtle;
  // turtle.request.x = 4;
  // turtle.request.y = 2;
  // turtle.request.theta = 0;
  // turtle.request.name = "turtle2";
  // spawner.call(turtle);

  // ros::Publisher turtle_vel =
  //   node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  rp_->new_arr_channel(mc_eye_left,  "left_eye_ori",  sejong::BLUE, 50, 1., 0.5, 4);  // 50 * 4 = 200 => 2 second tail @ 100 Hz
  rp_->new_arr_channel(mc_eye_right, "right_eye_ori", sejong::LIME, 8, 1., 0.5, 25);  // 8 * 25 = 200 => 2 second tail @ 100 Hz
  rp_->new_arr_channel(mc_head,      "head_ori",      sejong::RED,  10, 1., 0.25, 10);// 10* 10 = 100 => 1 second tail @ 100 Hz


  ros::Rate rate(100.0);
  while (node.ok()){
    geometry_msgs::TransformStamped tf_left_eye, tf_right_eye, tf_head;

    try{
      tf_left_eye =  tfBuffer.lookupTransform("world", "left_eye_yaw_link",    ros::Time(0));
      tf_right_eye = tfBuffer.lookupTransform("world", "right_eye_yaw_link",   ros::Time(0));
      tf_head =      tfBuffer.lookupTransform("world", "upper_neck_roll_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    pos_eye_left = sejong::Vect3(tf_left_eye.transform.translation.x,
                                 tf_left_eye.transform.translation.y,
                                 tf_left_eye.transform.translation.z);

    quat_eye_left = sejong::Quaternion(tf_left_eye.transform.rotation.w,
                                       tf_left_eye.transform.rotation.x,
                                       tf_left_eye.transform.rotation.y,
                                       tf_left_eye.transform.rotation.z);


    pos_eye_right = sejong::Vect3(tf_right_eye.transform.translation.x,
                                  tf_right_eye.transform.translation.y,
                                  tf_right_eye.transform.translation.z);

    quat_eye_right = sejong::Quaternion(tf_right_eye.transform.rotation.w,
                                        tf_right_eye.transform.rotation.x,
                                        tf_right_eye.transform.rotation.y,
                                        tf_right_eye.transform.rotation.z);

    pos_head = sejong::Vect3(tf_head.transform.translation.x,
                             tf_head.transform.translation.y,
                             tf_head.transform.translation.z);

    quat_head = sejong::Quaternion(tf_head.transform.rotation.w,
                                   tf_head.transform.rotation.x,
                                   tf_head.transform.rotation.y,
                                   tf_head.transform.rotation.z);


    rp_->pub_arr_marker(mc_eye_left,  pos_eye_left,  quat_eye_left);
    rp_->pub_arr_marker(mc_eye_right, pos_eye_right, quat_eye_right);
    rp_->pub_arr_marker(mc_head,      pos_head,      quat_head);



    // if( count == 1000 )
    // {
    //   rp_->set_maxNumSpheres(mc_eye_left, 100);
    //   count = 0;
    // }
    // count++;

    // geometry_msgs::Twist vel_msg;

    // vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
    //                                 transformStamped.transform.translation.x);
    // vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) +
    //                               pow(transformStamped.transform.translation.y, 2));
    // turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};