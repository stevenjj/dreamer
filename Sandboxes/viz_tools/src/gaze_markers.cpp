#include <ros/ros.h>


#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

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


void setArrLengthCallback (const std_msgs::Float32MultiArray::ConstPtr& array);
void setNumSpheresCallback(const std_msgs::Int32MultiArray::ConstPtr& array);
void setSphereFreqCallback(const std_msgs::Int32MultiArray::ConstPtr& array);

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;
  rp_ = ros_pub::get_ros_pub();


  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
/*  rp_->new_arr_channel(mc_eye_left,  "left_eye_ori",  sejong::BLUE, 50, 1., 0.5, 4);  // 50 * 4 = 200 => 2 second tail @ 100 Hz
  rp_->new_arr_channel(mc_eye_right, "right_eye_ori", sejong::LIME, 8, 1., 0.5, 25);  // 8 * 25 = 200 => 2 second tail @ 100 Hz
  rp_->new_arr_channel(mc_head,      "head_ori",      sejong::RED,  10, 1., 0.5, 10);// 10* 10 = 100 => 1 second tail @ 100 Hz*/


  rp_->new_arr_channel(mc_eye_left,  "left_eye_ori",  sejong::BLUE, 500, 1., 0.5, 5);  // 50 * 4 = 200 => 2 second tail @ 100 Hz
  rp_->new_arr_channel(mc_eye_right, "right_eye_ori", sejong::LIME, 500, 1., 0.5, 5);  // 8 * 25 = 200 => 2 second tail @ 100 Hz
  rp_->new_arr_channel(mc_head,      "head_ori",      sejong::RED,  500, 1., 0.5, 5);// 10* 10 = 100 => 1 second tail @ 100 Hz  


  ros::Subscriber sub1 = node.subscribe("setArrLength" , 5, setArrLengthCallback);
  ros::Subscriber sub2 = node.subscribe("setNumSpheres", 5, setNumSpheresCallback);
  ros::Subscriber sub3 = node.subscribe("setSphereFreq", 5, setSphereFreqCallback);


  ros::Rate rate(500.0);
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


    rate.sleep();
  }
  return 0;
}







void setArrLengthCallback (const std_msgs::Float32MultiArray::ConstPtr& LRH)
{
  if(LRH->data.size() == 3) {
    rp_->set_arrowLength( mc_eye_left  , LRH->data[0] );
    rp_->set_arrowLength( mc_eye_right , LRH->data[1] );
    rp_->set_arrowLength( mc_head      , LRH->data[2] );
  }
  else
    ROS_ERROR("Incorrect diminsion for setArrLength");
}
void setNumSpheresCallback(const std_msgs::Int32MultiArray::ConstPtr& LRH)
{
  if(LRH->data.size() == 3) {
    rp_->set_maxNumSpheres( mc_eye_left  , LRH->data[0] );
    rp_->set_maxNumSpheres( mc_eye_right , LRH->data[1] );
    rp_->set_maxNumSpheres( mc_head      , LRH->data[2] );
  }
  else
    ROS_ERROR("Incorrect diminsion for setArrLength");
}
void setSphereFreqCallback(const std_msgs::Int32MultiArray::ConstPtr& LRH)  // this is really the number of updates per sphere
{
  if(LRH->data.size() == 3) {
    rp_->set_numUpdatesPerSphere( mc_eye_left  , LRH->data[0] );
    rp_->set_numUpdatesPerSphere( mc_eye_right , LRH->data[1] );
    rp_->set_numUpdatesPerSphere( mc_head      , LRH->data[2] );
  }
  else
    ROS_ERROR("Incorrect diminsion for setArrLength");
}








//