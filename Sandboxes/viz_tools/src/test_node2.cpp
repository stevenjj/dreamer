// #include "ros/ros.h"

// #include <viz_tools/ros_pub.h>
// //#include <ControlSystem/R5_Controller/R5_Definition.h>



// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "test_node_initialized");
  


//   ROS_Pub *rp;
//   sejong::JS_Channel jsc;
//   rp = ROS_Pub::get_ROS_Pub();
//   rp->new_js_channel(jsc, "ghost");


//   sejong::Vector q = sejong::Vector::Zero(NUM_Q);
//   q[NUM_Q-1] = 1.0;  // Sets w of Quaternion to make identity
//   double joint_pos_test = 0.0;
//   sejong::Quaternion quat(0.,0.,0.,1.);
//   q[VIRTUAL_X] = 4.;
//   q[VIRTUAL_Z] = 1.2;

//   q[leftAnklePitch] = 1.;
//   q[rightAnklePitch] = 1.;
//   q[leftHipYaw] = 1.;
//   q[leftAnkleRoll] = 0.2;
//   q[leftKneePitch] = 0.2;
//   q[rightKneePitch] = 1.57;
//   q[rightShoulderYaw] = -1.57;
//   q[leftShoulderYaw] = -1.57;
//   q[leftShoulderRoll] = 1.57;
//   q[rightShoulderRoll] = -1.57;
//   q[rightHipPitch] = 0.1;
//   q[rightElbowPitch] = 0.9;
//   q[leftElbowPitch] = -0.9;
//   q[leftWristPitch] = 0.9;
//   q[rightWristPitch] = -0.9;

//   ros::Rate loop_rate(30);
//   while (ros::ok())
//   {

//     sejong::convert(joint_pos_test, 0.0, 0.0, quat);
    


//     q[VIRTUAL_Rx] = quat.x();
//     q[VIRTUAL_Ry] = quat.y();
//     q[VIRTUAL_Rz] = quat.z();
//     q[NUM_Q-1]    = quat.w();

//     rp->pub_js(jsc,q);
//     joint_pos_test += 0.1;

//     loop_rate.sleep();
//   }


//   return 0;
// }