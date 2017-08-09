#include <urdf/model.h>
#include <tinyxml.h>
#include <iostream>
#include <cstring>
#include <cmath>
#include <vector>
#include <map>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
// #include "std_msgs/String.h"

#include "dreamerJointPublisher.h"

dreamerJointPublisher::dreamerJointPublisher(){
	loadJointInformation(n);
	// jointPublisher = n.advertise<std_msgs::String>("joint_states", 5);
	jointPublisher = n.advertise<sensor_msgs::JointState>("joint_states", 5);
}


dreamerJointPublisher::~dreamerJointPublisher(void){}


void dreamerJointPublisher::loadJointInformation(ros::NodeHandle n){
	std::string retrieve;
	// Query for "robot_description" parameter
	if(n.hasParam("robot_description")){
		// Place retrieved parameter in string
		n.getParam("robot_description", retrieve);
		// std::cout << "Loaded Robot Description" << std::endl;

		// Declare xml parser
		TiXmlDocument robotXML;
		if(robotXML.Parse(retrieve.c_str(), 0, TIXML_ENCODING_UTF8)){

			// Pick out 
			TiXmlHandle hrob(&robotXML);
			TiXmlElement *ele = hrob.FirstChildElement("robot").FirstChildElement().ToElement();

			for(ele; ele; ele = ele->NextSiblingElement()){
				// Only retrieve "joint" elements
				if(ele->ValueStr() == "joint"){
					std::string type = ele->Attribute("type");					
					std::string name = ele->Attribute("name");
					
					// Ignore fixed/floating joints?
					if(type == "fixed" || type == "floating")
						continue;

					// Add joint to jointList vector
					jointList.push_back(name);

					double minval;
					double maxval;
					double zeroval;
					if(type == "continuous"){
						minval = -M_PI;
						maxval = M_PI;
					}
					else{
						try{
							TiXmlElement *limit = ele->FirstChildElement("limit");
							minval = atof(limit->Attribute("lower"));
							maxval = atof(limit->Attribute("upper"));
							// std::cout << "Retrieved " << name << " joint" << std::endl;
						}
						catch(...){
							std::cout << "Failed to get " << ele->Value() << " limits" << std::endl;
							continue;
						}
					}


					// Ignoring mimic tags
					std::string zeroes;
					if(n.hasParam("zeros")){
						// Place retrieved parameter in string
						n.getParam("zeros", zeroes);
						zeroval = 0; // TODO actually fill in correct zero value
					}
					else if(minval > 0 || maxval < 0)
						zeroval = ( maxval + minval ) / 2;
					else
						zeroval = 0;

					// Fill in map for joint information
					std::map<std::string, double> joint;
					joint["min"] = minval;
					joint["max"] = maxval;
					joint["zero"] = zeroval;
					joint["position"] = zeroval;

					// Continuous is True
					if(type == "continuous"){
						joint["continuous"] = 1;
					}

					// Add joint to free joint map
					freeJoints[name] = joint;

					// std::cout << ele->Attribute("name") << std::endl;
				}
			}


		}
		// robotXML.Print();
		std::cout << "Dreamer Joints loaded" << std::endl;
	}
	else
		std::cout << "Failed to find robot parameters" << std::endl;
}


void dreamerJointPublisher::publishJoints(void){
	std::cout << "Publishing joints" << std::endl;
	sensor_msgs::JointState msg;
	msg.header.stamp = ros::Time::now();

	for(int i = 0; i < jointList.size(); i++){
		if(freeJoints.find(jointList[i]) != freeJoints.end()){
			// std::cout << jointList[i] << std::endl;
			msg.name.push_back(jointList[i]);
			msg.position.push_back(freeJoints[jointList[i]]["position"]);
			// msg.position.push_back(.1);

		}
		else{
			msg.name.push_back(jointList[i]);
			msg.position.push_back(0);
		}
	 
	
	}
	// std::cout << msg << std::endl;
	jointPublisher.publish(msg);
	
}


void dreamerJointPublisher::printDebug(void){
	std::map < std::string, std::map<std::string, double> >::iterator it;
	for(it = freeJoints.begin(); it != freeJoints.end(); it++){
		std::cout << it->first << ":\t";
		std::cout << "min: " << it->second["min"];
		std::cout << "\tmax: " << it->second["max"];
		std::cout << "\tzero: " << it->second["zero"] << std::endl;
	}
}





/*
torso_lower_pitch
torso_upper_pitch
right_shoulder_extensor
right_shoulder_abductor
right_shoulder_rotator
right_elbow
right_wrist_rotator
right_wrist_pitch
right_wrist_yaw
left_shoulder_extensor
left_shoulder_abductor
left_shoulder_rotator
left_elbow
left_wrist_rotator
left_wrist_pitch
left_wrist_yaw
lower_neck_pitch
upper_neck_yaw
upper_neck_roll
upper_neck_pitch
eye_pitch
right_eye_yaw
left_eye_yaw
right_thumb_cmc
right_thumb_mcp
right_thumb_ip
right_pointer_finger_mcp
right_pointer_finger_pip
right_pointer_finger_dip
right_middle_finger_mcp
right_middle_finger_pip
right_middle_finger_dip
right_pinky_mcp
right_pinky_pip
right_pinky_dip
l_gripper_l_finger_joint
l_gripper_r_finger_joint
l_gripper_l_finger_tip_joint
l_gripper_r_finger_tip_joint

header: 
  seq: 4386
  stamp: 
    secs: 1502304064
    nsecs: 607492923
  frame_id: ''
name: ['torso_lower_pitch', 'torso_upper_pitch', 'right_shoulder_extensor', 'right_shoulder_abductor', 'right_shoulder_rotator', 'right_elbow', 'right_wrist_rotator', 'right_wrist_pitch', 'right_wrist_yaw', 'left_shoulder_extensor', 'left_shoulder_abductor', 'left_shoulder_rotator', 'left_elbow', 'left_wrist_rotator', 'left_wrist_pitch', 'left_wrist_yaw', 'lower_neck_pitch', 'upper_neck_yaw', 'upper_neck_roll', 'upper_neck_pitch', 'eye_pitch', 'right_eye_yaw', 'left_eye_yaw', 'right_thumb_cmc', 'right_thumb_mcp', 'right_thumb_ip', 'right_pointer_finger_mcp', 'right_pointer_finger_pip', 'right_pointer_finger_dip', 'right_middle_finger_mcp', 'right_middle_finger_pip', 'right_middle_finger_dip', 'right_pinky_mcp', 'right_pinky_pip', 'right_pinky_dip', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_r_finger_tip_joint']
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: []
effort: []

header: 
  seq: 4
  stamp: 
    secs: 1502304092
    nsecs: 640752163
  frame_id: ''
name: ['eye_pitch', 'l_gripper_l_finger_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_r_finger_joint', 'l_gripper_r_finger_tip_joint', 'left_elbow', 'left_eye_yaw', 'left_shoulder_abductor', 'left_shoulder_extensor', 'left_shoulder_rotator', 'left_wrist_pitch', 'left_wrist_rotator', 'left_wrist_yaw', 'lower_neck_pitch', 'right_elbow', 'right_eye_yaw', 'right_middle_finger_dip', 'right_middle_finger_mcp', 'right_middle_finger_pip', 'right_pinky_dip', 'right_pinky_mcp', 'right_pinky_pip', 'right_pointer_finger_dip', 'right_pointer_finger_mcp', 'right_pointer_finger_pip', 'right_shoulder_abductor', 'right_shoulder_extensor', 'right_shoulder_rotator', 'right_thumb_cmc', 'right_thumb_ip', 'right_thumb_mcp', 'right_wrist_pitch', 'right_wrist_rotator', 'right_wrist_yaw', 'torso_lower_pitch', 'torso_upper_pitch', 'upper_neck_pitch', 'upper_neck_roll', 'upper_neck_yaw']
position: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
velocity: []
effort: []

 */