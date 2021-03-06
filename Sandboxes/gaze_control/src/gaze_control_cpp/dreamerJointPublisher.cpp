/**
 * dreamerJointPublisher.cpp
 * Holds robot joint limits
 * Publishes robot joint list to RVIZ
 */

#include <iostream>
#include <cstring>
#include <cmath>
#include <vector>
#include <map>

#include "ros/ros.h"
#include <tinyxml.h>
#include <sensor_msgs/JointState.h>

#include "dreamerJointPublisher.h"

// Generic Constructor:
// Loads joints from external node
// Initializes publishing node
dreamerJointPublisher::dreamerJointPublisher(){
	loadJointInformation(jointNodeHandler);
	jointPublisher = jointNodeHandler.advertise<sensor_msgs::JointState>("joint_states", 5);
}

// Generic Destructor
dreamerJointPublisher::~dreamerJointPublisher(void){}


/**
 * Function: Load joint information from dreamer's URDF model
 * Inputs: ROS Node handle class
 * Returns: None
 */
void dreamerJointPublisher::loadJointInformation(ros::NodeHandle n){
	std::string retrieve;
	// Query for "robot_description" parameter
	if(jointNodeHandler.hasParam("robot_description")){
		// Place retrieved parameter in string
		jointNodeHandler.getParam("robot_description", retrieve);
		// std::cout << "Loaded Robot Description" << std::endl;

		// Declare xml parser
		TiXmlDocument robotXML;
		if(robotXML.Parse(retrieve.c_str(), 0, TIXML_ENCODING_UTF8)){

			// Pick out the first element in the xml
			TiXmlHandle hrob(&robotXML);
			TiXmlElement *ele = hrob.FirstChildElement("robot").FirstChildElement().ToElement();

			// Iterate through each element 
			for(ele; ele; ele = ele->NextSiblingElement()){
				// Only retrieve "joint" elements
				if(ele->ValueStr() == "joint"){
					std::string type = ele->Attribute("type");					
					std::string name = ele->Attribute("name");
					
					// Ignore fixed/floating joints
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
					if(jointNodeHandler.hasParam("zeros")){
						// Place retrieved parameter in string
						jointNodeHandler.getParam("zeros", zeroes);
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
				}

			}

		}

		std::cout << "Dreamer Joints loaded" << std::endl;
	}

	else
		std::cout << "Failed to find robot parameters" << std::endl;
	
}


/**
 * Function: Send joint positions to RVIZ
 * Inputs: None
 * Returns: None
 */
void dreamerJointPublisher::publishJoints(void){	
	// std::cout << "Publishing joints" << std::endl;
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


/**
 * Function: Prints out each free joint and it corresponding values
 * Inputs: None
 * Returns: None
 */
void dreamerJointPublisher::printDebug(void){
	std::map < std::string, std::map<std::string, double> >::iterator it;
	for(it = freeJoints.begin(); it != freeJoints.end(); it++){
		std::cout << it->first << ":\t";
		std::cout << "min: " << it->second["min"];
		std::cout << "\tmax: " << it->second["max"];
		std::cout << "\tzero: " << it->second["zero"] << std::endl;
	}
}
