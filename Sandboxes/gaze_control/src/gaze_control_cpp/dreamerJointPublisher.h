/**
 * dreamerJointPublisher.h
 * Holds robot joint limits
 * Publishes robot joint list to RVIZ
 */

#ifndef dreamerJointPublisher_h
#define dreamerJointPublisher_h

#include <tinyxml.h>
#include <cstring>
#include <vector>
#include <map>

#include "ros/ros.h"

class dreamerJointPublisher{
public:
	// Keep original joint order
	std::vector<std::string> jointList;

	// Keeps track of individual joint parameters
	std::map < std::string, std::map<std::string, double> > freeJoints;

	ros::NodeHandle jointNodeHandler;
	ros::Publisher jointPublisher;

	dreamerJointPublisher(void);
	~dreamerJointPublisher(void);
	
	/**
	 * Function: Load joint information from dreamer's URDF model
	 * Inputs: ROS Node handle class
	 * Returns: None
	 */
	void loadJointInformation(ros::NodeHandle);

	/**
	 * Function: Send joint positions to RVIZ
	 * Inputs: None
	 * Returns: None
	 */
	void publishJoints(void);

	/**
	 * Function: Prints out each free joint and it corresponding values
	 * Inputs: None
	 * Returns: None
	 */
	void printDebug(void);

};

#endif