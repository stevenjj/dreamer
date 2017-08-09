#ifndef dreamerJointPublisher_h
#define dreamerJointPublisher_h

#include <urdf/model.h>
#include <tinyxml.h>
#include <cstring>
#include <vector>
#include <map>

#include "ros/ros.h"

class dreamerJointPublisher{
public:
	std::vector<std::string> jointList;
	std::map < std::string, std::map<std::string, double> > freeJoints;

	ros::NodeHandle n;
	ros::Publisher jointPublisher;
	dreamerJointPublisher(void);
	~dreamerJointPublisher(void);
	void loadJointInformation(ros::NodeHandle);
	void printDebug(void);
	void publishJoints(void);

};

#endif