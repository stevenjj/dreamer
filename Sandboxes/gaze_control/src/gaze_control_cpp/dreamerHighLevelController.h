#ifndef dreamerHighLevelController_h
#define dreamerHighLevelController_h
#include <cstring>
#include "ros/ros.h"
#include "dreamerController.h"
#include "Waypoint.h"


class dreamerHighLevelController{
public:

	dreamerController lowCtrl;
	
	bool lowLevelControl;
	double nodeInterval;
	double printInterval;
	double relativeTime;

	std::string GUICommanded;
	std::string currentBehavior;
	std::string currentTask;

	bool behaviorCommanded;
	bool taskCommanded;

	double taskIndex;
	double taskInitTime;
	Eigen::VectorXd initTaskQ;

	Eigen::Vector3d currentDesiredHead;
	Eigen::Vector3d currentDesiredEyes;


	std::vector<std::string> taskList;
	std::vector< std::vector<Waypoint> > taskParams;
	std::vector<minJerkCoordinates> behaviorMin;

	ros::NodeHandle handlerHigh;
	ros::ServiceClient ctrldeqClient;
	ros::ServiceClient runPrgClient;


	dreamerHighLevelController(void);
	~dreamerHighLevelController(void);


	void GUICallback(const std_msgs::Int8);

	void sendLowLevelCommand(void);
	void sendGoHomeCommand(void);
	void sendCommand(void);
	
	void resetAll(void);
	void resetAllJoints(void);

	double jointCmdBound(const double, const std::string, const double, const double);
	void updateHeadJoints(const Eigen::VectorXd&, const double);

	void executeBehavior(void);
	void behaviorLogic(void);

	void taskLogic(void);

	void nextTask(void);
	void jointLogic(void);

	void printDebug(void);
	
	void loop(void);

};

#endif
