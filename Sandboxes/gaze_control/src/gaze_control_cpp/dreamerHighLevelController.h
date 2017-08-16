#ifndef dreamerHighLevelController_h
#define dreamerHighLevelController_h
#include <cstring>
#include "dreamerJointPublisher.h"
#include "dreamerController.h"
#include "Waypoint.h"


class dreamerHighLevelController{
public:

	dreamerController lowCtrl;
	dreamerJointPublisher pub;
	
	bool lowLevelControl;
	double nodeInterval;
	double printInterval;
	double relativeTime;

	std::string currentBehavior;
	std::string currentTask;

	bool behaviorCommanded;
	bool taskCommanded;

	double taskIndex;
	double taskInitTime;
	Eigen::VectorXd initTaskQ;

	std::vector<std::string> taskList;
	std::vector< std::vector<Waypoint> > taskParams;


	dreamerHighLevelController(void);
	~dreamerHighLevelController(void);

	void sendCommand(void);
	void resetAll(void);

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
