#include <Eigen/Dense>
#include <cstring>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>

#include "dreamerJointPublisher.h"
#include "dreamerController.h"
#include "dreamerHighLevelController.h"
#include "Waypoint.h"


// Max nodeRate on my Acer laptop = ~300Hz
const double nodeRate = 50.0;
const double JOINT_LIM_BOUND = .9;
// Print rate should be a multiple of the node rate so it will actually print correctly
const double printRate = nodeRate/10.0;


// Task List
const std::string TASK_NO_TASK 						= "TASK_NO_TASK";
const std::string TASK_GO_TO_POINT_HEAD_PRIORITY    = "TASK_GO_TO_POINT_HEAD_PRIORITY";
const std::string TASK_GO_TO_POINT_EYE_PRIORITY     = "TASK_GO_TO_POINT_EYE_PRIORITY";
const std::string TASK_FOLLOW_WAYPOINTS 			= "TASK_FOLLOW_WAYPOINTS";


// Behavior List
const std::string BEHAVIOR_NO_BEHAVIOR = "BEHAVIOR_NO_BEHAVIOR";
const std::string BEHAVIOR_HEAD_PRIORITY_TEST = "BEHAVIOR_HEAD_PRIORITY_TEST";
const std::string BEHAVIOR_EYE_PRIORITY_TEST = "BEHAVIOR_EYE_PRIORITY_TEST";
const std::string BEHAVIOR_FOLLOW_WAYPOINTS = "BEHAVIOR_FOLLOW_WAYPOINTS";
const std::string BEHAVIOR_FOLLOW_CIRCLE = "BEHAVIOR_FOLLOW_CIRCLE";
const std::string BEHAVIOR_GO_HOME = "BEHAVIOR_GO_HOME";








/**
 * Generic Constructor
 * Notes: Low level controller and publisher are created in dreamerHighLevelController.h file.
 */
dreamerHighLevelController::dreamerHighLevelController(void){
	lowLevelControl = false; // Denotes whether low level control is off or on
	nodeInterval = 0; // Debugging: saves node loop rate
	printInterval = 0; // Debugging: saves print loop rate
	relativeTime = 0; // Debugging: how long the program has been running

	// Initialize tasks and behaviors to nothing
	currentBehavior = BEHAVIOR_NO_BEHAVIOR;
	currentTask = TASK_NO_TASK;
	behaviorCommanded = false;
	taskCommanded = false;

	taskIndex = 0; // Save the index of the current task
	taskInitTime = 0; // Save the start time of each task
	initTaskQ = Eigen::VectorXd::Zero(7); // Save initial joint position of each task
}


// Generic Destructor
dreamerHighLevelController::~dreamerHighLevelController(void){}






/**
 * Function: Published to RVIZ and low level
 * Inputs: None
 * Return: None
 */
void dreamerHighLevelController::sendCommand(void){
	// Send to Low Level if there is a behavior running AND low level is ON
	if(lowLevelControl && (currentBehavior != BEHAVIOR_NO_BEHAVIOR) && (currentBehavior != BEHAVIOR_GO_HOME)){
		std::cout << "Sending Low Level Commands" << std::endl;	
	}

	// Visualize the new joints in RVIZ
	pub.publishJoints();
}



/**
 * Function: Resets behavior and task variables
 * Inputs: None
 * Return: None
 */
void dreamerHighLevelController::resetAll(void){
	currentBehavior = BEHAVIOR_NO_BEHAVIOR;
	currentTask = TASK_NO_TASK;
	behaviorCommanded = false;
	taskCommanded = false;
	taskIndex = 0;
	taskParams.clear();
	taskList.clear();
}



/**
 * Function: Bounds the joint value so we don't go over physical joint limits
 * Inputs: joint command value, joint name, joint max value, joint min value
 * Return: bounded joint command value
 */
double dreamerHighLevelController::jointCmdBound(const double val, const std::string jointName, const double jmax, const double jmin){
    if (val >= JOINT_LIM_BOUND*jmax){
        std::cout << "MAX Software Joint HIT! for joint " << jointName;
        return JOINT_LIM_BOUND*jmax;
    }
    else if (val <= JOINT_LIM_BOUND*jmin){
        std::cout << "MAX Software Joint HIT! for joint " << jointName;
        return JOINT_LIM_BOUND*jmin;
    }
    else
        return val;
}



/**
 * Function: Update dreamerController class joint list, 
 * Inputs: Reference to controller class to update, reference to publisher class, Vector of head joint values, time since last update
 * Return: None
 */
void dreamerHighLevelController::updateHeadJoints(const Eigen::VectorXd& headJoints, const double interval){
	lowCtrl.updateGazeFocus(interval);
	for(int i = 0; i<headJoints.size(); i++){
		// Load all joint information/bounds
		std::string jointName = lowCtrl.kinematics.JIndexToNames[i]; 
		double jointMax = pub.freeJoints[jointName]["max"];
		double jointMin = pub.freeJoints[jointName]["min"];
		
		// Bound the joint
		double jointVal = jointCmdBound(headJoints(i), jointName, jointMax, jointMin);
		
		// Save to joint publisher
		pub.freeJoints[jointName]["position"] = jointVal;
		
		// Save to headKinematics Joint list
		lowCtrl.kinematics.Jlist(i) = jointVal;
	}
}



/************************** Behavior Control **************************/

/**
 * Function: Helper function to initialize behavior variables
 * Inputs: None
 * Returns: None
 */
void dreamerHighLevelController::executeBehavior(void){
	behaviorCommanded = true;
	taskIndex = 0;
	lowCtrl.movement_complete = false;
	taskInitTime = ros::Time::now().toSec();
	currentTask = taskList[taskIndex];
	initTaskQ = lowCtrl.kinematics.Jlist;
}


/**
 * Function: Defines high level description of behavior
 * Inputs: None
 * Returns: None
 * Notes: 
 */
void dreamerHighLevelController::behaviorLogic(void){
	// If no behavior in progress, clear all and return
	if(currentBehavior == BEHAVIOR_NO_BEHAVIOR || behaviorCommanded){
		return;
	}

	else if(currentBehavior == BEHAVIOR_HEAD_PRIORITY_TEST){
			for (int i=0; i<7; i++)
				taskList.push_back(TASK_GO_TO_POINT_HEAD_PRIORITY);

			double duration = 3;
			for (int i=0; i<7; i++){
				double scale = .2;
				// Declare Eigen class for points
				Eigen::Vector3d tempHead(.75, 0, lowCtrl.kinematics.l1);
				Eigen::Vector3d tempEyes(1.0, scale * std::sin(2*M_PI*i / 6.0), lowCtrl.kinematics.l1 + scale * std::cos(2*M_PI*i / 6.0));
				
				// Create Waypoint with Eigen points
				Waypoint wHead(tempHead, duration);
				Waypoint wEyes(tempEyes, duration);
				
				// Make Vector specifying eye and head gaze locations and times
				std::vector<Waypoint> tempVec;
				tempVec.push_back(wHead);
				tempVec.push_back(wEyes);

				// Add Waypoint to taskParams
				taskParams.push_back(tempVec);

			}
			// Initialize behavior tracking
			executeBehavior();

	}
}



/************************** Task Control **************************/

/**
 * Function: Initilizes task variables for jointLogic
 * Inputs: None
 * Returns: None
 */
void dreamerHighLevelController::taskLogic(void){
	
	if(currentTask == TASK_NO_TASK || taskCommanded)
		return;
	
	// Retrieve current task
	currentTask = taskList[taskIndex];
	
	if(currentTask == TASK_GO_TO_POINT_HEAD_PRIORITY){
		// Get task variables
		Eigen::Vector3d xyzHead = taskParams[taskIndex][0].point;
		Eigen::Vector3d xyzEyes = taskParams[taskIndex][1].point;
		// Initialize focus point
		lowCtrl.initializeHeadEyeFocusPoint(xyzHead, xyzEyes);	
		taskCommanded = true;
		initTaskQ = lowCtrl.kinematics.Jlist;
	
	}
}



/************************** Joint Control **************************/

/**
 * Function: Helper Function for transitioning to next task
 * Inputs: None
 * Returns: None
 */
void dreamerHighLevelController::nextTask(void){
	taskIndex++;
	if(taskIndex == taskList.size()){
		resetAll();
	}
	lowCtrl.movement_complete = false;
	taskCommanded = false;
	initTaskQ = lowCtrl.kinematics.Jlist;
	taskInitTime = ros::Time::now().toSec();
}


/**
 * Function: Calculates joint movements to fulfill current task
 * Inputs: None
 * Returns: None
 * Notes: Replaces state_logic from python code
 */
void dreamerHighLevelController::jointLogic(void){
		
	if(currentTask == TASK_GO_TO_POINT_HEAD_PRIORITY){
			// Load desired points and time
			Eigen::Vector3d xyzHead = taskParams[taskIndex][0].point;
			Eigen::Vector3d xyzEyes = taskParams[taskIndex][1].point;
			double endTime = taskParams[taskIndex][0].Dt;
			
			// Calculate new joint position
			Eigen::VectorXd ret = lowCtrl.headPriorityEyeTrajectoryLookAtPoint(xyzHead, xyzEyes, initTaskQ, taskInitTime, endTime);

			// Update and limit joint variables
			updateHeadJoints(ret, nodeInterval);
	}


	if(lowCtrl.movement_complete == true)
		nextTask();

}



/**
 * Function: Print out useful information about look and current behavior
 * Inputs: None
 * Returns: None
 */
void dreamerHighLevelController::printDebug(void){
	std::cout << "ROS Time:\t" << relativeTime << " s" << std::endl;
	std::cout << std::endl;
	std::cout << "Behavior:\t" << currentBehavior << std::endl;
	std::cout << "Task:\t\t" << currentTask << std::endl;
	std::cout << std::endl;
	std::cout << "Low Level:\t" << lowLevelControl << std::endl;
	std::cout << std::boolalpha; // Print lowLevelControl as a boolean
	// std::cout << "GUI Command:\t" << guiCommand << std::endl;
	
	std::cout << std::endl;
	std::cout << "Node Rate:\t" << 1/nodeInterval << " Hz" << std::endl;
	std::cout << "Print Rate:\t" << 1/printInterval << " Hz" << std::endl;
}



/**
 * Primary loop
 * Everything happens here
 */
void dreamerHighLevelController::loop(void){
	// Save program start time
	double initTime = ros::Time::now().toSec();

	// Debugging Markers
	// ros::NodeHandle handler;
	// ros::Publisher marker_pub = handler.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	// int shape = visualization_msgs::Marker::CUBE;
	
	std::cout << "Beginning loop" << std::endl;
	double loopCurrent = ros::Time::now().toSec();
	double loopLast = loopCurrent;
	double printLast = loopCurrent;

	// Specify the loop rate and allow ROS "sleep" command
	ros::Rate r(nodeRate);
	
	// Debugging behavior
	currentBehavior = BEHAVIOR_HEAD_PRIORITY_TEST;


	/**
	 * Loop Structure:
	 * 	1. Gather timing variables. Necessary for printing rate and 
	 * 		velocity calculations
	 * 	2. Select and load behavior
	 * 	3. Update task variables specified by behavior
	 * 	4. Calculate joint movements based on the current task
	 * 		Move to next task if current task completed
	 * 	5. Visualize joint changes in RVIZ/Low Level
	 * 	6. Change visualized focus length
	 * 	7. Print debugging information at specified rate
	 * 	8. ROS sleep until next cycle
	 */
	while(ros::ok()){
		// Clock the loop
		loopLast = loopCurrent;
		loopCurrent = ros::Time::now().toSec();
		nodeInterval = loopCurrent - loopLast;
		relativeTime = loopCurrent - initTime;
		
		behaviorLogic();

		taskLogic();

		jointLogic();


		// Send to RVIZ and low level
		sendCommand();

		// Change the focus length to the correct distance
		lowCtrl.publishFocusLength();


		// Print debugging at printRate
		if( (loopCurrent - printLast) > (1/printRate) ){
			printInterval = loopCurrent - printLast;
			printLast = loopCurrent;
			printDebug();
		}
		

		// Debugging Marker
		// visualization_msgs::Marker marker;
		// marker.header.frame_id = "/my_world_neck";
		// marker.header.stamp = ros::Time::now();
		// marker.ns = "basic_shapes";
	 //    marker.id = 0;
		// marker.type = shape;
		// marker.action = visualization_msgs::Marker::ADD;
	 //    marker.pose.position.x = 1;
	 //    marker.pose.position.y = 0;
	 //    marker.pose.position.z = lowCtrl.kinematics.l1+.3;
  //       marker.scale.x = .01;
	 //    marker.scale.y = .01;
	 //    marker.scale.z = .01;
	 //    marker.color.r = 0.0f;
	 //    marker.color.g = 1.0f;
	 //    marker.color.b = 1.0f;
	 //    marker.color.a = 1.0;
	 //    marker.lifetime = ros::Duration();
		// marker_pub.publish(marker);


		// Sleep until next interval
		r.sleep();


	}

}




/**
 * Main
 * Inputs: Optional ROS Options
 * Returns: Shouldn't
 */
int main(int argc, char** argv){
	// Start ROS time
	ros::Time::init();

	// Take over the RVIZ publishing node
	ros::init(argc, argv, "dreamer_head_behavior");
	
	// Declare High Level Controller
	dreamerHighLevelController highCtrl;
	
	// Debugging marker
	// ros::init(argc, argv, "basic_shapes");

	// Begin loop
	highCtrl.loop();
	
	return 0;
}


