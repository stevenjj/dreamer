#include <Eigen/Dense>
#include <cstring>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include <visualization_msgs/Marker.h>
#include <gaze_control/HeadJointCmd.h>
#include <gaze_control/RunProgram.h>

#include "dreamerJointPublisher.h"
#include "dreamerController.h"
#include "dreamerHighLevelController.h"
#include "Waypoint.h"



// Task List
const std::string TASK_NO_TASK 						= "TASK_NO_TASK";
const std::string TASK_GO_TO_POINT_HEAD_PRIORITY    = "TASK_GO_TO_POINT_HEAD_PRIORITY";
const std::string TASK_GO_TO_POINT_EYE_PRIORITY     = "TASK_GO_TO_POINT_EYE_PRIORITY";
const std::string TASK_FOLLOW_WAYPOINTS 			= "TASK_FOLLOW_WAYPOINTS";
const std::string TASK_GO_TO_POINT_CONSTANT_VELOCITY = "TASK_GO_TO_POINT_CONSTANT_VELOCITY";

// Behavior List
const std::string BEHAVIOR_NO_BEHAVIOR = "BEHAVIOR_NO_BEHAVIOR";
const std::string BEHAVIOR_HEAD_PRIORITY_TEST = "BEHAVIOR_HEAD_PRIORITY_TEST";
const std::string BEHAVIOR_EYE_PRIORITY_TEST = "BEHAVIOR_EYE_PRIORITY_TEST";
const std::string BEHAVIOR_FOLLOW_WAYPOINTS = "BEHAVIOR_FOLLOW_WAYPOINTS";
const std::string BEHAVIOR_FOLLOW_CIRCLE = "BEHAVIOR_FOLLOW_CIRCLE";
const std::string BEHAVIOR_GO_HOME = "BEHAVIOR_GO_HOME";
const std::string BEHAVIOR_POINT_CONSTANT_VELOCITY = "BEHAVIOR_POINT_CONSTANT_VELOCITY";

// GUI List
const std::string INVALID_CMD = "INVALID_CMD";
const std::string NO_COMMAND = "NO_COMMAND";
const std::string LOW_LEVEL_OFF = "LOW_LEVEL_OFF";
const std::string LOW_LEVEL_ON = "LOW_LEVEL_ON";
const std::string STATE_TO_IDLE = "STATE_TO_IDLE";
const std::string GO_HOME = "GO_HOME";
const std::string HEAD_PRIORITY_TEST = "HEAD_PRIORITY_TEST";
const std::string EYE_PRIORITY_TEST = "EYE_PRIORITY_TEST";
const std::string CONSTANT_VELOCITY_TEST = "CONSTANT_VELOCITY_TEST";


// Max nodeRate on my Acer laptop = ~300Hz
const double nodeRate = 25.0;
const double JOINT_LIM_BOUND = .9;
// Print rate should be a multiple of the node rate so it will actually print correctly
const double printRate = nodeRate/5.0;
// Global GUI param variable

// Low Level stuff
const int LOW_LEVEL_FREQ = 550;
const short WAIT_TIME_GO_HOME = 3;









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

	// Initialize GUI stuff
	GUICommanded = NO_COMMAND;

	taskIndex = 0; // Save the index of the current task
	taskInitTime = 0; // Save the start time of each task
	initTaskQ = lowCtrl.kinematics.Jlist; // Save initial joint position of each task
}


// Generic Destructor
dreamerHighLevelController::~dreamerHighLevelController(void){}




/********************************* GUI Callback *********************************/

/**
 * Function: Handler for GUI interrupt/button press
 * 			 Utilizes the python GUI with some changes to naming
 * Input: number designation of the command button pressed
 */
void dreamerHighLevelController::GUICallback(const std_msgs::Int8 msg){
	int received = msg.data;
	ros::NodeHandle handlerCtrlDeq;
	switch(received){
		case 0: GUICommanded = LOW_LEVEL_OFF;
				lowLevelControl = false;
				break;

		case 1: GUICommanded = LOW_LEVEL_ON;
				lowLevelControl = true;
				ctrldeqClient = handlerHigh.serviceClient<gaze_control::HeadJointCmd>("ctrl_deq_append");
				runPrgClient = handlerHigh.serviceClient<gaze_control::RunProgram>("ctrl_deq_append");
				break;

		case 2: resetAll(); 
				GUICommanded = STATE_TO_IDLE; 
				break;

		case 3: resetAll(); 
				GUICommanded = GO_HOME; 
				currentBehavior = BEHAVIOR_GO_HOME;
				break;

		case 4: resetAll(); 
				GUICommanded = EYE_PRIORITY_TEST; 
				currentBehavior = BEHAVIOR_EYE_PRIORITY_TEST; 
				break;

		case 5: resetAll(); 
				GUICommanded = HEAD_PRIORITY_TEST; 
				currentBehavior = BEHAVIOR_HEAD_PRIORITY_TEST; 
				break;

		case 6: resetAll();
				GUICommanded = CONSTANT_VELOCITY_TEST; 
				currentBehavior = BEHAVIOR_POINT_CONSTANT_VELOCITY;
				break; 

		default: GUICommanded = INVALID_CMD; break;
	}

}	




/***************************** Low Level Functions *****************************/

/**
 * Function: Sends home command to low level controller
 * Inputs: None
 * Returns: None
 */
void dreamerHighLevelController::sendGoHomeCommand(void){
	gaze_control::HeadJointCmd hjc;
	std::vector<short> jointMap;
	std::vector<float> joint_val;
	for(int i=0; i<7; i++){
		jointMap.push_back(i);
		joint_val.push_back(0);
	}

	hjc.request.numCtrlSteps.data = (short)(LOW_LEVEL_FREQ * WAIT_TIME_GO_HOME);
	hjc.request.joint_mapping.data = jointMap;
	hjc.request.q_cmd_radians.data = joint_val;

	if(ctrldeqClient.call(hjc)) {
		ROS_INFO("Called ctrldeq with GO_HOME: %d", hjc.response.success.data);
	}
	else {
		ROS_ERROR("Failed to call ctrl_deq_append");
	}
}


/**
 * Function: Send commands to low level controller
 * Inputs: None
 * Returns: None
 */
void dreamerHighLevelController::sendLowLevelCommand(void){
	gaze_control::HeadJointCmd hjc;
	std::vector<short> jointMap;
	std::vector<float> joint_val;
	for(int i=0; i<7; i++){
		jointMap.push_back(i);
		joint_val.push_back(lowCtrl.kinematics.Jlist(i));
	}

	hjc.request.numCtrlSteps.data = (short)(LOW_LEVEL_FREQ * (1/nodeRate));
	hjc.request.joint_mapping.data = jointMap;
	hjc.request.q_cmd_radians.data = joint_val;

	if(ctrldeqClient.call(hjc)) {
		ROS_INFO("Called ctrldeq: %d", hjc.response.success.data);
	}
	else {
		ROS_ERROR("Failed to call ctrl_deq_append");
	}
}


/**
 * Function: Published to RVIZ and low level
 * Inputs: None
 * Return: None
 */
void dreamerHighLevelController::sendCommand(void){
	// Send to Low Level if there is a behavior running AND low level is ON
	if(lowLevelControl && 
		(currentBehavior != BEHAVIOR_NO_BEHAVIOR) && 
		(currentBehavior != BEHAVIOR_GO_HOME)){
			sendLowLevelCommand();
	}

	// Visualize the new joints in RVIZ
	lowCtrl.pub.publishJoints();
}





/********************** Helper functions: Resetting things **********************/

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
 * Function: Resets joints and gaze focuses to home positions
 * Inputs: None
 * Returns: None
 */
void dreamerHighLevelController::resetAllJoints(void){
	lowCtrl.kinematics.Jlist = Eigen::VectorXd::Zero(7);
	updateHeadJoints(lowCtrl.kinematics.Jlist, 1);
	lowCtrl.resetGazeFocus();
}




/******************** Helper functions: Updating robot joints ********************/

/**
 * Function: Bounds the joint value so we don't go over physical joint limits
 * Inputs: joint command value, joint name, joint max value, joint min value
 * Return: bounded joint command value
 */
double dreamerHighLevelController::jointCmdBound(const double val, const std::string jointName, const double jmax, const double jmin){
    if (val >= JOINT_LIM_BOUND*jmax){
        std::cout << "MAX Software Joint HIT! for joint " << jointName << std::endl;
        return JOINT_LIM_BOUND*jmax;
    }
    else if (val <= JOINT_LIM_BOUND*jmin){
        std::cout << "MIN Software Joint HIT! for joint " << jointName << std::endl;
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
		double jointMax = lowCtrl.pub.freeJoints[jointName]["max"];
		double jointMin = lowCtrl.pub.freeJoints[jointName]["min"];
		
		// Bound the joint
		double jointVal = jointCmdBound(headJoints(i), jointName, jointMax, jointMin);
		
		// Save to joint publisher
		lowCtrl.pub.freeJoints[jointName]["position"] = jointVal;
		
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

	else if(currentBehavior == BEHAVIOR_GO_HOME){
		currentBehavior = BEHAVIOR_NO_BEHAVIOR;
		currentTask = TASK_NO_TASK;
		sendGoHomeCommand();
		ros::Duration(3.5).sleep();
		resetAllJoints();
	}

	else if(currentBehavior == BEHAVIOR_HEAD_PRIORITY_TEST){
			for (int i=0; i<7; i++)
				taskList.push_back(TASK_GO_TO_POINT_HEAD_PRIORITY);

			double duration = 4;
			for (int i=0; i<7; i++){
				double scale = .1;
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

			// More Testing code
			// taskList.push_back(TASK_GO_TO_POINT_HEAD_PRIORITY);
			// double duration = 10.0;
			// Eigen::Vector3d pointTest(2, 0, lowCtrl.kinematics.l1+.05);
			// Waypoint wayTest(pointTest, duration);
			// std::vector<Waypoint> tempVec;
			// tempVec.push_back(wayTest);
			// tempVec.push_back(wayTest);
			// taskParams.push_back(tempVec);


			// Initialize behavior tracking
			executeBehavior();
	}


	else if(currentBehavior == BEHAVIOR_EYE_PRIORITY_TEST){
			for (int i=0; i<7; i++)
				taskList.push_back(TASK_GO_TO_POINT_EYE_PRIORITY);

			double duration = 3;
			for (int i=0; i<7; i++){
				double scale = .6;
				// Declare Eigen class for points
				Eigen::Vector3d tempHead(1.0, scale * std::sin(2*M_PI*i / 6.0), lowCtrl.kinematics.l1 + scale * std::cos(2*M_PI*i / 6.0));
				Eigen::Vector3d tempEyes(1.0, scale * std::sin(2*M_PI*i / 6.0), lowCtrl.kinematics.l1 + scale * std::cos(2*M_PI*i / 6.0));
				// Eigen::Vector3d tempEyes(2.0, 0, lowCtrl.kinematics.l1);
				
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
	else if(currentBehavior == BEHAVIOR_POINT_CONSTANT_VELOCITY){
			for (int i=0; i<7; i++)
				taskList.push_back(TASK_GO_TO_POINT_CONSTANT_VELOCITY);

			double duration = 1337;
			for (int i=0; i<7; i++){
				double scale = .5;
				// Declare Eigen class for points
				Eigen::Vector3d tempHead(1.0, scale * std::sin(2*M_PI*i / 6.0), lowCtrl.kinematics.l1 + scale * std::cos(2*M_PI*i / 6.0));
				Eigen::Vector3d tempEyes(1.0, scale * std::sin(2*M_PI*i / 6.0), lowCtrl.kinematics.l1 + scale * std::cos(2*M_PI*i / 6.0));
				// Eigen::Vector3d tempEyes(2.0, 0, lowCtrl.kinematics.l1);
				
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
		// Save current joint configuration
		initTaskQ = lowCtrl.kinematics.Jlist;
	}
	else if(currentTask == TASK_GO_TO_POINT_EYE_PRIORITY){
		// Get task variables
		Eigen::Vector3d xyzHead = taskParams[taskIndex][0].point;
		Eigen::Vector3d xyzEyes = taskParams[taskIndex][1].point;
		// Initialize focus point
		lowCtrl.initializeHeadEyeFocusPoint(xyzHead, xyzEyes);	
		taskCommanded = true;
		// Save current joint configuration
		initTaskQ = lowCtrl.kinematics.Jlist;
	}
	else if(currentTask == TASK_GO_TO_POINT_CONSTANT_VELOCITY){
		// Get task variables
		Eigen::Vector3d xyzHead = taskParams[taskIndex][0].point;
		Eigen::Vector3d xyzEyes = taskParams[taskIndex][1].point;
		// Initialize focus point
		lowCtrl.initializeHeadEyeFocusPoint(xyzHead, xyzEyes);	
		taskCommanded = true;
		// Save current joint configuration
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
	else if(currentTask == TASK_GO_TO_POINT_EYE_PRIORITY){
			// Load desired points and time
			Eigen::Vector3d xyzHead = taskParams[taskIndex][0].point;
			Eigen::Vector3d xyzEyes = taskParams[taskIndex][1].point;
			double endTime = taskParams[taskIndex][0].Dt;
			
			// Calculate new joint position
			Eigen::VectorXd ret = lowCtrl.eyePriorityHeadTrajectoryLookAtPoint(xyzHead, xyzEyes, initTaskQ, taskInitTime, endTime);

			// Update and limit joint variables
			updateHeadJoints(ret, nodeInterval);
	}
	else if(currentTask == TASK_GO_TO_POINT_CONSTANT_VELOCITY){
			// Load desired points and time
			Eigen::Vector3d xyzHead = taskParams[taskIndex][0].point;
			Eigen::Vector3d xyzEyes = taskParams[taskIndex][1].point;
			
			// Calculate new joint position
			Eigen::VectorXd ret = lowCtrl.constantVelocityLookAtPoint(xyzHead, xyzEyes, initTaskQ, taskInitTime);

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
	std::cout << "GUI Command:\t" << GUICommanded << std::endl;
	
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
	ros::NodeHandle handler;
	ros::Publisher marker_pub = handler.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	int shape = visualization_msgs::Marker::CUBE;
	
	std::cout << "Beginning loop" << std::endl;
	double loopCurrent = ros::Time::now().toSec();
	double loopLast = loopCurrent;
	double printLast = loopCurrent;

	// Specify the loop rate and allow ROS "sleep" command
	ros::Rate r(nodeRate);
	
	// Debugging behavior
	// currentBehavior = BEHAVIOR_HEAD_PRIORITY_TEST;


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
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/my_world_neck";
		marker.header.stamp = ros::Time::now();
		marker.ns = "basic_shapes";
	    marker.id = 0;
		marker.type = shape;
		marker.action = visualization_msgs::Marker::ADD;
	    marker.pose.position.x = 1;
	    marker.pose.position.y = 0;
	    marker.pose.position.z = lowCtrl.kinematics.l1+.7;
        marker.scale.x = .01;
	    marker.scale.y = .01;
	    marker.scale.z = .01;
	    marker.color.r = 0.0f;
	    marker.color.g = 1.0f;
	    marker.color.b = 1.0f;
	    marker.color.a = 1.0;
	    marker.lifetime = ros::Duration();
		marker_pub.publish(marker);


		// Sleep until next interval
		r.sleep();
		// Check for GUI input
		ros::spinOnce();


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

	// Listen for the GUI
	ros::NodeHandle handlerMain;
	ros::Subscriber GUI_sub = handlerMain.subscribe("GUI_cmd_int", 8, &dreamerHighLevelController::GUICallback, &highCtrl);
	
	// Debugging marker
	ros::init(argc, argv, "basic_shapes");

	// Begin loop
	highCtrl.loop();
	
	return 0;
}


