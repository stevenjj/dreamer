#include <Eigen/Dense>
#include "ros/ros.h"
#include <cstring>
#include "dreamerJointPublisher.h"
#include "dreamerController.h"
#include <visualization_msgs/Marker.h>


// Max nodeRate on my Acer laptop = ~300Hz
const double nodeRate = 100.0;
const double JOINT_LIM_BOUND = .9;

/**
 * Function: Bounds the joint value so we don't go over physical joint limits
 * Inputs: joint command value, joint name, joint max value, joint min value
 * Return: bounded joint command value
 */
double jointCmdBound(double val, std::string jointName, double jmax, double jmin){
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
 * Returns: None
 */
void updateHeadJoints(dreamerController& ctrl, dreamerJointPublisher& pub, const Eigen::VectorXd& headJoints, const double interval){
	ctrl.updateGazeFocus(interval);
	for(int i = 0; i<headJoints.size(); i++){
		// Load all joint information/bounds
		std::string jointName = ctrl.kinematics.JIndexToNames[i]; 
		double jointMax = pub.freeJoints[jointName]["max"];
		double jointMin = pub.freeJoints[jointName]["min"];
		
		// Bound the joint
		double jointVal = jointCmdBound(headJoints(i), jointName, jointMax, jointMin);
		
		// Save to joint publisher
		pub.freeJoints[jointName]["position"] = jointVal;
		
		// Save to headKinematics Joint list
		ctrl.kinematics.Jlist(i) = jointVal;
	}
}


int main(int argc, char** argv){
	// Start ROS time
	ros::Time::init();

	// Take over the RVIZ publishing node
	ros::init(argc, argv, "dreamer_head_behavior");
	
	// Declare variables
	dreamerJointPublisher pub;
	dreamerController ctrl;

	
		
	// Declare parameters for movement function
	double initTime = ros::Time::now().toSec();
	Eigen::Vector3d xyzHead(1, 0, ctrl.kinematics.l1);
	Eigen::Vector3d xyzEye(1, 0, ctrl.kinematics.l1+.02);
	double endTime = 5;
	ctrl.initializeHeadEyeFocusPoint(xyzHead, xyzEye);
	// Save initial joint configuration
	Eigen::VectorXd initJ = ctrl.kinematics.Jlist;


	// Debugging Markers
	ros::init(argc, argv, "basic_shapes");
	ros::NodeHandle handler;
	ros::Publisher marker_pub = handler.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	int shape = visualization_msgs::Marker::CUBE;



	/**
	 * Main Loop
	 * While(ROS is online AND the movement is not completed)
	 * 	Find the interval between loops(used for velocity calculation and timing the loop)
	 */
	
	std::cout << "Beginning loop" << std::endl;
	double loopCurrent = ros::Time::now().toSec();
	double loopLast = loopCurrent;
	// double loopSum = 0;
	// int loopCounter = 0;

	// Specify the loop rate
	ros::Rate r(nodeRate);
	
	while(ros::ok() && !ctrl.movement_complete){
		// Clock the loop
		loopLast = loopCurrent;
		loopCurrent = ros::Time::now().toSec();
		double interval = loopCurrent - loopLast;

		// // Used for loop rate debugging 
		// loopSum += interval;
		// loopCounter++;
		
		// Movement isn't completely correct, Eyes do not work correctly
		Eigen::VectorXd ret = ctrl.headPriorityEyeTrajectoryLookAtPoint(xyzHead, xyzEye, initJ, initTime, endTime);
		updateHeadJoints(ctrl, pub, ret, interval);

		pub.publishJoints();
		// Almost working
		ctrl.publishFocusLength();

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
	    marker.pose.position.z = ctrl.kinematics.l1 + .02;
        marker.scale.x = .01;
	    marker.scale.y = .01;
	    marker.scale.z = .01;
	    marker.color.r = 0.0f;
	    marker.color.g = 1.0f;
	    marker.color.b = 1.0f;
	    marker.color.a = 1.0;
	    marker.lifetime = ros::Duration();
		marker_pub.publish(marker);

		r.sleep();
	}
	// std::cout << "Average Loop Rate: " << 1.0/(loopSum/loopCounter) << " Hz" << std::endl;
	 

	return 0;
}
