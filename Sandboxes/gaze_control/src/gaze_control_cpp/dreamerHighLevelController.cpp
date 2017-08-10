#include <Eigen/Dense>
#include "ros/ros.h"
#include <cstring>
#include "dreamerJointPublisher.h"
#include "dreamerController.h"

const double nodeRate = 10.0;
const double JOINT_LIM_BOUND = .9;

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

void updateHeadJoints(dreamerController& ctrl, dreamerJointPublisher& pub, const Eigen::VectorXd& headJoints, const double interval){
	ctrl.updateGazeFocus(interval);
	for(int i = 0; i<headJoints.size(); i++){
		pub.freeJoints[ctrl.kinematics.JIndexToNames[i]]["position"] = headJoints(i);
	}
	ctrl.kinematics.Jlist = headJoints;
}


int main(int argc, char** argv){
	ros::Time::init();
	ros::init(argc, argv, "dreamer_head_behavior");
	

	dreamerJointPublisher pub;
	dreamerController ctrl;

	// Reset head to home position
	ctrl.kinematics.Jlist = Eigen::VectorXd::Zero(7);
	// Store the initial joint configuration
	Eigen::VectorXd J = ctrl.kinematics.Jlist;
	// updateHeadJoints(ctrl, pub, J);

	// Not reseting correctly
	// pub.publishJoints();
	
	// Declare parameters for movement function
	double initTime = ros::Time::now().toSec();
	Eigen::Vector3d xyzHead(1, .5, ctrl.kinematics.l1);
	Eigen::Vector3d xyzEye(1, .25, ctrl.kinematics.l1);
	double endTime = 2.5;
	ctrl.initializeHeadEyeFocusPoint(xyzHead, xyzEye);
	
	// Main loop
	double lastLoopTime = ros::Time::now().toSec();
	std::cout << "Beginning loop" << std::endl;
	while(ros::ok() && !ctrl.movement_complete){
		double interval =ros::Time::now().toSec() - lastLoopTime; 
		if(interval > (1.0 / nodeRate)) {
			// Movement isn't completely correct
			Eigen::VectorXd ret = ctrl.headPriorityEyeTrajectoryLookAtPoint(xyzHead, xyzEye, J, initTime, endTime);

			updateHeadJoints(ctrl, pub, ret, interval);

			pub.publishJoints();
			// Almost working
			ctrl.publishFocusLength();

			lastLoopTime = ros::Time::now().toSec();
		}
	}
	 

	return 0;
}