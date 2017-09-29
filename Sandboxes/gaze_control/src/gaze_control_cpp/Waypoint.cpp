/**
 * Waypoint.cpp
 * Provides a wrapper to contain a point, a time, and a special parameter
 */

#include "Waypoint.h"
#include <Eigen/Dense>

Waypoint::Waypoint(void){
	point = Eigen::VectorXd::Zero(3);
	Dt = 0;
	special = 0;
}

Waypoint::Waypoint(const Eigen::Vector3d p, const double t){
	point = p;
	Dt = t;
	special = 0;
}

Waypoint::Waypoint(const Eigen::Vector3d p, const double t, const double sp ){
	point = p;
	Dt = t;
	special = sp;
}

Waypoint::~Waypoint(void){}
