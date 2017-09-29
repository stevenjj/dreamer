#include "minJerkCoordinates.h"
#include "Waypoint.h"
#include <iostream>
#include <vector>
#include <cmath>

const double l1 = 0.13849;
const double l2 = 0.12508;

minJerkCoordinates::minJerkCoordinates(minJerkSingle sX, minJerkSingle sY, minJerkSingle sZ) {
	x = sX;
	y = sY;
	z = sZ;
	sys = 'c';
}

minJerkCoordinates::minJerkCoordinates(minJerkSingle sX, minJerkSingle sY, minJerkSingle sZ, char system) {
	x = sX;
	y = sY;
	z = sZ;
	sys = system;
}


minJerkCoordinates::~minJerkCoordinates() {}

Eigen::Vector3d minJerkCoordinates::getPosition(double time) {
	if(sys == 'c') {
		return Eigen::Vector3d(x.getPosition(time), y.getPosition(time), z.getPosition(time));
	}
	else { // assume polar coordinates
		double xCoord = x.getPosition(time) * std::sin(z.getPosition(time)) * std::cos(y.getPosition(time));
		double yCoord = x.getPosition(time) * std::sin(z.getPosition(time)) * std::sin(y.getPosition(time));
		double zCoord = x.getPosition(time) * std::cos(z.getPosition(time)) + l1;
		return Eigen::Vector3d(xCoord, yCoord, zCoord);
	}	
}

double minJerkCoordinates::getTotalRunTime(void) {
	return x.getTotalRunTime();
}




std::vector<minJerkCoordinates> testBehavior(void) {
	double accuracy = 64;
	std::vector<Waypoint> y;
	double time = 10;
	double radius = .3;
	double distance = 1;
	y.push_back(Waypoint(Eigen::Vector3d(0,0,0), 0));
	y.push_back(Waypoint(Eigen::Vector3d(0,0,0), time));
	
	std::vector<Waypoint> x;
	std::vector<Waypoint> z;
	x.push_back(Waypoint(Eigen::Vector3d(-1 * radius * std::cos(0) + (l2+distance), 0, 0), 0));
	z.push_back(Waypoint(Eigen::Vector3d(radius * std::sin(0) + l1, 0, 0), 0));
	for(int i=0; i<(accuracy+1); i++) {
		double theta = M_PI * i / (accuracy / 2);
		x.push_back(Waypoint(Eigen::Vector3d(-1 * radius * std::cos(theta) + (l2+distance), 0, 0), time/accuracy));
		z.push_back(Waypoint(Eigen::Vector3d(radius * std::sin(theta) + l1, 0, 0), time/accuracy));
	}
	
	std::vector<minJerkCoordinates> circle_ret;
	circle_ret.push_back(minJerkCoordinates(minJerkSingle(x), minJerkSingle(y), minJerkSingle(z)));
	circle_ret.push_back(minJerkCoordinates(minJerkSingle(x), minJerkSingle(y), minJerkSingle(z)));
	// circle_ret.push_back(circle);

	return circle_ret;
}

// int main(void) {
// 	std::vector<minJerkCoordinates> v = testBehavior();
// 	for(double i=0.1; i<v[0].getTotalRunTime(); i+=.1) {
// 		std::cout << v[0].getPosition(i) << std::endl;
// 		std::cout << std::endl;
// 	}
// }