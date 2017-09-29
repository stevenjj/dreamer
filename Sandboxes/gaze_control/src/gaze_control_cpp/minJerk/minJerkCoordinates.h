#ifndef minJerkCoordinates_h
#define minJerkCoordinates_h

#include "minJerkSingle.h"
#include <vector>
class minJerkCoordinates {
public:
	minJerkSingle x;
	minJerkSingle y;
	minJerkSingle z;
	char sys;

	minJerkCoordinates(minJerkSingle, minJerkSingle, minJerkSingle);
	minJerkCoordinates(minJerkSingle, minJerkSingle, minJerkSingle, char);
	minJerkCoordinates();
	~minJerkCoordinates();

	Eigen::Vector3d getPosition(double);
	double getTotalRunTime(void);

	std::vector<minJerkCoordinates> testBehavior(void);
};


std::vector<minJerkCoordinates> testBehavior(void);

#endif
