#ifndef minJerkSingle_h
#define minJerkSingle_h

#include <Eigen/Dense>
#include "../Waypoint.h"
#include <vector>

class minJerkSingle {
public:
	std::vector<Waypoint> waypointList;
	std::vector< Eigen::RowVectorXd > coeffs;
	std::vector< Eigen::RowVectorXd > specialCoeffs;

	minJerkSingle(std::vector<Waypoint>);
	minJerkSingle(void);
	~minJerkSingle();

	Eigen::RowVectorXd minJerkTwoPoints(const Waypoint&, const Waypoint&);
	void getAllMinJerk(void);
	double getPosition(double);
	double getTotalRunTime(void);

};

#endif
