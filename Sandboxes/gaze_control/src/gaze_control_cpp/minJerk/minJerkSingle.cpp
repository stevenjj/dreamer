#include "minJerkSingle.h"
#include <Eigen/Dense>
#include <vector>
#include <cmath>
// #include <iostream> // Debug


minJerkSingle::minJerkSingle(std::vector<Waypoint> list) {
    waypointList = list;
    
    for(unsigned int i=1; i<waypointList.size(); i++) {
    	waypointList[i].Dt = waypointList[i-1].Dt + waypointList[i].Dt; 
    }
    
    getAllMinJerk();
}

minJerkSingle::~minJerkSingle() {}

/*
 * Function: Calculates the minJerk coefficients between two points
 * Inputs: Initial Waypoint, Final Waypoint
 * Returns: Eigen::VectorXd of the 6 coefficient values of the polynomial function
 */
Eigen::RowVectorXd minJerkSingle::minJerkTwoPoints(const Waypoint& wInit, const Waypoint& wFin) {
    double to = wInit.Dt;
    double tf = wFin.Dt;
    Eigen::MatrixXd TMatrixCoeffs(6,6);
    TMatrixCoeffs << 1, to, std::pow(to,2),   std::pow(to,3),    std::pow(to,4), std::pow(to,5), 
                     0,  1,           2*to, 3*std::pow(to,2),  4*std::pow(to,3), 5*std::pow(to,4),
                     0,  0,              2,             6*to, 12*std::pow(to,2), 20*std::pow(to,3),
                     1, tf, std::pow(tf,2),   std::pow(tf,3),    std::pow(tf,4), std::pow(tf,5),
                     0,  1,           2*tf, 3*std::pow(tf,2),  4*std::pow(tf,3), 5*std::pow(tf,4),
                     0,  0,              2,             6*tf, 12*std::pow(tf,2), 20*std::pow(tf,2);
    Eigen::RowVectorXd boundPoints(6);
    boundPoints << wInit.point(0), wInit.point(1), wInit.point(2), wFin.point(0), wFin.point(1), wFin.point(2);
    Eigen::JacobiSVD<Eigen::MatrixXd> TMatrixCoeffs_Bar(TMatrixCoeffs, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return TMatrixCoeffs_Bar.solve(boundPoints.transpose());
}

/*
 * Function: Populates coeffs vector with minJerk coefficient values between all points
 * Inputs: None
 * Returns: None: updates class variable
 */
void minJerkSingle::getAllMinJerk(void) { 
	for(unsigned int i=1; i < waypointList.size(); i++) {
		coeffs.push_back(minJerkTwoPoints(waypointList[i-1], waypointList[i]));
	}
}


/*
 * Function: Calculates the position in a minJerk function
 * Inputs: time
 * Returns: position @ input time
 */
double minJerkSingle::getPosition(double time) {
	unsigned int listLength = waypointList.size();
	if(time < 0) {
		std::cerr << "Invalid time input to getPosition" << std::endl;
	}
	else if(time > waypointList[listLength-1].Dt) {
		time = waypointList[listLength-1].Dt;
	}

	double pos_ret = 1337;
	for(unsigned int i=1; i<listLength; i++) {
		if((waypointList[i-1].Dt <= time) && (time <= waypointList[i].Dt)) {
			pos_ret = coeffs[i-1][0] +
							 coeffs[i-1][1] * time +
							 coeffs[i-1][2] * std::pow(time, 2) +
							 coeffs[i-1][3] * std::pow(time, 3) +
							 coeffs[i-1][4] * std::pow(time, 4) +
							 coeffs[i-1][5] * std::pow(time, 5);
		}
	}
	return pos_ret;

}


// int main(void) {
//     Eigen::Vector3d c1(0,0,0);
//     Eigen::Vector3d c2(1,0,0);
//     Waypoint w1(c1, 0);
//     Waypoint w2(c2, 1);
//     std::vector<Waypoint> wList = {w1, w2};
//     minJerkSingle s1(wList);
//     s1.getPosition(.6);
//     std::cout << s1.getPosition(.6) << std::endl;
// }

