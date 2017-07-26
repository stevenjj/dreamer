#include <Eigen/Dense>
#include <iostream>
#include <ctime>

#include <cmath>
#include "modernRobotics.h"
#include "headKinematics.h"
#include "utilQuat.h"

int main(void){
	headKinematics hk;
	// std::cout << hk.Jlist << std::endl;
	// std::cout << hk.Jlist.head(3) << std::endl;
	Eigen::MatrixXd *point1 = hk.get6D_LeftEyePosition(hk.Jlist);
	Eigen::MatrixXd *point2 = hk.get6D_RightEyePosition(hk.Jlist);
	Eigen::MatrixXd *point3 = hk.get6D_HeadPosition(hk.Jlist);
	std::cout << point1[1] << std::endl;
	std::cout << point2[1] << std::endl;
	std::cout << point3[1] << std::endl;
	delete [] point1;
	delete [] point2;
	delete [] point3;



	return -1;


	// Eigen::MatrixXd se3mat(4,4);
	// se3mat << 0,                 0,                  0,                 0,
 //          0,                 0, -1.570796326794897, 2.356194490192345,
 //          0, 1.570796326794897,                  0, 2.356194490192345,
 //          0,                 0,                  0,                 0;

 //    std::cout << MatrixExp6(se3mat) << std::endl;


	
	// double time = 0;
 //    for(int i = 0; i < 1000; i++) {
	// 	std::clock_t    start;
 //    	start = std::clock();
	// 	hk.get6D_HeadJacobian(a);
	// 	time+=(std::clock() - start) / (double)(CLOCKS_PER_SEC/1000);
	// }
 //    std::cout << "Time: " << (time/1000) << " ms" << std::endl;

}
