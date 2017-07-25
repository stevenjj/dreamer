#include <Eigen/Dense>
#include <iostream>
#include <ctime>

#include <cmath>
#include "modernRobotics.h"
#include "headKinematics.h"
#include "utilQuat.h"

int main(void){
	double time = 0;
	headKinematics hk;
    for(int i = 0; i < 1000; i++) {
		std::clock_t    start;
    	start = std::clock();
		hk.get6D_HeadJacobian(hk.Jlist);
		time+=(std::clock() - start) / (double)(CLOCKS_PER_SEC/1000);
	}
    std::cout << "Time: " << (time/1000) << " ms" << std::endl;
	// std::cout << hk.get6D_LeftEyePosition(hk.Jlist)[1] << std::endl;

	return -1;
}
/*
       0        0       -1        0        0        0        0
      -1        0        0       -1       -1        0        0
       0        1        0        0        0        1        0
       0        0        0  0.13849  0.13849   -0.053        0
       0        0 -0.13849        0        0 -0.12508        0
       0        0        0        0 -0.12508        0        0
         0  -0.909297  -0.272012   0.556469   0.556469  -0.314125          0
        -1          0   0.756802   0.627609   0.627609   -0.44775          0
         0  -0.416147   0.594356  -0.544472  -0.544472  -0.837165          0
         0          0  0.0436162  0.0361705  0.0555438  -0.124974          0
         0          0  0.0905231  -0.100635  -0.190965  -0.119433          0
         0          0 -0.0953031 -0.0790339  -0.163356   0.110771          0
*/