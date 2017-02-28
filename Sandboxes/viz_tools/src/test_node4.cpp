#include <stdio.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

void tfCallback(const tf2_msgs::TFMessage& msg){
}


int main(int argc, char** argv){
	ros::init(argc, argv, "marker_publisher");	

	//ros::Subscriber sub = n.subscribe("/tf", 10, &tfCallback);

	ros::spin();
	return 0;
};