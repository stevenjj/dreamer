#include <stdio.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>			// desired pose in reference frame
#include <geometry_msgs/Twist.h>		// desired twist in reference frame
#include <gazebo_msgs/GetModelState.h>

/* GetModelState Service Description:
string model_name                    # name of Gazebo Model
string relative_entity_name          # return pose and twist relative to this entity
                                     # an entity can be a model, body, or geom
                                     # be sure to use gazebo notation (e.g. [model_name::body_name])
                                     # leave empty or "world" will use inertial world frame
---
geometry_msgs/Pose pose              # pose of model in relative entity frame
geometry_msgs/Twist twist            # twist of model in relative entity frame
bool success                         # return true if get successful
string status_message                # comments if available
*/


// gazebo_msgs::GetModelState getmodelstate;
// ros::ServiceClient client;


void tfCallback(const tf2_msgs::TFMessage& msg){

	// if (!client.call(getmodelstate))
	// {
	// 	ROS_ERROR("Failed to call service /gazebo/get_model_state");
	// 	std::cout << "model_name =           " << getmodelstate.request.model_name << std::endl;
	// 	std::cout << "relative_entity_name = " << getmodelstate.request.relative_entity_name << std::endl;

	// 	return;
	// }

	// static tf::TransformBroadcaster br;
	// tf::Transform transform;
	// transform.setOrigin( tf::Vector3(	getmodelstate.response.pose.position.x,
	// 									getmodelstate.response.pose.position.y,
	// 									getmodelstate.response.pose.position.z  ) );
	// tf::Quaternion q(	getmodelstate.response.pose.orientation.x,
	// 					getmodelstate.response.pose.orientation.y,
	// 					getmodelstate.response.pose.orientation.z,
	// 					getmodelstate.response.pose.orientation.w  );	
	// transform.setRotation(q);
	
	// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "pelvis"));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "marker_publisher");
	// if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
	// turtle_name = argv[1];

	ros::NodeHandle n;

	// client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	// getmodelstate.request.model_name = "valkyrie";
	// getmodelstate.request.relative_entity_name = "world";
	


	ros::Subscriber sub = n.subscribe("/tf", 10, &tfCallback);

	ros::spin();
	return 0;
};