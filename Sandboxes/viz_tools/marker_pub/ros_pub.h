#ifndef ROS_PUB_UTIL
#define ROS_PUB_UTIL

//#include <ControlSystem/R5_Controller/R5_Definition.h>

#include <string>
#include <utils/wrap_eigen.hpp>

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>


namespace sejong {

	enum Marker_Channel_Type{
	    POS_MARKER   ,
	    ORI_MARKER   ,
	    ARR_MARKER   ,
	    MESH_MARKER
	};

	enum RvizColor{
	    BLACK    ,
	    WHITE    ,
	    RED      ,
	    LIME     ,
	    BLUE     ,
	    YELLOW   ,
	    CYAN     ,
	    MAGENTA  
	};

	class MarkerChannel{
		public: 
			Marker_Channel_Type type_;
			int maxNumMrks_;
			int numUpdatesPerSphere_;
			int index_;
			int index2_;
			double minSleepTime_; // milliseconds
			visualization_msgs::Marker marker_;
			visualization_msgs::Marker ori_marker_[6]; //0-2 are arrows, 3-5 are spheres
			double length_;
			bool firstMarker_;

	};

	class JS_Channel{
		public:
			std::string tf_prefix_;
			sensor_msgs::JointState jsm_;
			double minSleepTime_; // milliseconds
	};
}


class ros_pub{
public:

	static ros_pub* get_ros_pub();
	//static ros_pub* get_ros_pub_for(ros::NodeHandle n);
	virtual ~ros_pub();

	// Position Markers
	void new_pos_channel(sejong::MarkerChannel & mc, std::string name, sejong::RvizColor color, 
				int maxNumMrks=1, double minSleepTime=1000./30., double size=0.01 );
	void pub_pos_marker(sejong::MarkerChannel & mc, sejong::Vect3 & xyz);

	// Orientation Markers
	void new_ori_channel(sejong::MarkerChannel & mc, std::string name, sejong::RvizColor color,
				int maxNumMrks=1, double minSleepTime=1000./30., double length=0.25 );
	void pub_ori_marker(sejong::MarkerChannel & mc, sejong::Vect3 & xyz, sejong::Quaternion & quat);
	

	// Direction Markers
	void new_arr_channel(sejong::MarkerChannel & mc, std::string name, sejong::RvizColor color,
				int maxNumMrks=1, double minSleepTime=1000./30., double length=0.25, int numUpdatesPerSphere=1 );
	void pub_arr_marker(sejong::MarkerChannel & mc, sejong::Vect3 & xyz, sejong::Quaternion & quat);
	void set_arrowLength(  sejong::MarkerChannel & mc , const double & length );
	void set_maxNumSpheres(sejong::MarkerChannel & mc , const int & maxNumMrks );
	void set_numUpdatesPerSphere(sejong::MarkerChannel & mc , const int & numUpdatesPerSphere );


	void new_js_channel(sejong::JS_Channel & jsc, std::string name, double minSleepTime=1000./30.);
	void pub_js(sejong::JS_Channel & jsc, sejong::Vector & Q);

private:
	ros_pub();

	void _setPosition(visualization_msgs::Marker & m, sejong::Vect3 & xyz);
	void _setOrientation(visualization_msgs::Marker & m, sejong::Quaternion quat);
	void _setPoint(geometry_msgs::Point & p, sejong::Vect3 & xyz);
	void _setColor(visualization_msgs::Marker & m, sejong::RvizColor color);
	void _setSphSize(visualization_msgs::Marker & m, double size);
	void _setArrSize(visualization_msgs::Marker & m, double diameter);

	// int marker_id_counter_=0;
	ros::NodeHandle node_handle_;
	ros::Publisher marker_pub_;
	ros::Publisher js_pub_;
};



#endif
