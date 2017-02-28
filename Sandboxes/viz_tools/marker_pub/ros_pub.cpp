#include "ros_pub.h"

ros_pub::~ros_pub(){
}


ros_pub* ros_pub::get_ros_pub(){
    static ros_pub gen_pub_;
    return &gen_pub_;
}

// ros_pub* ros_pub::get_ros_pub_for(ros::NodeHandle n){
//     static ros_pub gen_pub_;
//     node_handle_ = n;
//     return &gen_pub_;
// }


///////////////////
//    Position   //
///////////////////

void ros_pub::new_pos_channel(sejong::MarkerChannel & mc, std::string name, sejong::RvizColor color, 
                                int maxNumMrks, double minSleepTime, double size)
{
    if(!marker_pub_) // if publisher has not been created yet, then pointer is null
    {
        std::cout << "[ros_pub] Now advertising markers. Node handle refreshed. "<<name<<"\n";
        //ros::NodeHandle n;
        marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("viz_markers", 20);
    }

    mc.type_ = sejong::POS_MARKER;
    mc.minSleepTime_ = minSleepTime;
    mc.maxNumMrks_ = maxNumMrks;
    mc.firstMarker_ = true;
    mc.index_ = 0;

    mc.marker_.ns = name;
    mc.marker_.id = 0;
    mc.marker_.type = visualization_msgs::Marker::SPHERE_LIST;
    mc.marker_.action = visualization_msgs::Marker::ADD;
    
    ros::Time t = ros::Time::now();
    mc.marker_.header.stamp = t;
    mc.marker_.header.frame_id = "/world";
    mc.marker_.lifetime = ros::Duration();

    _setSphSize(mc.marker_, size);

    sejong::Quaternion identity(1.0, 0.0, 0.0, 0.0);
    _setOrientation(mc.marker_, identity );
    _setColor(mc.marker_ ,color);
}


void ros_pub::pub_pos_marker(sejong::MarkerChannel & mc, sejong::Vect3 & xyz)
{
    if(mc.type_ != sejong::POS_MARKER)
    {
        std::cout << "[ros_pub] Warning: Either your marker channel has not been initialized, or you didn't pass an position channel." << std::endl;
    }
    else
    {
    	ros::Time t = ros::Time::now();
        if( !mc.firstMarker_ ) // Before all points are created...
        {
            if(t.toSec() - mc.marker_.header.stamp.toSec() >= mc.minSleepTime_/1000.)
            {
                mc.marker_.header.stamp = t;

                _setPoint(mc.marker_.points[mc.index_],xyz);
			    
                marker_pub_.publish(mc.marker_);

                mc.index_ = (mc.index_ + 1) % mc.maxNumMrks_;
		        
                ros::spinOnce();
            }
        }
        else  // after all points are created ...
        {
            if(t.toSec() - mc.marker_.header.stamp.toSec() >= mc.minSleepTime_/1000.)
            {
                mc.marker_.header.stamp = t;
			   	

                geometry_msgs::Point p;
                _setPoint(p,xyz);
                mc.marker_.points.push_back(p);
			    
                marker_pub_.publish(mc.marker_);
		        
                if( mc.marker_.points.size() == mc.maxNumMrks_ )
                    mc.firstMarker_ = false;

                ros::spinOnce();
            }
        }
    }

}



///////////////////
//  Quaternions  //
///////////////////


void ros_pub::new_ori_channel(sejong::MarkerChannel & mc, std::string name, sejong::RvizColor color, 
                                 int maxNumMrks, double minSleepTime, double length)
{
    if(!marker_pub_) // if publisher has not been created yet, then pointer is null
    {
        std::cout << "[ros_pub] advertising markers. Node handle refreshed: "<<name<<"\n";
        //ros::NodeHandle n;
        marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("viz_markers", 20);
    }

    mc.type_ = sejong::ORI_MARKER;
    mc.minSleepTime_ = minSleepTime;
    mc.maxNumMrks_ = maxNumMrks;
    mc.firstMarker_ = true;
    mc.length_ = length;

    int32_t arr_shape = visualization_msgs::Marker::ARROW;
    int32_t sph_shape = visualization_msgs::Marker::SPHERE;

    mc.ori_marker_[0].ns = name + " - X_arr";
    mc.ori_marker_[1].ns = name + " - Y_arr";
    mc.ori_marker_[2].ns = name + " - Z_arr";
    mc.ori_marker_[3].ns = name + " - X_sph";
    mc.ori_marker_[4].ns = name + " - Y_sph";
    mc.ori_marker_[5].ns = name + " - Z_sph";

    ros::Time t = ros::Time::now();

    for(int i(0); i < 3; ++i)
    {
    	// Arrows (indicies: 0-2)
        mc.ori_marker_[i].id = 0;
    	mc.ori_marker_[i].type = arr_shape;
    	mc.ori_marker_[i].action = visualization_msgs::Marker::ADD;
        mc.ori_marker_[i].header.stamp = t;
        mc.ori_marker_[i].header.frame_id = "/world";
        mc.ori_marker_[i].lifetime = ros::Duration();
    	// _setArrSize(mc.ori_marker_[i],length);
    	_setArrSize(mc.ori_marker_[i],0.01);


    	// Spheres at arrowhead: (indices: 3-5)
        mc.ori_marker_[i+3].id = 0;
        mc.ori_marker_[i+3].type = sph_shape;
        mc.ori_marker_[i+3].action = visualization_msgs::Marker::ADD;
        mc.ori_marker_[i+3].header.stamp = t;
        mc.ori_marker_[i+3].header.frame_id = "/world";
        mc.ori_marker_[i+3].lifetime = ros::Duration();
    	_setSphSize(mc.ori_marker_[i+3], 0.01 );
    	_setColor(mc.ori_marker_[i+3] ,color);	// color

    }

    _setColor(mc.ori_marker_[0], sejong::RED);
    _setColor(mc.ori_marker_[1], sejong::LIME);
    _setColor(mc.ori_marker_[2], sejong::BLUE);

}



void ros_pub::pub_ori_marker(sejong::MarkerChannel & mc, sejong::Vect3 & xyz, sejong::Quaternion & quat)
{
    if(mc.type_ != sejong::ORI_MARKER)
    {
        std::cout << "[ros_pub] Warning: Either your marker channel has not been initialized, or you didn't pass an orientation channel." << std::endl;
    }
    else
    {
        ros::Time t = ros::Time::now();
        // std::cout << mc.firstMarker_ << std::endl;
        if(!(mc.firstMarker_))
        {
            if(t.toSec() - mc.ori_marker_[0].header.stamp.toSec() >= mc.minSleepTime_/1000.)
            {

                for(int i(0); i < 3; ++i)
                {
                    mc.ori_marker_[i  ].header.stamp = t;
                    mc.ori_marker_[i+3].header.stamp = t;

                    sejong::Vect3 tip = xyz + quat._transformVector( mc.length_*sejong::Vect3::Unit(i) );
					
                    geometry_msgs::Point point1;
                    geometry_msgs::Point point2;

                    _setPoint(point1, xyz);
                    _setPoint(point2, tip);

                    mc.ori_marker_[i].points[0] = point1;
                    mc.ori_marker_[i].points[1] = point2;

                    // mc.ori_marker_[i].points.push_back(point1);
                    // mc.ori_marker_[i].points.push_back(point2);                    

                    _setPosition(mc.ori_marker_[i+3], tip);

                    marker_pub_.publish(mc.ori_marker_[i  ]);
                    marker_pub_.publish(mc.ori_marker_[i+3]);

                    mc.ori_marker_[i  ].id = (mc.ori_marker_[i  ].id + 1) % mc.maxNumMrks_; //Only keep last 'maxNumMrks_' markers
                    mc.ori_marker_[i+3].id = (mc.ori_marker_[i+3].id + 1) % mc.maxNumMrks_; //Only keep last 'maxNumMrks_' markers
                }

                ros::spinOnce();
            }
        }
        else
        {
            for(int i(0); i < 3; ++i)
            {
                mc.ori_marker_[i  ].header.stamp = t;
                mc.ori_marker_[i+3].header.stamp = t;

                sejong::Vect3 tip = xyz + quat._transformVector( mc.length_*sejong::Vect3::Unit(i) );
					
                geometry_msgs::Point point1;
                geometry_msgs::Point point2;

                _setPoint(point1, xyz);
                _setPoint(point2, tip);

                mc.ori_marker_[i].points.push_back(point1);
                mc.ori_marker_[i].points.push_back(point2);

                std::cout << mc.ori_marker_[i].points.size() << std::endl;

                _setPosition(mc.ori_marker_[i+3], tip);

                marker_pub_.publish(mc.ori_marker_[i  ]);
                marker_pub_.publish(mc.ori_marker_[i+3]);

                mc.ori_marker_[i  ].id = (mc.ori_marker_[i  ].id + 1) % mc.maxNumMrks_; //Only keep last 'maxNumMrks_' markers
                mc.ori_marker_[i+3].id = (mc.ori_marker_[i+3].id + 1) % mc.maxNumMrks_; //Only keep last 'maxNumMrks_' markers
            }

            ros::spinOnce();

            mc.firstMarker_ = false;
        }
    }
}





///////////////////
//  One Arrow    //
///////////////////


void ros_pub::new_arr_channel(sejong::MarkerChannel & mc, std::string name, sejong::RvizColor color, 
                                 int maxNumMrks, double minSleepTime, double length, int numUpdatesPerSphere )
{
    if(!marker_pub_) // if publisher has not been created yet, then pointer is null
    {
        std::cout << "[ros_pub] advertising markers. Node handle refreshed: "<<name<<"\n";
        //ros::NodeHandle n;
        marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("viz_markers", 20);
    }

    mc.type_ = sejong::ARR_MARKER;
    mc.minSleepTime_ = minSleepTime;
    mc.maxNumMrks_ = std::max(1,maxNumMrks);
    mc.firstMarker_ = true;
    mc.length_ = length;
    mc.index_ = 0;
    mc.index2_ = 0;
    mc.numUpdatesPerSphere_ = numUpdatesPerSphere;


    // ********* SPHERE_LIST *********** //

    mc.marker_.ns = name + " - sphere trail";
    mc.marker_.id = 0;
    mc.marker_.type = visualization_msgs::Marker::SPHERE_LIST;
    mc.marker_.action = visualization_msgs::Marker::ADD;
    
    ros::Time t = ros::Time::now();
    mc.marker_.header.stamp = t;
    mc.marker_.header.frame_id = "/world";
    mc.marker_.lifetime = ros::Duration();

    _setSphSize(mc.marker_, 0.01);

    sejong::Quaternion identity(1.0, 0.0, 0.0, 0.0);
    _setOrientation(mc.marker_, identity );
    _setColor(mc.marker_ , color);


    // *********** ARROW ************** //

    mc.ori_marker_[0].ns = name + " - arrow";

    mc.ori_marker_[0].id = 0;
    mc.ori_marker_[0].type = visualization_msgs::Marker::ARROW;
    mc.ori_marker_[0].action = visualization_msgs::Marker::ADD;
    mc.ori_marker_[0].header.stamp = t;
    mc.ori_marker_[0].header.frame_id = "/world";
    mc.ori_marker_[0].lifetime = ros::Duration();
    _setArrSize(mc.ori_marker_[0],0.01);
    _setColor(mc.ori_marker_[0], color);

}



void ros_pub::pub_arr_marker(sejong::MarkerChannel & mc, sejong::Vect3 & xyz, sejong::Quaternion & quat)
{
    if(mc.type_ != sejong::ARR_MARKER)
    {
        std::cout << "[ros_pub] Warning: Either your marker channel has not been initialized, or you didn't pass an arrow channel." << std::endl;
    }
    else
    {
        ros::Time t = ros::Time::now();
        // std::cout << mc.firstMarker_ << std::endl;
        if(!(mc.firstMarker_))
        {
            if(t.toSec() - mc.ori_marker_[0].header.stamp.toSec() >= mc.minSleepTime_/1000.)
            {

                mc.ori_marker_[0].header.stamp = t;
                mc.    marker_   .header.stamp = t;

                sejong::Vect3 tip = xyz + quat._transformVector( mc.length_*sejong::Vect3::Unit(0) );
                
                geometry_msgs::Point point1;
                geometry_msgs::Point point2;

                _setPoint(point1, xyz);
                _setPoint(point2, tip);

                mc.ori_marker_[0].points[0] = point1;
                mc.ori_marker_[0].points[1] = point2;                


                if( mc.index2_ < mc.numUpdatesPerSphere_ ) {
                    mc.marker_.points[mc.index_] = point2;
                    mc.index2_++;
                }
                else {
                    if( mc.marker_.points.size() < mc.maxNumMrks_ ) {
                        mc.marker_.points.push_back(point2);
                        mc.index_++;
                    }
                    else {
                        mc.index_ = (mc.index_+1) % mc.maxNumMrks_;
                        mc.marker_.points[mc.index_] = point2;
                    }
                    mc.index2_ = 1;
                }

                marker_pub_.publish(mc.ori_marker_[0]);
                marker_pub_.publish(mc.    marker_   );

                ros::spinOnce();
            }
        }
        else
        {
            mc.ori_marker_[0].header.stamp = t;
            mc.    marker_   .header.stamp = t;

            sejong::Vect3 tip = xyz + quat._transformVector( mc.length_*sejong::Vect3::Unit(0) );
                
            geometry_msgs::Point point1;
            geometry_msgs::Point point2;

            _setPoint(point1, xyz);
            _setPoint(point2, tip);

            mc.ori_marker_[0].points.push_back(point1);
            mc.ori_marker_[0].points.push_back(point2);

            mc.marker_.points.push_back(point2);
            mc.index2_ = 1;

            marker_pub_.publish(mc.ori_marker_[0]);
            marker_pub_.publish(mc.    marker_   );

            ros::spinOnce();

            mc.firstMarker_ = false;
        }
    }
}


void ros_pub::set_arrow_length( sejong::MarkerChannel & mc , const double & length )
{
    if(mc.type_ != sejong::ARR_MARKER)
    {
        std::cout << "[ros_pub] Warning: Either your marker channel has not been initialized, or you didn't pass an arrow channel." << std::endl;
    }
    else
    {
        mc.length_ = length;
    }
}

void ros_pub::set_maxNumSpheres(sejong::MarkerChannel & mc , const int & maxNumMrks )
{
    if(mc.type_ != sejong::ARR_MARKER)
    {
        std::cout << "[ros_pub] Warning: Either your marker channel has not been initialized, or you didn't pass an arrow channel." << std::endl;
    }
    else
    {
        // mc.marker_.action = visualization_msgs::Marker::DELETE;
        // marker_pub_.publish(mc.marker_);
        mc.index_ = -1;
        mc.index2_ = mc.numUpdatesPerSphere_;
        mc.marker_.points.clear();

        //mc.marker_.action = visualization_msgs::Marker::ADD;
        mc.maxNumMrks_ = std::max(1,maxNumMrks);

             
    }
}





///////////////////
//  JointStates  //
///////////////////

void ros_pub::new_js_channel(sejong::JS_Channel & jsc, std::string name, double minSleepTime)
{
    // NOTE:  Currently "name" parameter does nothing 

    if(!js_pub_) // if publisher has not been created yet, then pointer is null
    {
        std::cout << "[ros_pub] advertising joint states. Node handle refreshed: "<<name<<"\n";
        //ros::NodeHandle n;
        //Sets the topic name for the advertising
        js_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states_2", 20);
    }


    ros::Time t = ros::Time::now();
    jsc.jsm_.header.stamp = t;
    jsc.minSleepTime_ = minSleepTime;
        

    static int TOT_NUM_JOINTS(59);
    jsc.jsm_.position.resize(TOT_NUM_JOINTS);


    for(int i(0); i++; i<TOT_NUM_JOINTS)
        jsc.jsm_.position[i] = 0.0;

    

    jsc.jsm_.name = {
            // Joints that WBOSC cares about in same order as R5_Definition Q:
            "leftHipYaw"               ,
            "leftHipRoll"              ,
            "leftHipPitch"             ,
            "leftKneePitch"            , 
            "leftAnklePitch"           ,
            "leftAnkleRoll"            ,
            "rightHipYaw"              ,
            "rightHipRoll"             ,
            "rightHipPitch"            ,
            "rightKneePitch"           , 
            "rightAnklePitch"          ,
            "rightAnkleRoll"           ,
            "torsoYaw"                 ,
            "torsoPitch"               ,
            "torsoRoll"                ,
            "leftShoulderPitch"        ,
            "leftShoulderRoll"         ,
            "leftShoulderYaw"          ,
            "leftElbowPitch"           ,
            "leftForearmYaw"           ,
            "leftWristRoll"            ,
            "leftWristPitch"           ,
            "lowerNeckPitch"           , 
            "neckYaw"                  ,
            "upperNeckPitch"           ,
            "rightShoulderPitch"       ,
            "rightShoulderRoll"        ,
            "rightShoulderYaw"         ,
            "rightElbowPitch"          ,
            "rightForearmYaw"          ,
            "rightWristRoll"           ,
            "rightWristPitch"          ,


            "hokuyo_joint"             ,

            // Right Hand Stuff            
            "rightThumbRoll"           ,
            "rightThumbPitch1"         ,
            "rightThumbPitch2"         ,
            "rightThumbPitch3"         ,
            "rightIndexFingerPitch1"   ,
            "rightIndexFingerPitch2"   ,
            "rightIndexFingerPitch3"   ,
            "rightMiddleFingerPitch1"  ,
            "rightMiddleFingerPitch2"  ,
            "rightMiddleFingerPitch3"  ,
            "rightPinkyPitch1"         ,
            "rightPinkyPitch2"         ,
            "rightPinkyPitch3"         ,
            
            // Left Hand Stuff
            "leftThumbRoll"            ,
            "leftThumbPitch1"          ,
            "leftThumbPitch2"          ,
            "leftThumbPitch3"          ,
            "leftIndexFingerPitch1"    ,
            "leftIndexFingerPitch2"    ,
            "leftIndexFingerPitch3"    ,
            "leftMiddleFingerPitch1"   ,
            "leftMiddleFingerPitch2"   ,
            "leftMiddleFingerPitch3"   ,
            "leftPinkyPitch1"          ,
            "leftPinkyPitch2"          ,
            "leftPinkyPitch3"          
        };

}

void ros_pub::pub_js(sejong::JS_Channel & jsc, sejong::Vector & Q)
{
    ros::Time t = ros::Time::now();

    if(t.toSec() - jsc.jsm_.header.stamp.toSec() >= jsc.minSleepTime_/1000.)
    {

        /*  ******************** Space Robotics Challenge Stuff ***********************
        if( Q.size() != NUM_Q)
            std::cout << "ros_pub::pub_js() ERROR: Q.size() must be equal to NUM_Q" << std::endl;
        else
        {
            jsc.jsm_.header.stamp = t;
            
            // Match joints in Q with joints in jsc.jsm_.name[]
            for(int i(NUM_VIRTUAL); i < NUM_Q-1; i++ )
                jsc.jsm_.position[i-NUM_VIRTUAL] = Q[i];

            // Publish JointState Message:
            js_pub_.publish(jsc.jsm_);

            // Publish Pelvis transform
            static tf::TransformBroadcaster br;
            tf::Transform transform;

            transform.setOrigin( tf::Vector3(   Q[VIRTUAL_X], 
                                                Q[VIRTUAL_Y], 
                                                Q[VIRTUAL_Z] ) );
            
            transform.setRotation(tf::Quaternion(   Q[VIRTUAL_Rx],
                                                    Q[VIRTUAL_Ry],
                                                    Q[VIRTUAL_Rz],
                                                    Q[NUM_Q-1]       ) );
            
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ghost/pelvis"));

        }*/
    }
}




///////////////////////
//  Private Methods  //
///////////////////////

void ros_pub::_setPosition(visualization_msgs::Marker & m, sejong::Vect3 & xyz)
{
    m.pose.position.x = xyz[0];
    m.pose.position.y = xyz[1];
    m.pose.position.z = xyz[2];
}
void ros_pub::_setOrientation(visualization_msgs::Marker & m, sejong::Quaternion quat)
{
    m.pose.orientation.x = quat.x();
    m.pose.orientation.y = quat.y();
    m.pose.orientation.z = quat.z();
    m.pose.orientation.w = quat.w();
}
void ros_pub::_setPoint(geometry_msgs::Point & p, sejong::Vect3 & xyz)
{
    p.x = xyz[0];
    p.y = xyz[1];
    p.z = xyz[2];
}

void ros_pub::_setSphSize(visualization_msgs::Marker & m, double size)
{
    m.scale.x = size;
    m.scale.y = size;
    m.scale.z = size;
}
void ros_pub::_setArrSize(visualization_msgs::Marker & m, double diameter)
{
    // m.scale.x = 0.25;  // shaft diameter
    // m.scale.y = 0.02;  // head diameter
    // m.scale.z = 0.02;    // head length (automatic if set to 0)    

    m.scale.x = diameter;  // shaft diameter
    m.scale.y = diameter*2;  // head diameter
    m.scale.z = 0.05;    // head length (automatic if set to 0)
}

void ros_pub::_setColor(visualization_msgs::Marker & m, sejong::RvizColor color)
{
    sejong::Vect3 rgb;

    switch(color)
    {
		
    case sejong::BLACK: 	rgb << 0.,0.,0.; break;   //  #000000     (0,0,0)
    case sejong::WHITE:    	rgb << 1.,1.,1.; break;   //  #FFFFFF     (255,255,255)
    case sejong::RED:		rgb << 1.,0.,0.; break;   //  #FF0000     (255,0,0)
    case sejong::LIME:   	rgb << 0.,1.,0.; break;   //  #00FF00     (0,255,0)
    case sejong::BLUE:     	rgb << 0.,0.,1.; break;   //  #0000FF     (0,0,255)
    case sejong::YELLOW:	rgb << 1.,1.,0.; break;   //  #FFFF00     (255,255,0)
    case sejong::CYAN:		rgb << 0.,1.,1.; break;   //  #00FFFF     (0,255,255)
    case sejong::MAGENTA:  	rgb << 1.,0.,1.; break;   //  #FF00FF     (255,0,255)  	
    default:		        rgb << 1.,1.,1.; break;   //  #FFFFFF     (255,255,255)
    }

    m.color.r = rgb[0];
    m.color.g = rgb[1];
    m.color.b = rgb[2];
    m.color.a = 1.0;

}



ros_pub::ros_pub()
{
}














/////////////////
//   Not Used  //
/////////////////

// Set arrow size for position/orientation option instead of point option
// void ros_pub::_setArrSize(visualization_msgs::Marker & m, double length)
// {
//     m.scale.x = length;  // length
//     m.scale.y = 0.01; 	 // width
//     m.scale.z = 0.01;    // height
// }

// Set orientation vectors via position and orientation instead of points
// void ros_pub::pub_ori_marker(sejong::MarkerChannel & mc, sejong::Vect3 & xyz, sejong::Quaternion & quat)
// {
// 	Eigen::Matrix3d R(quat);
// 	ros::Time t = ros::Time::now();
// 	// std::cout << mc.firstMarker_ << std::endl;
//     if(!mc.firstMarker_)
//     {
//     	if(t.toSec() - mc.ori_marker_[0].header.stamp.toSec() >= mc.minSleepTime_/1000.)
//     	{

//     		sejong::Quaternion quatY (cos( M_PI/2.0), 0.0, sin(M_PI/2.0), 0.0           );
//     		sejong::Quaternion quatZ (cos(-M_PI/2.0), 0.0, 0.0          , sin(-M_PI/2.0));

//     		_setOrientation(mc.ori_marker_[0],quat);
//     		_setOrientation(mc.ori_marker_[1],sejong::QuatMultiply(quat,quatZ));
//     		_setOrientation(mc.ori_marker_[2],sejong::QuatMultiply(quat,quatY));

// 			for(int i(0); i < 3; ++i)
// 			{
// 				mc.ori_marker_[i  ].header.stamp = t;
// 			    mc.ori_marker_[i+3].header.stamp = t;

// 				sejong::Vect3 tip = xyz + quat._transformVector( mc.length_*sejong::Vect3::Unit(i) );
			

//  		   		_setPosition(mc.ori_marker_[i],   xyz);
// 				_setPosition(mc.ori_marker_[i+3], tip);

// 				marker_pub_.publish(mc.ori_marker_[i  ]);
// 		        marker_pub_.publish(mc.ori_marker_[i+3]);

// 		        mc.ori_marker_[i  ].id = (mc.ori_marker_[i  ].id + 1) % mc.maxNumMrks_; //Only keep last 'maxNumMrks_' markers
// 		        mc.ori_marker_[i+3].id = (mc.ori_marker_[i+3].id + 1) % mc.maxNumMrks_; //Only keep last 'maxNumMrks_' markers
// 			}

// 	        ros::spinOnce();
// 		}
// 	}
// 	else
// 	{
// 			sejong::Quaternion quatY (cos( M_PI/2.0), 0.0, sin(M_PI/2.0), 0.0           );
//     		sejong::Quaternion quatZ (cos(-M_PI/2.0), 0.0, 0.0          , sin(-M_PI/2.0));

//     		_setOrientation(mc.ori_marker_[0],quat);
//     		_setOrientation(mc.ori_marker_[1],sejong::QuatMultiply(quat,quatZ));
//     		_setOrientation(mc.ori_marker_[2],sejong::QuatMultiply(quat,quatY));

// 			for(int i(0); i < 3; ++i)
// 			{
// 				mc.ori_marker_[i  ].header.stamp = t;
// 			    mc.ori_marker_[i+3].header.stamp = t;

// 				sejong::Vect3 tip = xyz + quat._transformVector( mc.length_*sejong::Vect3::Unit(i) );
			

//  		   		_setPosition(mc.ori_marker_[i],   xyz);
// 				_setPosition(mc.ori_marker_[i+3], tip);

// 				marker_pub_.publish(mc.ori_marker_[i  ]);
// 		        marker_pub_.publish(mc.ori_marker_[i+3]);

// 		        mc.ori_marker_[i  ].id = (mc.ori_marker_[i  ].id + 1) % mc.maxNumMrks_; //Only keep last 'maxNumMrks_' markers
// 		        mc.ori_marker_[i+3].id = (mc.ori_marker_[i+3].id + 1) % mc.maxNumMrks_; //Only keep last 'maxNumMrks_' markers
// 			}

//         	ros::spinOnce();

// 	    mc.firstMarker_ = false;
// 	}
// }




/////////////////////////////////////////////
/////////////// Joint stuff: ////////////////
/////////////////////////////////////////////

        // jsc.jsm_.name = {
        //     "hokuyo_joint"             ,
        //     "torsoYaw"                 ,
        //     "torsoPitch"               ,
        //     "torsoRoll"                ,
        //     "lowerNeckPitch"           , 
        //     "neckYaw"                  ,
        //     "upperNeckPitch"           ,
        //     "rightShoulderPitch"       ,
        //     "rightShoulderRoll"        ,
        //     "rightShoulderYaw"         ,
        //     "rightElbowPitch"          ,
        //     "rightForearmYaw"          ,
        //     "rightWristRoll"           ,
        //     "rightWristPitch"          ,
        //     "rightThumbRoll"           ,//->
        //     "rightThumbPitch1"         ,
        //     "rightThumbPitch2"         ,
        //     "rightThumbPitch3"         ,
        //     "rightIndexFingerPitch1"   ,
        //     "rightIndexFingerPitch2"   ,
        //     "rightIndexFingerPitch3"   ,
        //     "rightMiddleFingerPitch1"  ,
        //     "rightMiddleFingerPitch2"  ,
        //     "rightMiddleFingerPitch3"  ,
        //     "rightPinkyPitch1"         ,
        //     "rightPinkyPitch2"         ,
        //     "rightPinkyPitch3"         ,//<-
        //     "leftShoulderPitch"        ,
        //     "leftShoulderRoll"         ,
        //     "leftShoulderYaw"          ,
        //     "leftElbowPitch"           ,
        //     "leftForearmYaw"           ,
        //     "leftWristRoll"            ,
        //     "leftWristPitch"           ,
        //     "leftThumbRoll"            ,//->
        //     "leftThumbPitch1"          ,
        //     "leftThumbPitch2"          ,
        //     "leftThumbPitch3"          ,
        //     "leftIndexFingerPitch1"    ,
        //     "leftIndexFingerPitch2"    ,
        //     "leftIndexFingerPitch3"    ,
        //     "leftMiddleFingerPitch1"   ,
        //     "leftMiddleFingerPitch2"   ,
        //     "leftMiddleFingerPitch3"   ,
        //     "leftPinkyPitch1"          ,
        //     "leftPinkyPitch2"          ,
        //     "leftPinkyPitch3"          ,//<-
        //     "rightHipYaw"              ,
        //     "rightHipRoll"             ,
        //     "rightHipPitch"            ,
        //     "rightKneePitch"           , 
        //     "rightAnklePitch"          ,
        //     "rightAnkleRoll"           ,
        //     "leftHipYaw"               ,
        //     "leftHipRoll"              ,
        //     "leftHipPitch"             ,
        //     "leftKneePitch"            , 
        //     "leftAnklePitch"           ,
        //     "leftAnkleRoll"            };