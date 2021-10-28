/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/robot_qt/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_qt {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"robot_qt");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	odom_sub = n.subscribe("odom", 1000, &QNode::vel_sub_callback, this);
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	// power_sub = n.subscribe("power_information", 1000, &QNode::power_sub_callback, this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"robot_qt");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	start();
	return true;
}



void QNode::run() {
	ros::Rate loop_rate(1);
	
	// send a successfull message
	std_msgs::String msg;
	std::stringstream ss;
	ss << "Connect successfully!" ;
	msg.data = ss.str();
	chatter_publisher.publish(msg);
	log(Info,msg.data);
	
	while ( ros::ok() ) 
	{
		// write
	}
	
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

// vel_sub callback

void QNode::vel_sub_callback(const nav_msgs::Odometry& msg)
{
	emit vel_signal(msg.twist.twist.linear.x, msg.twist.twist.angular.z);
}

// set cmd_vel
void QNode::set_cmd_vel(char k)
{
	// vector {linear_vel, raw_vel}
	std::map<char, std::vector<float>> move_blindings
	{
		{'F', {1, 0}},
		{'L', {0, 1}},
		{'R', {0, -1}},
		{'B', {-1, 0}},
		{'S', {0, 0}}
	};
	int linear = move_blindings[k][0];
	int raw = move_blindings[k][1];

	float speed_control = 0.5;

	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x = linear * speed_control;
	vel_msg.angular.z = raw * speed_control;
	cmd_vel_pub.publish(vel_msg);

}

// power_sub_callback
/*
void QNode::power_sub_callback(const tcp::PowerInformation& msg)
{
	emit power_signal(msg.percentage, msg.voltage);	
} 

*/


}  // namespace robot_qt
