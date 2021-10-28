/**
 * @file /include/robot_qt/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robot_qt_QNODE_HPP_
#define robot_qt_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_qt {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
	
	// sub callback
	void vel_sub_callback(const nav_msgs::Odometry& msg);
	// void power_sub_callback(const tcp_server::PowerInformation& msg);

	// set cmd_vel
	void set_cmd_vel(char k);

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
	// vel_sub signal with mainwindow
	void vel_signal(float, float);

	// power_signal, int percentage, float voltage
	// void power_signal(int, float); 

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;
	ros::Subscriber odom_sub;
	ros::Publisher cmd_vel_pub;
	ros::Subscriber power_sub;
};

}  // namespace robot_qt

#endif /* robot_qt_QNODE_HPP_ */
