/**
 * @file /include/robot_qt/main_window.hpp
 *
 * @brief Qt based gui for robot_qt.
 *
 * @date November 2010
 **/
#ifndef robot_qt_MAIN_WINDOW_H
#define robot_qt_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "dashboard.hpp"
#include "qrviz.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace robot_qt {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
	void slot_vel_update(float, float);

	void slot_cmd_vel_btn();

	// nav_btn_slot
	void slot_set_init_pose();
	void slot_set_goal_pose();

	// power information slot
	// void slot_power_information(int, float);

	// abs for vel_value
	float abs(float vel);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	DashBoard* speed_dash_board;
	qrviz* my_rviz;
};

}  // namespace robot_qt

#endif // robot_qt_MAIN_WINDOW_H
