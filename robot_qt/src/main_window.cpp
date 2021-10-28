/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/robot_qt/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_qt {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }


	// speed_board init
	speed_dash_board = new DashBoard(ui.vel_board_widget);
	speed_dash_board->setGeometry(ui.vel_board_widget->rect());
	speed_dash_board->set_speed(0);

	// connect vel update

	connect(&qnode, SIGNAL(vel_signal(float, float)), this, SLOT(slot_vel_update(float, float)));
	
	// connect cmd_vel_btn
	connect(ui.forward_btn, SIGNAL(clicked()), this, SLOT(slot_cmd_vel_btn()));
	connect(ui.back_btn, SIGNAL(clicked()), this, SLOT(slot_cmd_vel_btn()));
	connect(ui.stop_btn, SIGNAL(clicked()), this, SLOT(slot_cmd_vel_btn()));
	connect(ui.left_btn, SIGNAL(clicked()), this, SLOT(slot_cmd_vel_btn()));
	connect(ui.right_btn, SIGNAL(clicked()), this, SLOT(slot_cmd_vel_btn()));

	// connect nav_btn
	connect(ui.pose_estimate_btn, SIGNAL(clicked()), this, SLOT(slot_set_init_pose()));
	connect(ui.nav_goal_btn, SIGNAL(clicked()), this, SLOT(slot_set_goal_pose()));

	// connect power information
	// connect(&qnode, SIGNAL(power_signal(int, float)), this, SLOT(slot_power_information(int, float)));
}



MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			// rviz widget init
			my_rviz = new qrviz(ui.rviz_layout);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
			// rviz widget init
			my_rviz = new qrviz(ui.rviz_layout);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "robot_qt");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "robot_qt");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

/*****************************************************************************
** my_function
*****************************************************************************/

float MainWindow::abs(float vel)
{
	if (vel >= 0)
		return vel;
	else
		return -vel;
}


/*****************************************************************************
** my_slot
*****************************************************************************/

void MainWindow::slot_cmd_vel_btn()
{
	QPushButton* sender_btn = qobject_cast<QPushButton*>(sender());
	char k = sender_btn->text().toStdString()[0];
	qnode.set_cmd_vel(k);
}

void MainWindow::slot_vel_update(float linear, float raw)
{
	if (abs(linear)*100 < 0.1)	
		ui.linear_velocity_value->setText(QString::number(0));
	else
		ui.linear_velocity_value->setText(QString::number(linear*100));
	if (abs(raw)*100 < 0.01)
		ui.raw_velocity_value->setText(QString::number(0));
	else	
		ui.raw_velocity_value->setText(QString::number(raw*180/3.14));

	speed_dash_board->set_speed(abs(linear*100));
}

void MainWindow::slot_set_init_pose()
{
	my_rviz->set_init_pose();
}

void MainWindow::slot_set_goal_pose()
{
	my_rviz->set_goal_pose();
}

/*
void MainWindow::slot_power_information(int per, float vol)
{
	ui.progressBar_battery->setValue(per);
}
*/

}  // namespace robot_qt

