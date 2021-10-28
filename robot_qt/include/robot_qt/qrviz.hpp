#ifndef QRVIZ_HPP
#define QRVIZ_HPP

#include <QObject>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <ros/ros.h>
#include <QVBoxLayout>


class qrviz
{
public: 
    qrviz(QVBoxLayout* layout);
    void set_goal_pose();
    void set_init_pose();

private:
    rviz::RenderPanel* render_panel;
    rviz::VisualizationManager* manager = NULL;
    rviz::Display* grid = NULL;
    rviz::Display* tf = NULL;
    rviz::Display* laser_scan = NULL;
    rviz::Display* map = NULL;
    rviz::Display* path = NULL;
    rviz::Display* robot_model = NULL;
    rviz::ToolManager* tool_manager = NULL;
};

#endif