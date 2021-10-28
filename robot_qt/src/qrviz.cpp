#include "../include/robot_qt/qrviz.hpp"

qrviz::qrviz(QVBoxLayout* layout)
{
    render_panel = new rviz::RenderPanel();
    layout->addWidget(render_panel);

    manager = new rviz::VisualizationManager(render_panel);
    ROS_ASSERT(manager != NULL);
    
    // render_pannel init
    render_panel->initialize(manager->getSceneManager(), manager);
    manager->initialize();
    manager->startUpdate();
    manager->removeAllDisplays();
    manager->setFixedFrame("map");

    // grid init
    grid = manager->createDisplay("rviz/Grid", "my_grid", true);
    grid->subProp("PlaneCellCount")->setValue(1);
    ROS_ASSERT(grid != NULL);

    // tf
    tf = manager->createDisplay("rviz/TF", "my_tf", true);
    ROS_ASSERT(tf != NULL);

    // laser_scan
    laser_scan = manager->createDisplay("rviz/LaserScan", "my_laserscan", true);
    ROS_ASSERT(laser_scan != NULL);

    // map
    map = manager->createDisplay("rviz/Map", "my_map", true);
    map->subProp("Topic")->setValue("/map");
    ROS_ASSERT(map != NULL);

    // robot_model
    robot_model = manager->createDisplay("rviz/RobotModel", "my_robot_model", true);
    ROS_ASSERT(robot_model != NULL);

    // path
    // robot k201 topic name: /movebase/GlobalPlanner/plan
    path = manager->createDisplay("rviz/Path", "my_path", true);
    path->subProp("Topic")->setValue("/move_base/NavfnROS/plan");
    ROS_ASSERT(path != NULL);

    // tool_manager
    tool_manager = manager->getToolManager();

}

void qrviz::set_goal_pose()
{
    rviz::Tool* current_tool = tool_manager->addTool("rviz/SetGoal");
    rviz::Property* pro = current_tool->getPropertyContainer();
    pro->subProp("Topic")->setValue("/move_base_simple/goal");
    tool_manager->setCurrentTool(current_tool);
}

void qrviz::set_init_pose()
{
    rviz::Tool* current_tool = tool_manager->addTool("rviz/SetInitialPose");
    tool_manager->setCurrentTool(current_tool);
}