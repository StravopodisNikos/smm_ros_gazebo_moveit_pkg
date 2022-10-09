// Main ROS Library
#include <ros/ros.h>
// MoveIt Libraries
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
// Custom Libraries
#include <path_properties.hpp>
// C++ Libraries
#include <iostream>

debug_error my_path_debug_code;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_joint_space_sequence");
    ros::NodeHandle node_handle;  
    ros::AsyncSpinner spinner(1);
    path_properties path2execute(node_handle);

    spinner.start();

    /* This sleep is ONLY to allow Rviz to come up */
    sleep(2.0);

    // Define the joint group constructed with moveit manager
    moveit::planning_interface::MoveGroupInterface group("smm_arm");
    path2execute.ptr2group = &group;

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

    // (Optional) Create a publisher for visualizing plans in Rviz.
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Rviz visualization tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

    // Getting Basic Information
    ROS_INFO("Base frame: %s", group.getPlanningFrame().c_str());
    path2execute.return_fn_state = group.setEndEffectorLink("massage_tool_2");
    ROS_INFO("Tool frame: %s", group.getEndEffectorLink().c_str());

    // Get Robot Model for accessing kinematic data
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());   

    // Access Joints' data
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    path2execute.p_joint_model_group = kinematic_model->getJointModelGroup(path2execute.p_s_group_name);
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("smm_arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    path2execute.p_cur_state_ptr = kinematic_state;

    ROS_INFO("[INFO] CURRENT JOINT ANGLES: ");
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    // Access Links' data
    moveit::core::LinkModel tool_link_model("massage_tool_2");

    // Load Joint States from Parameter Seever
    ROS_INFO("Extracting Joint States from Parameter Server... ");
    path2execute.getConfigSpaceStates(node_handle, &my_path_debug_code);
    
    // Segment sequence execution
    path2execute.executeJointSpaceSegment(node_handle, 0, &my_path_debug_code);

    path2execute.executeJointSpaceSegment(node_handle, 1, &my_path_debug_code);
    // Terminate ROS node
    ros::shutdown();  

    return 0;
}