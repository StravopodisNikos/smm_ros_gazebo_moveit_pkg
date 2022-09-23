// Main ROS Library
#include <ros/ros.h>
// MoveIt Libraries
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/MotionSequenceRequest.h>
#include <moveit_msgs/MotionSequenceResponse.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
// Custom Libraries
#include <path_properties.hpp>
// C++ Libraries
#include <iostream>
/*
 *  Executes a cartesian path, through the user defined joint configurations.
 *  IK MUST be solved in Matlab and joint configurations @ each path point are
 *  outsourced from trajectory implementation data saved in MATLAB Code!
 */
debug_error my_path_debug_code;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_task_space");
    ros::NodeHandle node_handle;  
    ros::AsyncSpinner spinner(1);
    path_properties path2execute(node_handle);

    spinner.start();

    /* This sleep is ONLY to allow Rviz to come up */
    sleep(2.0);

    // Define the joint group constructed with moveit manager
    moveit::planning_interface::MoveGroupInterface group(path2execute.p_s_group_name);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

    // Enabling the Pilz Industrial Motion Planner
    // same as group("smm_arm")

    // Get Robot Model for accessing kinematic data
    robot_model_loader::RobotModelLoader robot_model_loader(path2execute.p_robot_description);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

    // Access Joints' data
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("smm_arm");

    // Setup a PlanningPipeline object. Uses the Parameter Server to determine request adapters and the plugins
    planning_pipeline::PlanningPipelinePtr planning_pipeline(
        new planning_pipeline::PlanningPipeline(kinematic_model, node_handle, "planning_plugin", "request_adapters")
    );


    // (Optional) Set Planning properties
    //group.setPlanningTime(5.0);
    //group.setNumPlanningAttempts(2);
    //group.setGoalOrientationTolerance(1000.0);
    //group.setGoalPositionTolerance(0.01);
    
    // (Optional) Create a publisher for visualizing plans in Rviz.
    //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    //moveit_msgs::DisplayTrajectory display_trajectory;

    // Rviz visualization tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

    // I. Get path waypoints from Parameter Server
    ROS_INFO("Extracting path waypoints from Parameter Server... ");
    //path2execute.getPathWaypoints(node_handle);
    path2execute.getConfigSpaceStates(node_handle, &my_path_debug_code);

	//const double cs02 =  *(*(path2execute.ptr2joints + 0) + 2);
    // Assign the joint positions of the first state to the group
    // (Example) kinematic_state->setJointGroupPositions(joint_model_group, *(path2execute.ptr2joints + 0));
	
    // II. Create a motion plan request
    //planning_interface::
    path2execute.fillMotionSequenceItem(node_handle, kinematic_state, joint_model_group, path2execute.seq, &my_path_debug_code);


    // Compute the task path
    ROS_INFO("Computing the cartesian task path... ");
    //group.setPoseReferenceFrame("base_plate");
    //path2execute.p_cart_path = group.computeCartesianPath(path2execute.p_path_waypoints, path2execute.p_eef_step, path2execute.p_jump_threshold, path2execute.p_traj_cart_path,false, path2execute.error_code_returned);
    sleep(2.0);

    // Print the extracted trajectory
    ROS_INFO("Printing the cartesian task path... ");
    //path2execute.printComputedTrajectory(path2execute.p_traj_cart_path);

    // Visualize the plan in RViz

    /// Plan & Execute p2p motion
    ROS_INFO("Planning the cartesian task path... ");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //moveit::core::MoveItErrorCode success = group.plan(my_plan); 
    sleep(2.0); // Sleep to give Rviz time to visualize the plan
    ROS_INFO("Executing the cartesian task path... ");
    //group.execute(my_plan);	
    sleep(2.0);

    // Terminate ROS node
    ros::shutdown();  

    return 0;
}
