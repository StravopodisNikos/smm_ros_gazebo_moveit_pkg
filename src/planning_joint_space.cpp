#include <ros/ros.h>

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

bool return_fn_state;
float _cur_end_effector_pos_x;
float _cur_end_effector_pos_y;
float _cur_end_effector_pos_z;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_joint_space");
    ros::NodeHandle node_handle;  
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /* This sleep is ONLY to allow Rviz to come up */
    sleep(2.0);

    // Define the joint group constructed with moveit manager
    moveit::planning_interface::MoveGroupInterface group("smm_arm");

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
    return_fn_state = group.setEndEffectorLink("massage_tool_2");
    ROS_INFO("Tool frame: %s", group.getEndEffectorLink().c_str());

    // Get Robot Model for accessing kinematic data
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());   

    // Access Joints' data
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("smm_arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    ROS_INFO("[INFO] CURRENT JOINT ANGLES: ");
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    // Access Links' data
    moveit::core::LinkModel tool_link_model("massage_tool_2");

    // Assign goal joint values
    std::vector<double> goal_joint_values = { 1.5708, -0.5, 0.7854};
    kinematic_state->setJointGroupPositions(joint_model_group, goal_joint_values);
    ROS_INFO("[INFO] GOAL JOINT ANGLES: ");
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), goal_joint_values[i]);
        group.setJointValueTarget(joint_names[i].c_str(), goal_joint_values[i]);
    }
 
    /// Plan & Execute p2p motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::MoveItErrorCode success = group.plan(my_plan); 
    sleep(2.0); // Sleep to give Rviz time to visualize the plan
    group.execute(my_plan);	
    sleep(2.0);

    // Read current robot's joint state & end effector pose
    geometry_msgs::PoseStamped robot_current_pose = group.getCurrentPose("massage_tool_2");
    _cur_end_effector_pos_x = robot_current_pose.pose.position.x;
    _cur_end_effector_pos_y = robot_current_pose.pose.position.y;
    _cur_end_effector_pos_z = robot_current_pose.pose.position.z;
    ROS_INFO_STREAM("End-effector position X: " << _cur_end_effector_pos_x << "\n");
    ROS_INFO_STREAM("End-effector position Y: " << _cur_end_effector_pos_y << "\n");
    ROS_INFO_STREAM("End-effector position Z: " << _cur_end_effector_pos_z << "\n");

    // Read current end effector pose
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("massage_tool_2");
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

/*
 * ABORTED - TO BE ACCOMPLISHED IN DIFFERENT FILE AFTER MATLAB OPTIMIZATION
 *
    // Define the desired path in Cartesian-Space
    // A 2-segment linear path will be given: +0.1m@x & +0.1m@z
    ROS_INFO_STREAM("[INFO] Cartesian Path execution...");
    geometry_msgs::Pose cart_path_p1;
    geometry_msgs::Pose cart_path_p2;
    geometry_msgs::Pose cart_path_p3;
    cart_path_p1.position.x = 0.534613;
    cart_path_p1.position.y = 0.0402965;
    cart_path_p1.position.z = 0.73282;
    cart_path_p2.position.x = cart_path_p1.position.x - 0.05;
    cart_path_p2.position.y = cart_path_p1.position.y;
    cart_path_p2.position.z = cart_path_p1.position.z;
    cart_path_p3.position.x = cart_path_p2.position.x;
    cart_path_p3.position.y = cart_path_p2.position.y - 0.05;
    cart_path_p3.position.z = cart_path_p2.position.z;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(cart_path_p1);
    waypoints.push_back(cart_path_p2);
    waypoints.push_back(cart_path_p3);
    double eef_step = 0.05;
    double jump_threshold = 0.01;
    moveit_msgs::RobotTrajectory traj_cart_path;

    // Compute the Cartesian path
    double cart_path1 = group.computeCartesianPath(waypoints, eef_step, jump_threshold, traj_cart_path);
    sleep(2.0);
    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
    {
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    }
    //visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Execute the path
    group.execute(traj_cart_path);
    sleep(2.0);

    */

    // Read joint sensor data

    // Terminate ROS node
    ros::shutdown();  

    return 0;
}
