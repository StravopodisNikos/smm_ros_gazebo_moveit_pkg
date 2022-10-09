// Main ROS Library
#include <ros/ros.h>
// ActionLib Libraries

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

const std::string MOVE_GR_ACT_NAME("/move_group");
const std::string SEQ_MOVE_GR_ACT_NAME("/sequence_move_group");

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_task_space_ind");
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
    // Action Client
    //actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> mot_seq_acli(node_handle,SEQ_MOVE_GR_ACT_NAME);

    // Get Robot Model for accessing kinematic data
    robot_model_loader::RobotModelLoader robot_model_loader(path2execute.p_robot_description);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

    // Access Joints' data
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    //const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("smm_arm");
    path2execute.p_joint_model_group = kinematic_model->getJointModelGroup(path2execute.p_s_group_name);
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state->getJointModelGroup(path2execute.p_s_group_name); //same thing
    

    // Using the RobotModel, we construct a PlanningScene that maintains the state of the world (including the robot).
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));

    // Setup a PlanningPipeline object. Uses the Parameter Server to determine request adapters and the plugins
    planning_pipeline::PlanningPipelinePtr planning_pipeline(
        new planning_pipeline::PlanningPipeline(kinematic_model, node_handle, "planning_plugin", "request_adapters")
    );

    // Configure a valid robot state
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

    // Construct a loader to load a planner, by name.( Using the ROS pluginlib library)
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    // We will get the name of planning plugin we want to load
    // from the ROS parameter server, and then load the planner
    // making sure to catch all exceptions.
    if (!node_handle.getParam("/planning_task_space_ind/planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(kinematic_model, node_handle.getNamespace()))
        ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (const auto& cls : classes)
        ss << cls << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                            << "Available plugins: " << ss.str());
    }

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
    path2execute.getPathWaypoints(node_handle, &my_path_debug_code);
    path2execute.getConfigSpaceStates(node_handle, &my_path_debug_code);

	//const double cs02 =  *(*(path2execute.ptr2joints + 0) + 2);
    // Assign the joint positions of the first state to the group
    // (Example) kinematic_state->setJointGroupPositions(joint_model_group, *(path2execute.ptr2joints + 0));
	
    // II.i Create a motion plan sequence(!) request
    //moveit_msgs::MotionSequenceRequest seq0;
    moveit_msgs::MotionPlanRequest seq0;
    planning_interface::MotionPlanResponse res0;
    path2execute.seq_cnt = 0;
    path2execute.fillMotionPlanRequestMsg(node_handle, kinematic_state, seq0 ,path2execute.seq_cnt,&my_path_debug_code);

    // II.ii Construct a planning context
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, seq0, res0.error_code_);
    context->solve(res0);
    if (res0.error_code_.val != res0.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        my_path_debug_code = PLANNING_FAILED;
    }

    // II.iii Visualize the result
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    moveit_msgs::MotionPlanResponse response;
    res0.getMessage(response);
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher.publish(display_trajectory);

    // II.iv Change the current state to the previous goal state
    kinematic_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*kinematic_state.get());

    // II.v Display the goal state
    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
    visual_tools.trigger();

    // III. Calling for action
    /*
    ROS_INFO("Calling the action server... ");
    //mot_seq_acli.sendGoal(seq0);

    //bool finished_before_timeout = mot_seq_acli.waitForResult(ros::Duration(5.0));
    bool finished_before_timeout = true;
    if (finished_before_timeout)
    {
        //actionlib::SimpleClientGoalState state = mot_seq_acli.getState();
        //ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
    }
    */

    // Compute the task path
    //ROS_INFO("Computing the cartesian task path... ");
    //group.setPoseReferenceFrame("base_plate");
    //path2execute.p_cart_path = group.computeCartesianPath(path2execute.p_path_waypoints, path2execute.p_eef_step, path2execute.p_jump_threshold, path2execute.p_traj_cart_path,false, path2execute.error_code_returned);
    //sleep(2.0);

    // Print the extracted trajectory
    //ROS_INFO("Printing the cartesian task path... ");
    //path2execute.printComputedTrajectory(path2execute.p_traj_cart_path);

    // Visualize the plan in RViz

    /// Plan & Execute p2p motion
    //ROS_INFO("Planning the cartesian task path... ");
    //moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //moveit::core::MoveItErrorCode success = group.plan(my_plan); 
    //sleep(2.0); // Sleep to give Rviz time to visualize the plan
    //ROS_INFO("Executing the cartesian task path... ");
    //group.execute(my_plan);	
    sleep(2.0);
    planner_instance.reset();

    // Terminate ROS node
    ros::shutdown();  

    return 0;
}
