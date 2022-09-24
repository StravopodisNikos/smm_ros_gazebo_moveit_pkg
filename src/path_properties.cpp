#include <path_properties.hpp>
#include <debug_codes.h>
#include <iostream>
#include <string> 


const std::string PIPELINE_ID("pilz_industrial_motion_planner");
//const std::string PLANNER_ID("PTP");
//const std::string GROUP_NAME("panda_arm");
//const std::string LINK_NAME("panda_hand");

path_properties::path_properties(ros::NodeHandle node_handle)
{
    node_handle.getParam("/path_waypoints/tot_path_waypoints", _tot_waypoints);
    node_handle.getParam("/cs_states/tot_cs_states", _tot_cs_states);
    //ROS_INFO_STREAM("[path_properties] _tot_waypoints1:" << _tot_waypoints << "\n" );
    //node_handle.getParam("/path_waypoints/point0_x", _par_tmp);
    //ROS_INFO_STREAM("[getPathWaypoints] position_x0:" << _par_tmp << "\n" );

    p_eef_step = 0.05;
    p_jump_threshold = 0.01;
    //ROS_INFO_STREAM("[path_properties] p_eef_step:" << p_eef_step << "\n" );
    //ROS_INFO_STREAM("[path_properties] p_jump_threshold:" << p_jump_threshold << "\n" );

    _par_tmp = 0;

    p_s_group_name = "smm_arm";
    p_s_end_effector_name = "massage_tool_2";
    p_robot_description = "robot_description";
}

path_properties::~path_properties()
{
}

void path_properties::getPathWaypoints(ros::NodeHandle node_handle, debug_error * custom_error_code)
{
    /*
     *  Loads the user-specified points from Parameter Server and returns the
     *  vector of the waypoints.
     */  
    ROS_INFO_STREAM("[getPathWaypoints] _tot_waypoints:" << _tot_waypoints << "\n" );

    //geometry_msgs::Pose cartesian_points[_tot_waypoints];

    // default string rosparam names
    std::string s_0 = "/path_waypoints/point";
    std::string s_x = "_x";
    std::string s_y = "_y";
    std::string s_z = "_z";

    if ( (_tot_waypoints > 0) && (_tot_waypoints < MAX_TRAJ_POINTS) )
    {
        for (int i = 0; i < _tot_waypoints; i++)
        {
            // create the param string
            auto s_i = std::to_string(i);

            // load the parameterσ
            node_handle.getParam( (const std::string) s_0+s_i+s_x, (cartesian_points+i)->position.x );
            ROS_INFO_STREAM("[getPathWaypoints] position_x:" << (cartesian_points+i)->position.x << "\n" );
            node_handle.getParam( (const std::string) s_0+s_i+s_y, (cartesian_points+i)->position.y );
            ROS_INFO_STREAM("[getPathWaypoints] position_y:" << (cartesian_points+i)->position.y << "\n" );
            node_handle.getParam( (const std::string) s_0+s_i+s_z, (cartesian_points+i)->position.z );
            ROS_INFO_STREAM("[getPathWaypoints] position_z:" << (cartesian_points+i)->position.z << "\n" );        

            // build the vector
            p_path_waypoints.push_back(*(cartesian_points+i));
        }
        (* custom_error_code) = NO_ERROR;

    }
    else
    {
        (* custom_error_code) = VIOLATED_MAX_TRAJ_POINTS;
        ROS_ERROR_STREAM("[getPathWaypoints] Violated maximum number of trajectory points!");
    }

    return;
}

void path_properties::getConfigSpaceStates(ros::NodeHandle node_handle, debug_error * custom_error_code)
{
    /*
     *  Loads the user-specified joint-configurations from Parameter Server and returns a 2d array.
     */  
    ROS_INFO_STREAM("[getConfigSpaceStates] _tot_cs_states:" << _tot_cs_states << "\n" );
    
    // Now that the number of trajectory points is known a 2D array (an array that each element is an
    // array of const double[3]) can be constructed
    double cs_states_traj[_tot_cs_states][ROBOT_DOF];
    //      |           - - - - DOF - - - - [1 | 2 | 3]
    // _tot_cs_states(i)
    //      |
    //      
    //const double cs_states_traj[ROBOT_DOF];
    ptr2joints = cs_states_traj;
    
    // default string rosparam names
    std::string s1 = "/cs_states/joint1_";
    std::string s2 = "/cs_states/joint2_";
    std::string s3 = "/cs_states/joint3_";

    if ( (_tot_cs_states > 0) && (_tot_cs_states < MAX_TRAJ_POINTS) )
    {
        for (int i = 0; i < _tot_cs_states; i++)
        {
            // Each for iteration fills the elements of a row
            auto s_i = std::to_string(i);
            ROS_INFO_STREAM("[getConfigSpaceStates] Configuration State:" << i << "\n" );

            // read the 3 (=ROBOT_DOF) joint configurations from yaml and save them to an array
            node_handle.getParam( (const std::string) s1+s_i, *(*(ptr2joints + i) + 0));
            node_handle.getParam( (const std::string) s2+s_i, *(*(ptr2joints + i) + 1));
            node_handle.getParam( (const std::string) s3+s_i, *(*(ptr2joints + i) + 2));

            //ROS_INFO_STREAM("[getConfigSpaceStates] Joint 1:" << *(*(ptr2joints + i) + 0) << "\n" );
            //ROS_INFO_STREAM("[getConfigSpaceStates] Joint 2:" << *(*(ptr2joints + i) + 1) << "\n" );
            //ROS_INFO_STREAM("[getConfigSpaceStates] Joint 3:" << *(*(ptr2joints + i) + 2) << "\n" );

            // In main() the Configuration States can be accessed through ptr2joints
        }

        (* custom_error_code) = NO_ERROR;
    }
    else
    {
        (* custom_error_code) = VIOLATED_MAX_TRAJ_POINTS;
        ROS_ERROR_STREAM("[getConfigSpaceStates] Violated maximum number of trajectory points!");
    }

    return;
}

void path_properties::printComputedTrajectory(moveit_msgs::RobotTrajectory &computed_trajectory)
{
    /*
     *  Prints the computed trajectory returned from computeCartesianPath()
     */

    ROS_INFO("Printing computed trajectory data... ");

    std::cout << "Number of active joints: " << sizeof(computed_trajectory.joint_trajectory.joint_names);

    return;
}

void path_properties::fillMotionSequenceItem(ros::NodeHandle node_handle, moveit::core::RobotStatePtr& kin_state, const moveit::core::JointModelGroup* mod_group, moveit_msgs::MotionSequenceItemPtr& motion_seq_item, debug_error * custom_error_code )
{
    /*
     *  
     */

    // Set the desired blend radius
    //node_handle.getParam("/plan_sequence/blend_radius1", motion_seq_item->blend_radius);
    //ROS_INFO_STREAM("[fillMotionSequenceItem] blend_radius:" << motion_seq_item->blend_radius); 

    // Set the elements of the moveit_msgs/MotionPlanRequest msg
    //node_handle.getParam("/plan_sequence/max_allowed_planning_time", motion_seq_item->req.allowed_planning_time);
    //ROS_INFO_STREAM("[fillMotionSequenceItem] allowed_planning_time:" << motion_seq_item->req.allowed_planning_time);     

    //node_handle.getParam("/plan_sequence/max_planning_attempts", motion_seq_item->req.num_planning_attempts);
    //ROS_INFO_STREAM("[fillMotionSequenceItem] num_planning_attempts:" << motion_seq_item->req.num_planning_attempts);

    //motion_seq_item->req.group_name = p_s_group_name;

    //kin_state->setJointGroupPositions(mod_group, *(ptr2joints+0) );
    kin_state->setJointGroupPositions(mod_group, *(ptr2joints+0) );
    moveit_msgs::Constraints joint_goal_cons = kinematic_constraints::constructGoalConstraints(*kin_state, mod_group, 0.002, 0.002);

    // View the derived joint constraints, only for evaluation
    return;
}

void path_properties::fillMotionPlanRequestMsg(ros::NodeHandle node_handle, moveit::core::RobotStatePtr& kin_state, moveit_msgs::MotionPlanRequestPtr ptr2_mot_pl_req, char path_state_cnt, debug_error * custom_error_code )
{
    /*
     *
     */

    ptr2_mot_pl_req->allowed_planning_time = 2.0;
    ptr2_mot_pl_req->cartesian_speed_end_effector_link = p_s_end_effector_name;
    ptr2_mot_pl_req->group_name = p_s_group_name;
    ptr2_mot_pl_req->planner_id = "PTP";
    ptr2_mot_pl_req->pipeline_id = PIPELINE_ID;
    ptr2_mot_pl_req->num_planning_attempts = 2;
    //ptr2_mot_pl_req->workspace_parameters // intentionally avoided

    if (path_state_cnt == 0)
    {
        moveit::core::robotStateToRobotStateMsg(*kin_state, ptr2_mot_pl_req->start_state );
    }
    else
    {
        moveit::core::robotStateToRobotStateMsg(*p_cur_state, ptr2_mot_pl_req->start_state );
    }

    // Set the goal_constraints
    moveit::core::RobotState goal_state(*kin_state);
    goal_state.setJointGroupPositions(p_joint_model_group, *(ptr2joints+path_state_cnt));
    moveit_msgs::Constraints joint_goal_cons = kinematic_constraints::constructGoalConstraints(*kin_state, p_joint_model_group, 0.002, 0.002);
    ptr2_mot_pl_req->goal_constraints.clear();
    ptr2_mot_pl_req->goal_constraints.push_back(joint_goal_cons);

    // Set the path_constraints
    // ...
    // Set the trajectory_constraints
    // ...

    ptr2_mot_pl_req->max_velocity_scaling_factor = 0.5;
    ptr2_mot_pl_req->max_acceleration_scaling_factor = 0.9;
    ptr2_mot_pl_req->max_cartesian_speed = 1.1;
    
    // Finally set the current state to the goal state(assume successful plan+execute)
    p_cur_state = &goal_state;
    
    return;
}
