/*
 *  Custom class definition to group useful properties useful
 *  for path execution
 * 
 *  Diary:
 * 
 *  - [17/9/22] Started writing
 */

#ifndef PATH_PROPERTIES_H
#define PATH_PROPERTIES_H

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h> // accepts only one goal at a time!

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/MoveGroupAction.h>
#include "moveit_msgs/RobotTrajectory.h"
#include "moveit_msgs/MoveItErrorCodes.h"
#include <moveit_msgs/MotionSequenceItem.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

#include <debug_codes.h>

#define ROBOT_DOF 3
#define MAX_TRAJ_POINTS 5

typedef unsigned char debug_error;

class path_properties
{
private:
    // total path waypoints
    int _tot_waypoints;
    int _tot_cs_states;
    
    // end-effector position
    float _cur_end_effector_p_x;
    float _cur_end_effector_p_y;
    float _cur_end_effector_p_z;

    // end-effector velocity
    float _cur_end_effector_v_x;
    float _cur_end_effector_v_y;
    float _cur_end_effector_v_z;

    float _par_tmp;

    double _blend_radius;

public:
    path_properties(ros::NodeHandle node_handle);
    ~path_properties();

    bool return_fn_state;

    std::vector<double> goal_joint_values;

    // The robot joint group defined in moveit configuration package manager
    std::string p_s_group_name;
    std::string p_robot_description;
    std::string p_s_end_effector_name;
    const moveit::core::JointModelGroup* p_joint_model_group; 
    // Public Variables
    double p_eef_step;
    double p_jump_threshold;
    double p_cart_path;
    moveit_msgs::RobotTrajectory p_traj_cart_path;
    std::vector<geometry_msgs::Pose> p_path_waypoints;
    geometry_msgs::Pose * cartesian_points;
    moveit_msgs::MoveItErrorCodes *error_code_returned;
    double (*ptr2joints)[ROBOT_DOF]; // ptr2joints is a pointer to an array of ROBOT_DOF=3 const double vars -> enough to access a 2D array!
    //const double *ptr2cs_states_traj[]; // declaration of an array of pointers of type const double
    moveit_msgs::MotionSequenceItemPtr seq;
    moveit::core::RobotState * p_cur_state;
    moveit::core::RobotStatePtr p_cur_state_ptr;

    moveit::planning_interface::MoveGroupInterface *ptr2group;

    char seq_cnt;
    
    // Methods for I/O user defined data from the Parameter Server
    void getPathWaypoints(ros::NodeHandle node_handle, debug_error * custom_error_code);
    void getConfigSpaceStates(ros::NodeHandle node_handle, debug_error * custom_error_code);
    void printComputedTrajectory(moveit_msgs::RobotTrajectory &trajectory);

    // Methods for generating ROS msgs used for trajectory implementation
    void fillMotionSequenceItem(ros::NodeHandle node_handle, moveit::core::RobotStatePtr& kin_state, const moveit::core::JointModelGroup* mod_group, moveit_msgs::MotionSequenceItemPtr& motion_seq_item, debug_error * custom_error_code );
    void fillMotionPlanRequestMsg(ros::NodeHandle node_handle, moveit::core::RobotStatePtr& kin_state, moveit_msgs::MotionPlanRequest mot_pl_req, char path_state_cnt, debug_error * custom_error_code );

    void executeJointSpaceSegment(ros::NodeHandle node_handle, int path_state_cnt, debug_error * custom_error_code );

};


#endif