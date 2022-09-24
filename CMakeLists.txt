cmake_minimum_required(VERSION 3.1.3)
project(smm_ros_gazebo_moveit_pkg)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    Eigen3
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    roslaunch
    actionlib
    actionlib_msgs
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    actionlib
    actionlib_msgs
  DEPENDS
    EIGEN3
)

# custom library
include_directories(
	include
	SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_library(path_properties
  src/path_properties.cpp
)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#install(
#  TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)
#install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

# roslaunch
roslaunch_add_file_check(launch)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# rosnodes
add_executable(planning_joint_space_node src/planning_joint_space.cpp)
target_link_libraries(planning_joint_space_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(torque_sensor1_subscriber_node src/torque_sensor1_subscriber.cpp)
target_link_libraries(torque_sensor1_subscriber_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(planning_task_space src/planning_task_space.cpp)
target_link_libraries(planning_task_space path_properties ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(planning_task_space_ind src/planning_task_space_ind.cpp)
target_link_libraries(planning_task_space_ind path_properties ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(torque_sensor2_subscriber_node src/torque_sensor2_subscriber.cpp)
#target_link_libraries(torque_sensor2_subscriber_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
