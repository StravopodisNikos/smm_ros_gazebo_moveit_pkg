#include "ros/ros.h"
#include <rosbag/bag.h>
#include <geometry_msgs/WrenchStamped.h>


//float f_x;
//float f_y;
//float f_z;
//float t_x;
//float t_y;
float t_z;
std::vector<float> torque_joint2;

/*
 *  Callback function that saves the wrench vector when a new msg is
 *  received from force/torque sensor. 
 */
void ft_chatter_Callback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
    // create temporary storage
    //geometry_msgs::Vector3 forces;

    //f_x = msg->wrench.force.x;
    //f_y = msg->wrench.force.y;
    //f_z = msg->wrench.force.z;

    //t_x = msg->wrench.torque.x;
    //t_y = msg->wrench.torque.y;
    t_z = msg->wrench.torque.z;

    //ft_sensor_data_file.write((char *) &f_x, sizeof f_x);
    //ft_sensor_data_file.write((char *) &f_y, sizeof f_y);
    //ft_sensor_data_file.write((char *) &f_z, sizeof f_z);

    //ft_sensor_data_file.write((char *) &t_x, sizeof t_x);
    //ft_sensor_data_file.write((char *) &t_y, sizeof t_y);
    //ft_sensor_data_file.write((char *) &t_z, sizeof t_z);

    torque_joint2.push_back(t_z);

    //ROS_INFO_STREAM("End-effector torque X: " << t_x << "\n");
    //ROS_INFO_STREAM("End-effector torque Y: " << t_y << "\n");
    ROS_INFO_STREAM("End-effector torque Z: " << t_z << "\n");
    

    bag.write("torque_sensor2_topic",ros::Time::now(), msg);
    
    //ft_sensor_data_file << "\n";


    //ROS_INFO_STREAM("Writing force/torque data to log file...");
}

int main(int argc, char **argv)
{    
    ros::init(argc, argv, "torque_sensor2_subscriber");

    ros::NodeHandle n;

    rosbag::Bag bag("home/nikos/catkin_ws/src/smm_ros_gazebo_moveit_pkg/bagfiles/ft_log1.bag", rosbag::bagmode::Write);


    ros::Subscriber sub = n.subscribe("torque_sensor2_topic", 1000, ft_chatter_Callback);

    bag.close();

    ros::spin();

  return 0;
}