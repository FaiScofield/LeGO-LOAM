#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
//#include <Vector3.h>

#include <iostream>
#include <iomanip>  // std::setprecision
#include <fstream>
#include <eigen3/Eigen/Core>

std::ofstream f;
std::string file_name;

void odomCallback(const nav_msgs::OdometryConstPtr& odom)
{
//    ROS_INFO("Got a odom message.");
    geometry_msgs::Point p = odom->pose.pose.position;
    geometry_msgs::Quaternion q = odom->pose.pose.orientation;
    tf::Matrix3x3 R(tf::Quaternion(q.x, q.y, q.z, q.w));
    double r[12];
    R.getOpenGLSubMatrix(r);

    f.open(file_name, std::ios_base::app);
    f << std::fixed;    // 禁用科学计数法
    f << std::setprecision(6)
      << p.x  << " " << p.y  << " " << p.z  << " "
      << r[0] << " " << r[1] << " " << r[2] << " "
      << r[4] << " " << r[5] << " " << r[6] << " "
      << r[8] << " " << r[9] << " " << r[10] << std::endl;
    f.close();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "saveTrajectory");
    ros::NodeHandle nh;

    ros::Subscriber subOdom = nh.subscribe("/integrated_to_init", 100, odomCallback);

    ros::param::get("~trajectory_file", file_name);
    if (file_name.empty())
        file_name = "/home/vance/velodyne_ws/src/LeGO-LOAM/laser_trajectory.txt";

    ROS_INFO("\033[1;32m---->\033[0m Trajectory Saving Started.");
    ROS_INFO("Saving trajectory to %s", file_name.c_str());

    f.open(file_name);
    f << std::fixed;
    f.close();

    while (ros::ok()) {
        ros::spinOnce();
    }

    ROS_INFO("Trajectory file saved.");

    return 0;
}



