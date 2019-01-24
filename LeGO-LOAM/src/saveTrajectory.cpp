#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <iomanip>  // std::setprecision
#include <fstream>
#include <eigen3/Eigen/Core>
#include <pcl/common/common.h>

std::ofstream f;
std::string file_name;

void odomCallback(const nav_msgs::OdometryConstPtr& odom)
{
    geometry_msgs::Point p = odom->pose.pose.position;
    geometry_msgs::Quaternion q = odom->pose.pose.orientation;
    tf::Matrix3x3 R(tf::Quaternion(q.x, q.y, q.z, q.w));
    double r[12];
    R.getOpenGLSubMatrix(r);

    f.open(file_name, std::ios_base::app);
    f << std::fixed << std::setprecision(6) // 禁用科学计数法
      << r[0] << " " << r[1] << " " << r[2]  << " " << -p.x << " "
      << r[4] << " " << r[5] << " " << r[6]  << " " << p.y << " "
      << r[8] << " " << r[9] << " " << r[10] << " " << p.z << std::endl;
    f.close();
}

/*
void keyPoseCallback(const sensor_msgs::PointCloud2Ptr& posesCloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr poses(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*posesCloud, *poses);
    pcl::PointXYZI p = poses->points.back();

    f.open(file_name, std::ios_base::app);
//    f << std::fixed << td::setprecision(6);    // 禁用科学计数法
    f << p.x << " " << p.y << " " << p.z << std::endl;
    f.close();
}
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "saveTrajectory");
    ros::NodeHandle nh;

    ros::Subscriber subOdom = nh.subscribe("/integrated_to_init", 100, odomCallback);
//    ros::Subscriber subPose = nh.subscribe("/key_pose_origin", 100, keyPoseCallback);

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



