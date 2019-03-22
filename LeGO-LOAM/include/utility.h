#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "cloud_msgs/cloud_info.h"

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

// for debug
#include <chrono>
#include <pcl/io/io.h>

#define PI 3.14159265

using namespace std;

typedef pcl::PointXYZI  PointType;

/*
 * Horizon_SCAN = 360 / horitontal_resolution. For velodyne, horitontal_resolution = 0.2.
 * GroundScanInd means roughly how many scans represent ground from the bottom.
 *
 * For 64ES3:
 * Angular Resolution (Vertical): 0.4°
 * Angular Resolution (Horizontal/Azimuth(yaw)): 0.0864° – 0.3456°
 *  RPM   RPS(Hz)   Total Laser Points   Points Per Laser       Angular
 *                  per Revolution       per Revolution     Resolution(degrees)
 * 0300     05          266,627             4167                0.0864
 * 0600     10          133,333             2083                0.1728
 * 0900     15          88,889              1389                0.2592
 * 1200     20          66,657              1042                0.3456
 *
 * Vertical Field of View(VFOV) 26.8 degrees(+2° to -24.8°):
 *    +2.00 to - 8.33 @ 1/3 degree spacing for 32 lasers
 *    -8.83 to -24.33 @ 1/2 degree spacing for 32 lasers
 *
 */

// VLP-16
// extern const int N_SCAN = 16;
//extern const int Horizon_SCAN = 1800;
//extern const float ang_res_x = 0.2;
//extern const float ang_res_y = 2.0;
//extern const float ang_bottom = 15.0+0.1;
//extern const int groundScanInd = 7;

// 64ES3
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 26.8/float(N_SCAN-1);
extern const float ang_bottom = 15.1;   // 15.1, 24.8
extern const int groundScanInd = 20;

// Ouster OS1-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 15;


extern const bool loopClosureEnableFlag = true;
extern const double mappingProcessInterval = 0.3;   //建图处理间隔s

extern const float scanPeriod = 0.1;
extern const int systemDelay = 0;
extern const int imuQueLength = 200;
extern const string imuTopic = "/imu/data";


extern const float sensorMountAngle = 0.0;
extern const float segmentTheta = 1.0472;   //点云分割时的角度跨度上限（π/3）
extern const int segmentValidPointNum = 5;  //检查上下左右连续5个点做为分割的特征依据
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


extern const int edgeFeatureNum = 2;
extern const int surfFeatureNum = 4;
extern const int sectionsTotal = 6;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;

extern const float surroundingKeyframeSearchRadius = 50.0;  //关键帧搜索半径m
extern const int   surroundingKeyframeSearchNum = 50;       //关键帧搜索数量

extern const float historyKeyframeSearchRadius = 5.0;       //历史帧搜索半径m
extern const int   historyKeyframeSearchNum = 25;
extern const float historyKeyframeFitnessScore = 0.3;

extern const float globalMapVisualizationSearchRadius = 1000.0; // 500

extern const bool saveDataForDebug = false;  // for debug


struct smoothness_t{
    float value;
    size_t ind;
};

struct by_value{
    bool operator()(smoothness_t const &left, smoothness_t const &right) {
        return left.value < right.value;
    }
};

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (float, roll, roll)
                                   (float, pitch, pitch)
                                   (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

#endif
