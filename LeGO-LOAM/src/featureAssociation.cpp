// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

#include "utility.h"
#include <pcl/io/pcd_io.h>

class FeatureAssociation{

private:
    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    ros::Subscriber subLaserCloudInfo;
    ros::Subscriber subOutlierCloud;
    ros::Subscriber subImu;

    ros::Publisher pubCornerPointsSharp;
    ros::Publisher pubCornerPointsLessSharp;
    ros::Publisher pubSurfPointsFlat;
    ros::Publisher pubSurfPointsLessFlat;

    pcl::PointCloud<PointType>::Ptr undistortionCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
    pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<PointType>::Ptr surfPointsFlat;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;   //下采样后的点云

    pcl::VoxelGrid<PointType> downSizeFilter;

    ///以下为点云消息的时间戳
    double timeScanCur;
    double timeNewSegmentedCloud;
    double timeNewSegmentedCloudInfo;
    double timeNewOutlierCloud;

    ///以下为新消息到来的标志
    bool newSegmentedCloud;
    bool newSegmentedCloudInfo;
    bool newOutlierCloud;

    cloud_msgs::cloud_info segInfo;
    std_msgs::Header cloudHeader;

    int systemInitCount;
    bool systemInited;

    std::vector<smoothness_t> cloudSmoothness;
    float cloudCurvature[N_SCAN*Horizon_SCAN];      //点云曲率
    int cloudNeighborPicked[N_SCAN*Horizon_SCAN];   //点是否筛选过标志：0-未筛选过，1-筛选过
    int cloudLabel[N_SCAN*Horizon_SCAN];            //点分类标号:2-代表曲率很大，1-代表曲率比较大,-1-代表曲率很小，0-曲率比较小(其中1包含了2,0包含了-1,0和1构成了点云全部的点)

    ///以下为imu数据
    int imuPointerFront;    //imu时间戳大于当前点云时间戳的位置
    int imuPointerLast;     //imu最新收到的点在数组中的位置
    int imuPointerLastIteration;

    //一帧中第一个/最后一个/当前点的RPY
//    以及当前点的位移/速度/欧拉角
    float imuRollStart, imuPitchStart, imuYawStart;
    float imuRollLast, imuPitchLast, imuYawLast;
    float imuRollCur, imuPitchCur, imuYawCur;
    float cosImuRollStart, cosImuPitchStart, cosImuYawStart, sinImuRollStart, sinImuPitchStart, sinImuYawStart;

    //第一个/当前点的速度和平移量
    float imuVeloXStart, imuVeloYStart, imuVeloZStart;
    float imuShiftXStart, imuShiftYStart, imuShiftZStart;
    float imuVeloXCur, imuVeloYCur, imuVeloZCur;
    float imuShiftXCur, imuShiftYCur, imuShiftZCur;

    //当前点相对于开始第一个点的畸变位移，速度，角度
    float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
    float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

    float imuAngularRotationXCur, imuAngularRotationYCur, imuAngularRotationZCur;
    float imuAngularRotationXLast, imuAngularRotationYLast, imuAngularRotationZLast;
    float imuAngularFromStartX, imuAngularFromStartY, imuAngularFromStartZ;

    float imuShiftFromStartX, imuShiftFromStartY, imuShiftFromStartZ;
    float imuVeloFromStartX, imuVeloFromStartY, imuVeloFromStartZ;

    //IMU信息
    double imuTime[imuQueLength];
    float imuRoll[imuQueLength];
    float imuPitch[imuQueLength];
    float imuYaw[imuQueLength];

    float imuAccX[imuQueLength];
    float imuAccY[imuQueLength];
    float imuAccZ[imuQueLength];

    float imuVeloX[imuQueLength];
    float imuVeloY[imuQueLength];
    float imuVeloZ[imuQueLength];

    float imuShiftX[imuQueLength];
    float imuShiftY[imuQueLength];
    float imuShiftZ[imuQueLength];
    //角速度
    float imuAngularVeloX[imuQueLength];
    float imuAngularVeloY[imuQueLength];
    float imuAngularVeloZ[imuQueLength];
    //角度
    float imuAngularRotationX[imuQueLength];
    float imuAngularRotationY[imuQueLength];
    float imuAngularRotationZ[imuQueLength];

    ros::Publisher pubLaserCloudCornerLast;
    ros::Publisher pubLaserCloudSurfLast;
    ros::Publisher pubLaserOdometry;
    ros::Publisher pubOutlierCloudLast;
    ros::Publisher pubUndistortionCloud;

    int skipFrameNum;       //跳帧数1，控制发给laserMapping的频率
    bool systemInitedLM;    //LM准备好的标注位

    int laserCloudCornerLastNum;    //上一帧边缘特征点数量
    int laserCloudSurfLastNum;      //上一帧平面特征点数量

    int pointSelCornerInd[N_SCAN*Horizon_SCAN];
    float pointSearchCornerInd1[N_SCAN*Horizon_SCAN];
    float pointSearchCornerInd2[N_SCAN*Horizon_SCAN];

    int pointSelSurfInd[N_SCAN*Horizon_SCAN];
    float pointSearchSurfInd1[N_SCAN*Horizon_SCAN];
    float pointSearchSurfInd2[N_SCAN*Horizon_SCAN];
    float pointSearchSurfInd3[N_SCAN*Horizon_SCAN];

    //这里的顺序是针对相机坐标系的,故数据的输入顺序应为：Ry,Rz,Rx,ty,tz,tx
    float transformCur[6];  //当前帧相对上一帧的变换
    float transformSum[6];  //总变换，即相对首帧的变换


    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudOri;  //存放平面或边缘特征点，每次迭代前会清空
    pcl::PointCloud<PointType>::Ptr coeffSel;       //存放特征点的权重，每次迭代前会清空

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    PointType pointOri, pointSel/*选中的特征点*/, tripod1, tripod2, tripod3/*特征点的匹配点*/, pointProj/*投影点？unused*/, coeff;

    nav_msgs::Odometry laserOdometry;

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform laserOdometryTrans;

    bool isDegenerate;
    cv::Mat matP;

    int frameCount;

    int frameNum = 0;
    double tatalRunTime = 0.0;

public:
    FeatureAssociation(): nh("~") {
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 1, &FeatureAssociation::laserCloudHandler, this);
        subLaserCloudInfo = nh.subscribe<cloud_msgs::cloud_info>("/segmented_cloud_info", 1, &FeatureAssociation::laserCloudInfoHandler, this);
        subOutlierCloud = nh.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud", 1, &FeatureAssociation::outlierCloudHandler, this);
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 50, &FeatureAssociation::imuHandler, this);

        pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 1);
        pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 1);
        pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 1);
        pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 1);

        pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
        pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
        pubOutlierCloudLast = nh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2);
        pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);

        pubUndistortionCloud = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_undistortion", 1);


        initializationValue();
    }

    void initializationValue() {
        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);

        undistortionCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
        cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
        surfPointsFlat.reset(new pcl::PointCloud<PointType>());
        surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());

        surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>());
        surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>());

        timeScanCur = 0;
        timeNewSegmentedCloud = 0;
        timeNewSegmentedCloudInfo = 0;
        timeNewOutlierCloud = 0;

        newSegmentedCloud = false;
        newSegmentedCloudInfo = false;
        newOutlierCloud = false;

        systemInitCount = 0;
        systemInited = false;

        imuPointerFront = 0;
        imuPointerLast = -1;
        imuPointerLastIteration = 0;

        //点云数据开始第一个点的位移/速度/欧拉角
        imuRollStart = 0; imuPitchStart = 0; imuYawStart = 0;
        cosImuRollStart = 0; cosImuPitchStart = 0; cosImuYawStart = 0;
        sinImuRollStart = 0; sinImuPitchStart = 0; sinImuYawStart = 0;
        imuRollCur = 0; imuPitchCur = 0; imuYawCur = 0;

        imuVeloXStart = 0; imuVeloYStart = 0; imuVeloZStart = 0;
        imuShiftXStart = 0; imuShiftYStart = 0; imuShiftZStart = 0;

        //当前点的速度，位移信息
        imuVeloXCur = 0; imuVeloYCur = 0; imuVeloZCur = 0;
        imuShiftXCur = 0; imuShiftYCur = 0; imuShiftZCur = 0;

        //当前点相对于开始第一个点的畸变位移，速度
        imuShiftFromStartXCur = 0; imuShiftFromStartYCur = 0; imuShiftFromStartZCur = 0;
        imuVeloFromStartXCur = 0; imuVeloFromStartYCur = 0; imuVeloFromStartZCur = 0;

        imuAngularRotationXCur = 0; imuAngularRotationYCur = 0; imuAngularRotationZCur = 0;
        imuAngularRotationXLast = 0; imuAngularRotationYLast = 0; imuAngularRotationZLast = 0;
        imuAngularFromStartX = 0; imuAngularFromStartY = 0; imuAngularFromStartZ = 0;

        //IMU信息
        for (int i = 0; i < imuQueLength; ++i) {
            imuTime[i] = 0;
            imuRoll[i] = 0; imuPitch[i] = 0; imuYaw[i] = 0;
            imuAccX[i] = 0; imuAccY[i] = 0; imuAccZ[i] = 0;
            imuVeloX[i] = 0; imuVeloY[i] = 0; imuVeloZ[i] = 0;
            imuShiftX[i] = 0; imuShiftY[i] = 0; imuShiftZ[i] = 0;
            imuAngularVeloX[i] = 0; imuAngularVeloY[i] = 0; imuAngularVeloZ[i] = 0;
            imuAngularRotationX[i] = 0; imuAngularRotationY[i] = 0; imuAngularRotationZ[i] = 0;
        }

        skipFrameNum = 1;

        for (int i = 0; i < 6; ++i) {
            transformCur[i] = 0;
            transformSum[i] = 0;
        }

        systemInitedLM = false;

        imuRollLast = 0; imuPitchLast = 0; imuYawLast = 0;
        imuShiftFromStartX = 0; imuShiftFromStartY = 0; imuShiftFromStartZ = 0;
        imuVeloFromStartX = 0; imuVeloFromStartY = 0; imuVeloFromStartZ = 0;

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerLast.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfLast.reset(new pcl::KdTreeFLANN<PointType>());

        laserOdometry.header.frame_id = "/camera_init";
        laserOdometry.child_frame_id = "/laser_odom";

        laserOdometryTrans.frame_id_ = "/camera_init";
        laserOdometryTrans.child_frame_id_ = "/laser_odom";

        isDegenerate = false;
        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

        frameCount = skipFrameNum;
    }

    void updateImuRollPitchYawStartSinCos() {
        cosImuRollStart = cos(imuRollStart);
        cosImuPitchStart = cos(imuPitchStart);
        cosImuYawStart = cos(imuYawStart);
        sinImuRollStart = sin(imuRollStart);
        sinImuPitchStart = sin(imuPitchStart);
        sinImuYawStart = sin(imuYawStart);
    }

    //计算局部坐标系下点云中的点相对第一个开始点的由于加减速运动产生的位移畸变
    void ShiftToStartIMU(float pointTime) {
        //计算相对于第一个点由于加减速产生的畸变位移(全局坐标系下畸变位移量delta_Tg)
        //imuShiftFromStartCur = imuShiftCur - (imuShiftStart + imuVeloStart * pointTime)
        imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
        imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
        imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;

        /********************************************************************************
        Rz(pitch).inverse * Rx(pitch).inverse * Ry(yaw).inverse * delta_Tg
        transfrom from the global frame to the local frame
        *********************************************************************************/

        //绕y轴旋转(-imuYawStart)，即Ry(yaw).inverse
        float x1 = cosImuYawStart * imuShiftFromStartXCur - sinImuYawStart * imuShiftFromStartZCur;
        float y1 = imuShiftFromStartYCur;
        float z1 = sinImuYawStart * imuShiftFromStartXCur + cosImuYawStart * imuShiftFromStartZCur;

        //绕x轴旋转(-imuPitchStart)，即Rx(pitch).inverse
        float x2 = x1;
        float y2 = cosImuPitchStart * y1 + sinImuPitchStart * z1;
        float z2 = -sinImuPitchStart * y1 + cosImuPitchStart * z1;

        //绕z轴旋转(-imuRollStart)，即Rz(pitch).inverse
        imuShiftFromStartXCur = cosImuRollStart * x2 + sinImuRollStart * y2;
        imuShiftFromStartYCur = -sinImuRollStart * x2 + cosImuRollStart * y2;
        imuShiftFromStartZCur = z2;
    }

    //计算局部坐标系下点云中的点相对第一个开始点由于加减速产生的的速度畸变（增量）
    void VeloToStartIMU() {
        //计算相对于第一个点由于加减速产生的畸变速度(全局坐标系下畸变速度增量delta_Vg)
        imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
        imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
        imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

/********************************************************************************
  Rz(pitch).inverse * Rx(pitch).inverse * Ry(yaw).inverse * delta_Vg
  transfrom from the global frame to the local frame
*********************************************************************************/

        //绕y轴旋转(-imuYawStart)，即Ry(yaw).inverse
        float x1 = cosImuYawStart * imuVeloFromStartXCur - sinImuYawStart * imuVeloFromStartZCur;
        float y1 = imuVeloFromStartYCur;
        float z1 = sinImuYawStart * imuVeloFromStartXCur + cosImuYawStart * imuVeloFromStartZCur;

        //绕x轴旋转(-imuPitchStart)，即Rx(pitch).inverse
        float x2 = x1;
        float y2 = cosImuPitchStart * y1 + sinImuPitchStart * z1;
        float z2 = -sinImuPitchStart * y1 + cosImuPitchStart * z1;

        //绕z轴旋转(-imuRollStart)，即Rz(pitch).inverse
        imuVeloFromStartXCur = cosImuRollStart * x2 + sinImuRollStart * y2;
        imuVeloFromStartYCur = -sinImuRollStart * x2 + cosImuRollStart * y2;
        imuVeloFromStartZCur = z2;
    }

    //去除点云加减速产生的位移畸变
    void TransformToStartIMU(PointType *p) {
        /********************************************************************************
          Ry*Rx*Rz*Pl, transform point to the global frame
        *********************************************************************************/
        //绕z轴旋转(imuRollCur)
        float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
        float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
        float z1 = p->z;

        //绕x轴旋转(imuPitchCur)
        float x2 = x1;
        float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
        float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

        //绕y轴旋转(imuYawCur)
        float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
        float y3 = y2;
        float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

        /********************************************************************************
          Rz(pitch).inverse * Rx(pitch).inverse * Ry(yaw).inverse * Pg
          transfrom global points to the local frame
        *********************************************************************************/

        //绕y轴旋转(-imuYawStart)
        float x4 = cosImuYawStart * x3 - sinImuYawStart * z3;
        float y4 = y3;
        float z4 = sinImuYawStart * x3 + cosImuYawStart * z3;

        //绕x轴旋转(-imuPitchStart)
        float x5 = x4;
        float y5 = cosImuPitchStart * y4 + sinImuPitchStart * z4;
        float z5 = -sinImuPitchStart * y4 + cosImuPitchStart * z4;

        //绕z轴旋转(-imuRollStart)，然后叠加平移量
        p->x = cosImuRollStart * x5 + sinImuRollStart * y5 + imuShiftFromStartXCur;
        p->y = -sinImuRollStart * x5 + cosImuRollStart * y5 + imuShiftFromStartYCur;
        p->z = z5 + imuShiftFromStartZCur;
    }

    //积分速度与位移
    void AccumulateIMUShiftAndRotation() {
        float roll = imuRoll[imuPointerLast];
        float pitch = imuPitch[imuPointerLast];
        float yaw = imuYaw[imuPointerLast];
        float accX = imuAccX[imuPointerLast];
        float accY = imuAccY[imuPointerLast];
        float accZ = imuAccZ[imuPointerLast];

        //将当前时刻的加速度值绕交换过的ZXY固定轴（原XYZ）分别旋转(roll, pitch, yaw)角，转换得到世界坐标系下的加速度值(right hand rule)
        //绕z轴旋转(roll)
        float x1 = cos(roll) * accX - sin(roll) * accY;
        float y1 = sin(roll) * accX + cos(roll) * accY;
        float z1 = accZ;

        //绕x轴旋转(pitch)
        float x2 = x1;
        float y2 = cos(pitch) * y1 - sin(pitch) * z1;
        float z2 = sin(pitch) * y1 + cos(pitch) * z1;

        //绕y轴旋转(yaw)
        accX = cos(yaw) * x2 + sin(yaw) * z2;
        accY = y2;
        accZ = -sin(yaw) * x2 + cos(yaw) * z2;

        //上一个imu点
        int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
        //上一个点到当前点所经历的时间，即计算imu测量周期
        double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
        //要求imu的频率至少比lidar高，这样的imu信息才使用，后面校正也才有意义
        if (timeDiff < scanPeriod) {  //（隐含从静止开始运动）
            //求每个imu时间点的位移与速度,两点之间视为匀加速直线运动
            imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff + accX * timeDiff * timeDiff / 2;
            imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff + accY * timeDiff * timeDiff / 2;
            imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff + accZ * timeDiff * timeDiff / 2;

            imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
            imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
            imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;

            imuAngularRotationX[imuPointerLast] = imuAngularRotationX[imuPointerBack] + imuAngularVeloX[imuPointerBack] * timeDiff;
            imuAngularRotationY[imuPointerLast] = imuAngularRotationY[imuPointerBack] + imuAngularVeloY[imuPointerBack] * timeDiff;
            imuAngularRotationZ[imuPointerLast] = imuAngularRotationZ[imuPointerBack] + imuAngularVeloZ[imuPointerBack] * timeDiff;
        }
    }

    //接收imu消息，imu坐标系为x轴向前，y轴向右，z轴向上的坐标系
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn) {
        //convert Quaternion msg to Quaternion
        //This will get the roll pitch and yaw from the matrix about fixed axes X, Y, Z respectively. That's R = Rz(yaw)*Ry(pitch)*Rx(roll).
        //Here roll pitch yaw is in the global frame
        double roll, pitch, yaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imuIn->orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        //减去重力的影响,求出xyz方向的加速度实际值，并进行坐标轴交换，统一到z轴向前,x轴向左的右手坐标系, 交换过后RPY对应fixed axes ZXY(RPY---ZXY)。Now R = Ry(yaw)*Rx(pitch)*Rz(roll).
        float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
        float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
        float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;

        //循环移位效果，形成环形数组
        imuPointerLast = (imuPointerLast + 1) % imuQueLength;

        imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
        imuRoll[imuPointerLast] = roll;
        imuPitch[imuPointerLast] = pitch;
        imuYaw[imuPointerLast] = yaw;
        imuAccX[imuPointerLast] = accX;
        imuAccY[imuPointerLast] = accY;
        imuAccZ[imuPointerLast] = accZ;

        imuAngularVeloX[imuPointerLast] = imuIn->angular_velocity.x;
        imuAngularVeloY[imuPointerLast] = imuIn->angular_velocity.y;
        imuAngularVeloZ[imuPointerLast] = imuIn->angular_velocity.z;

        AccumulateIMUShiftAndRotation();
    }

    //接收点云数据，velodyne雷达坐标系安装为x轴向前，y轴向左，z轴向上的坐标系
    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
        cloudHeader = laserCloudMsg->header;

        //当前点云时间，消息的时间戳
        timeScanCur = cloudHeader.stamp.toSec();
        timeNewSegmentedCloud = timeScanCur;

        segmentedCloud->clear();
//        undistortionCloud->clear();
        pcl::fromROSMsg(*laserCloudMsg, *segmentedCloud);
//        pcl::fromROSMsg(*laserCloudMsg, *undistortionCloud);

        newSegmentedCloud = true;
    }

    void outlierCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msgIn) {
        timeNewOutlierCloud = msgIn->header.stamp.toSec();

        outlierCloud->clear();
        pcl::fromROSMsg(*msgIn, *outlierCloud);

        newOutlierCloud = true;
    }

    void laserCloudInfoHandler(const cloud_msgs::cloud_infoConstPtr& msgIn) {
        timeNewSegmentedCloudInfo = msgIn->header.stamp.toSec();
        segInfo = *msgIn;
        newSegmentedCloudInfo = true;
    }

    //通过imu数据校正雷达运动畸变
    void adjustDistortion() {
        //lidar扫描线是否旋转过半
        bool halfPassed = false;
        int cloudSize = segmentedCloud->points.size();

        PointType point;

        for (int i = 0; i < cloudSize; i++) {
            //坐标轴交换，velodyne lidar的坐标系也转换到z轴向前，x轴向左的右手坐标系
            point.x = segmentedCloud->points[i].y;
            point.y = segmentedCloud->points[i].z;
            point.z = segmentedCloud->points[i].x;

            //该点的旋转角
            float ori = -atan2(point.x, point.z);
            //根据扫描线是否旋转过半选择与起始位置还是终止位置进行差值计算，从而进行补偿
            if (!halfPassed) {
                //确保-pi/2 < ori - startOri < 3*pi/2
                if (ori < segInfo.startOrientation - M_PI / 2)
                    ori += 2 * M_PI;
                else if (ori > segInfo.startOrientation + M_PI * 3 / 2)
                    ori -= 2 * M_PI;

                if (ori - segInfo.startOrientation > M_PI)
                    halfPassed = true;
            } else {
                ori += 2 * M_PI;

                //确保-3*pi/2 < ori - endOri < pi/2
                if (ori < segInfo.endOrientation - M_PI * 3 / 2)
                    ori += 2 * M_PI;
                else if (ori > segInfo.endOrientation + M_PI / 2)
                    ori -= 2 * M_PI;
            }

            //-0.5 < relTime < 1.5（点旋转的角度与整个周期旋转角度的比率, 即点云中点的相对时间）
            float relTime = (ori - segInfo.startOrientation) / segInfo.orientationDiff;
            //点强度=强度+点相对时间（即一个整数+一个小数，整数部分是线号，小数部分是该点的相对时间）,匀速扫描：根据当前扫描的角度和扫描周期计算相对扫描起始位置的时间
            point.intensity = int(segmentedCloud->points[i].intensity) + scanPeriod * relTime;

            //如果收到IMU数据,使用IMU矫正点云畸变
            if (imuPointerLast >= 0) {
                //点时间=点云时间+周期时间
                float pointTime = relTime * scanPeriod; //计算点的周期时间
                imuPointerFront = imuPointerLastIteration;
                //寻找是否有点云的时间戳小于IMU的时间戳的IMU位置:imuPointerFront
                while (imuPointerFront != imuPointerLast) {
                    if (timeScanCur + pointTime < imuTime[imuPointerFront]) {
                        break;
                    }
                    imuPointerFront = (imuPointerFront + 1) % imuQueLength;
                }

                //没找到,此时imuPointerFront==imtPointerLast,只能以当前收到的最新的IMU的速度，位移，欧拉角作为当前点的速度，位移，欧拉角使用
                if (timeScanCur + pointTime > imuTime[imuPointerFront]) {
                    imuRollCur = imuRoll[imuPointerFront];
                    imuPitchCur = imuPitch[imuPointerFront];
                    imuYawCur = imuYaw[imuPointerFront];

                    imuVeloXCur = imuVeloX[imuPointerFront];
                    imuVeloYCur = imuVeloY[imuPointerFront];
                    imuVeloZCur = imuVeloZ[imuPointerFront];

                    imuShiftXCur = imuShiftX[imuPointerFront];
                    imuShiftYCur = imuShiftY[imuPointerFront];
                    imuShiftZCur = imuShiftZ[imuPointerFront];
                } else {
                    //找到了点云时间戳小于IMU时间戳的IMU位置,则该点必处于imuPointerBack和imuPointerFront之间，据此线性插值，计算点云点的速度，位移和欧拉角
                    int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
                    //按时间距离计算权重分配比率,也即线性插值
                    float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack])
                                                     / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                    float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime)
                                                    / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

                    imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
                    imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
                    if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > M_PI) {
                        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
                    } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -M_PI) {
                        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
                    } else {
                        imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
                    }

                    //本质:imuVeloXCur = imuVeloX[imuPointerback] + (imuVelX[imuPointerFront]-imuVelX[imuPoniterBack])*ratioFront
                    imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
                    imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
                    imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

                    imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
                    imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
                    imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
                }

                //如果是第一个点,记住点云起始位置的速度，位移，欧拉角
                if (i == 0) {
                    imuRollStart = imuRollCur;
                    imuPitchStart = imuPitchCur;
                    imuYawStart = imuYawCur;

                    imuVeloXStart = imuVeloXCur;
                    imuVeloYStart = imuVeloYCur;
                    imuVeloZStart = imuVeloZCur;

                    imuShiftXStart = imuShiftXCur;
                    imuShiftYStart = imuShiftYCur;
                    imuShiftZStart = imuShiftZCur;

                    if (timeScanCur + pointTime > imuTime[imuPointerFront]) {
                        imuAngularRotationXCur = imuAngularRotationX[imuPointerFront];
                        imuAngularRotationYCur = imuAngularRotationY[imuPointerFront];
                        imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront];
                    } else {
                        int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
                        float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack])
                                                         / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                        float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime)
                                                        / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                        imuAngularRotationXCur = imuAngularRotationX[imuPointerFront] * ratioFront + imuAngularRotationX[imuPointerBack] * ratioBack;
                        imuAngularRotationYCur = imuAngularRotationY[imuPointerFront] * ratioFront + imuAngularRotationY[imuPointerBack] * ratioBack;
                        imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront] * ratioFront + imuAngularRotationZ[imuPointerBack] * ratioBack;
                    }

                    imuAngularFromStartX = imuAngularRotationXCur - imuAngularRotationXLast;
                    imuAngularFromStartY = imuAngularRotationYCur - imuAngularRotationYLast;
                    imuAngularFromStartZ = imuAngularRotationZCur - imuAngularRotationZLast;

                    imuAngularRotationXLast = imuAngularRotationXCur;
                    imuAngularRotationYLast = imuAngularRotationYCur;
                    imuAngularRotationZLast = imuAngularRotationZCur;

                    updateImuRollPitchYawStartSinCos();
                }
                //如果不是第一个点，计算相对于第一个点的畸变，并对点其补偿矫正
                else {
                    VeloToStartIMU();           //将Lidar运动速度转到IMU起始坐标系下
                    TransformToStartIMU(&point);//将点坐标转到起始IMU坐标系下
                }
            }
            ///如果没有IMU数据,使用匀速运动模型矫正
//            else if (saveDataForDebug &&
//                     pubUndistortionCloud.getNumSubscribers() != 0){

//                PointType pp;
//                TransformToEnd(&point, &pp);

//                undistortionCloud->points[i] = pp;
//            }

            segmentedCloud->points[i] = point;
        }

        imuPointerLastIteration = imuPointerLast;
    }

    //计算曲率
    void calculateSmoothness() {
        int cloudSize = segmentedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++) {
            //直接以深度值之差的平方做曲率
            float diffRange = segInfo.segmentedCloudRange[i-5] + segInfo.segmentedCloudRange[i-4]
                            + segInfo.segmentedCloudRange[i-3] + segInfo.segmentedCloudRange[i-2]
                            + segInfo.segmentedCloudRange[i-1] - segInfo.segmentedCloudRange[i] * 10
                            + segInfo.segmentedCloudRange[i+1] + segInfo.segmentedCloudRange[i+2]
                            + segInfo.segmentedCloudRange[i+3] + segInfo.segmentedCloudRange[i+4]
                            + segInfo.segmentedCloudRange[i+5];

            //曲率计算
            cloudCurvature[i] = diffRange * diffRange;

            cloudNeighborPicked[i] = 0; //初始时，点全未筛选过
            cloudLabel[i] = 0;          //初始化为less flat点

            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    //挑选点，排除容易被斜面挡住的点以及离群点
    void markOccludedPoints() {
        int cloudSize = segmentedCloud->points.size();

        //挑选点，排除容易被斜面挡住的点以及离群点，有些点容易被斜面挡住，而离群点可能出现带有偶然性，这些情况都可能导致前后两次扫描不能被同时看到
        for (int i = 5; i < cloudSize - 6; ++i) { //与后一个点差值，所以减6
            float depth1 = segInfo.segmentedCloudRange[i];    //点的深度
            float depth2 = segInfo.segmentedCloudRange[i+1];  //后一个点的深度
            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[i+1] - segInfo.segmentedCloudColInd[i]));

            //两点相差2度的扫描角度且深度值相差30cm以内则认为是容易遮挡的，
            //将该点及前面/后面五个点（大致都在斜面上）全部置为筛选过
            if (columnDiff < 10) {
                if (depth1 - depth2 > 0.3) {
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                } else if (depth2 - depth1 > 0.3) {
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }

            float diff1 = std::abs(segInfo.segmentedCloudRange[i-1] - segInfo.segmentedCloudRange[i]);
            float diff2 = std::abs(segInfo.segmentedCloudRange[i+1] - segInfo.segmentedCloudRange[i]);

            //与前后点的平方和都大于深度平方和的百分之二，这些点视为离群点，包括陡斜面上的点，强烈凸凹点和空旷区域中的某些点，置为筛选过，弃用
            //如果当前点距离左右邻点都过远，则视其为瑕点，因为入射角可能太小导致误差较大
            if (diff1 > 0.02 * segInfo.segmentedCloudRange[i] && diff2 > 0.02 * segInfo.segmentedCloudRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    //提取特征
    void extractFeatures() {
        cornerPointsSharp->clear();
        cornerPointsLessSharp->clear();
        surfPointsFlat->clear();
        surfPointsLessFlat->clear();

        //将每条线上的点分入相应的类别：边缘点和平面点
        for (int i = 0; i < N_SCAN; i++) {
            surfPointsLessFlatScan->clear();

            //将每个scan的曲率点分成6等份处理,确保周围都有点被选作特征点
            for (int j = 0; j < 6; j++) {
                //六等份起点：sp = scanStartInd + (scanEndInd - scanStartInd)*j/6
                int sp = (segInfo.startRingIndex[i] * (6 - j)    + segInfo.endRingIndex[i] * j) / 6;
                //六等份终点：ep = scanStartInd - 1 + (scanEndInd - scanStartInd)*(j+1)/6
                int ep = (segInfo.startRingIndex[i] * (5 - j)    + segInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;

                //按曲率从小到大排序
                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

                //挑选每个分段的曲率很大和比较大的点
                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--) {
                    int ind = cloudSmoothness[k].ind; //曲率最大点的点序

                    //如果曲率大的非平面点，曲率的确比较大，并且未被筛选过滤掉
                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] > edgeThreshold &&
                        segInfo.segmentedCloudGroundFlag[ind] == false) {

                        largestPickedNum++;
                        if (largestPickedNum <= 2) {  //挑选曲率最大的前2个点放入sharp点集合
                            cloudLabel[ind] = 2;      //2代表点曲率很大
                            cornerPointsSharp->push_back(segmentedCloud->points[ind]);
                            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                        } else if (largestPickedNum <= 20) {  //挑选曲率最大的前20个点放入less sharp点集合
                            cloudLabel[ind] = 1;      //1代表点曲率比较大
                            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                        } else {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1; //筛选标志置位

                        //将曲率比较大的点的前后各5个连续距离比较近的点筛选出去，防止特征点聚集，使得特征点在每个方向上尽量分布均匀
                        for (int l = 1; l <= 5; l++) {
                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                //挑选每个分段的曲率很小比较小的点
                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++) {
                    int ind = cloudSmoothness[k].ind;
                    //如果曲率的确比较小，并且未被筛选出
                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] < surfThreshold &&
                        segInfo.segmentedCloudGroundFlag[ind] == true) {

                        cloudLabel[ind] = -1; //-1代表曲率很小的点
                        surfPointsFlat->push_back(segmentedCloud->points[ind]);

                        smallestPickedNum++;
                        if (smallestPickedNum >= 4) { //只选最小的四个，剩下的Label==0,就都是曲率比较小的
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {  //同样防止特征点聚集

                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {

                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                //将剩余的点（包括之前被排除的点）全部归入平面点中less flat类别中
                for (int k = sp; k <= ep; k++) {
                    if (cloudLabel[k] <= 0) {
                        surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
                    }
                }
            }

            //由于less flat点最多，对每个分段less flat的点进行体素栅格滤波
            surfPointsLessFlatScanDS->clear();
            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
            downSizeFilter.filter(*surfPointsLessFlatScanDS);

            //less flat点汇总
            *surfPointsLessFlat += *surfPointsLessFlatScanDS;
        }
    }

    //发布特征点云话题
    void publishCloud() {
        sensor_msgs::PointCloud2 laserCloudOutMsg;

      if (pubCornerPointsSharp.getNumSubscribers() != 0) {
          pcl::toROSMsg(*cornerPointsSharp, laserCloudOutMsg);
          laserCloudOutMsg.header.stamp = cloudHeader.stamp;
          laserCloudOutMsg.header.frame_id = "/camera";
          pubCornerPointsSharp.publish(laserCloudOutMsg);
      }

      if (pubCornerPointsLessSharp.getNumSubscribers() != 0) {
          pcl::toROSMsg(*cornerPointsLessSharp, laserCloudOutMsg);
          laserCloudOutMsg.header.stamp = cloudHeader.stamp;
          laserCloudOutMsg.header.frame_id = "/camera";
          pubCornerPointsLessSharp.publish(laserCloudOutMsg);
      }

      if (pubSurfPointsFlat.getNumSubscribers() != 0) {
          pcl::toROSMsg(*surfPointsFlat, laserCloudOutMsg);
          laserCloudOutMsg.header.stamp = cloudHeader.stamp;
          laserCloudOutMsg.header.frame_id = "/camera";
          pubSurfPointsFlat.publish(laserCloudOutMsg);
      }

      if (pubSurfPointsLessFlat.getNumSubscribers() != 0) {
//          for (auto i = 0; i < surfPointsLessFlat->size(); ++i) {
//              if (surfPointsLessFlat->points[i].intensity > 31.0)
//                  surfPointsLessFlat->points[i].y += 50.0;
//          }
          pcl::toROSMsg(*surfPointsLessFlat, laserCloudOutMsg);
          laserCloudOutMsg.header.stamp = cloudHeader.stamp;
          laserCloudOutMsg.header.frame_id = "/camera";
          pubSurfPointsLessFlat.publish(laserCloudOutMsg);
      }


      if (pubUndistortionCloud.getNumSubscribers() != 0) {
          pcl::toROSMsg(*undistortionCloud, laserCloudOutMsg);
          laserCloudOutMsg.header.stamp = cloudHeader.stamp;
          laserCloudOutMsg.header.frame_id = "/camera";
          pubUndistortionCloud.publish(laserCloudOutMsg);
      }

      //保存数据
//      if (saveDataForDebug && frameNum % 100 == 0) {
//          stringstream ss1, ss2;
//          ss1 << "/home/vance/matches/" << frameNum << "_segCloud.pcd";
//          ss2 << "/home/vance/matches/" << frameNum << "_undisCloud.pcd";

//          pcl::io::savePCDFileASCII(ss1.str(), *segmentedCloud);
//          pcl::io::savePCDFileASCII(ss2.str(), *undistortionCloud);

//          ROS_INFO("Save cloud data to %s", ss2.str().c_str());
//      }

    }

    //当前点云中的点相对第一个点去除因匀速运动产生的畸变，效果相当于得到在点云扫描开始位置静止扫描得到的点云
    void TransformToStart(PointType const * const pi, PointType * const po) {
        //插值系数计算，云中每个点的相对时间/点云周期10
        float s = 10 * (pi->intensity - int(pi->intensity));

        //线性插值：根据每个点在点云中的相对位置关系，乘以相应的旋转平移系数
        float rx = s * transformCur[0];
        float ry = s * transformCur[1];
        float rz = s * transformCur[2];
        float tx = s * transformCur[3];
        float ty = s * transformCur[4];
        float tz = s * transformCur[5];

        //平移后绕z轴旋转（-rz）
        float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
        float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
        float z1 = (pi->z - tz);

        //绕x轴旋转（-rx）
        float x2 = x1;
        float y2 = cos(rx) * y1 + sin(rx) * z1;
        float z2 = -sin(rx) * y1 + cos(rx) * z1;

        //绕y轴旋转（-ry）
        po->x = cos(ry) * x2 - sin(ry) * z2;
        po->y = y2;
        po->z = sin(ry) * x2 + cos(ry) * z2;
        po->intensity = pi->intensity;
    }

    //将上一帧点云中的点相对结束位置去除因匀速运动产生的畸变，效果相当于得到在点云扫描结束位置静止扫描得到的点云
    void TransformToEnd(PointType const * const pi, PointType * const po) {
        //插值系数计算
        float s = 10 * (pi->intensity - int(pi->intensity));

        float rx = s * transformCur[0];
        float ry = s * transformCur[1];
        float rz = s * transformCur[2];
        float tx = s * transformCur[3];
        float ty = s * transformCur[4];
        float tz = s * transformCur[5];

        //平移后绕z轴旋转（-rz）
        float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
        float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
        float z1 = (pi->z - tz);

        //绕x轴旋转（-rx）
        float x2 = x1;
        float y2 = cos(rx) * y1 + sin(rx) * z1;
        float z2 = -sin(rx) * y1 + cos(rx) * z1;

        //绕y轴旋转（-ry）
        float x3 = cos(ry) * x2 - sin(ry) * z2;
        float y3 = y2;
        float z3 = sin(ry) * x2 + cos(ry) * z2; //求出了相对于起始点校正的坐标

        rx = transformCur[0];
        ry = transformCur[1];
        rz = transformCur[2];
        tx = transformCur[3];
        ty = transformCur[4];
        tz = transformCur[5];

        //绕y轴旋转（ry）
        float x4 = cos(ry) * x3 + sin(ry) * z3;
        float y4 = y3;
        float z4 = -sin(ry) * x3 + cos(ry) * z3;

        //绕x轴旋转（rx）
        float x5 = x4;
        float y5 = cos(rx) * y4 - sin(rx) * z4;
        float z5 = sin(rx) * y4 + cos(rx) * z4;

        //绕z轴旋转（rz），再平移
        float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
        float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
        float z6 = z5 + tz;

        //平移后绕z轴旋转（imuRollStart）
        float x7 = cosImuRollStart * (x6 - imuShiftFromStartX)
                 - sinImuRollStart * (y6 - imuShiftFromStartY);
        float y7 = sinImuRollStart * (x6 - imuShiftFromStartX)
                 + cosImuRollStart * (y6 - imuShiftFromStartY);
        float z7 = z6 - imuShiftFromStartZ;

        //绕x轴旋转（imuPitchStart）
        float x8 = x7;
        float y8 = cosImuPitchStart * y7 - sinImuPitchStart * z7;
        float z8 = sinImuPitchStart * y7 + cosImuPitchStart * z7;

        //绕y轴旋转（imuYawStart）
        float x9 = cosImuYawStart * x8 + sinImuYawStart * z8;
        float y9 = y8;
        float z9 = -sinImuYawStart * x8 + cosImuYawStart * z8;

        //绕y轴旋转（-imuYawLast）
        float x10 = cos(imuYawLast) * x9 - sin(imuYawLast) * z9;
        float y10 = y9;
        float z10 = sin(imuYawLast) * x9 + cos(imuYawLast) * z9;

        //绕x轴旋转（-imuPitchLast）
        float x11 = x10;
        float y11 = cos(imuPitchLast) * y10 + sin(imuPitchLast) * z10;
        float z11 = -sin(imuPitchLast) * y10 + cos(imuPitchLast) * z10;

        //绕z轴旋转（-imuRollLast）
        po->x = cos(imuRollLast) * x11 + sin(imuRollLast) * y11;
        po->y = -sin(imuRollLast) * x11 + cos(imuRollLast) * y11;
        po->z = z11;
        po->intensity = int(pi->intensity); //只保留线号
    }

    //利用IMU修正旋转量，根据起始欧拉角，当前点云的欧拉角修正
    void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz,
                           float alx, float aly, float alz, float &acx, float &acy, float &acz) {
        float sbcx = sin(bcx);
        float cbcx = cos(bcx);
        float sbcy = sin(bcy);
        float cbcy = cos(bcy);
        float sbcz = sin(bcz);
        float cbcz = cos(bcz);

        float sblx = sin(blx);
        float cblx = cos(blx);
        float sbly = sin(bly);
        float cbly = cos(bly);
        float sblz = sin(blz);
        float cblz = cos(blz);

        float salx = sin(alx);
        float calx = cos(alx);
        float saly = sin(aly);
        float caly = cos(aly);
        float salz = sin(alz);
        float calz = cos(alz);

        float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly)
                  - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
                  - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
                  - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
                  - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
        acx = -asin(srx);

        float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
                     - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
                     - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
                     - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz)
                     + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
        float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
                     - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz)
                     - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
                     - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
                     + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
        acy = atan2(srycrx / cos(acx), crycrx / cos(acx));

        float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz)
                     - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx)
                     - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly)
                     + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx)
                     - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz
                     + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz)
                     + calx*cblx*salz*sblz);
        float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly)
                     - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx)
                     + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                     + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly)
                     + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly
                     - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz)
                     - calx*calz*cblx*sblz);
        acz = atan2(srzcrx / cos(acx), crzcrx / cos(acx));
    }

    //相对于第一帧积累旋转量，即相对世界坐标系的累计旋转
    void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz,
                            float &ox, float &oy, float &oz) {
        float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
        ox = -asin(srx);

        float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz)
                     + sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
        float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy)
                     - cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
        oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

        float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz)
                     + sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
        float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz)
                     - cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
        oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
    }

    double rad2deg(double radians) {
      return radians * 180.0 / M_PI;
    }

    double deg2rad(double degrees) {
      return degrees * M_PI / 180.0;
    }

    //寻找对应的边缘点
    void findCorrespondingCornerFeatures(int iterCount) {
        int cornerPointsSharpNum = cornerPointsSharp->points.size();

        //处理当前点云中的曲率最大的特征点,从上个点云中曲率比较大的特征点中找两个最近距离点，一个点使用kd-tree查找，另一个根据找到的点在其相邻线找另外一个最近距离的点
        for (int i = 0; i < cornerPointsSharpNum; i++) {
            TransformToStart(&cornerPointsSharp->points[i], &pointSel);

            //刚开始找一次，之后每迭代五次，重新查找最近点
            if (iterCount % 5 == 0) {
                //kd-tree查找一个最近距离点，边沿点未经过体素栅格滤波，一般边沿点本来就比较少，不做滤波
                kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                int closestPointInd = -1, minPointInd2 = -1;

                //寻找相邻线距离目标点距离最小的点
                //再次提醒：velodyne是2度一线，scanID相邻并不代表线号相邻，相邻线度数相差2度，也即线号scanID相差2
                if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) { //找到的最近点距离的确很近的话 (<25)
                    closestPointInd = pointSearchInd[0];
                    //提取最近点线号
                    int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

                    float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist;
                    //寻找距离目标点最近距离的平方和最小的点
                    for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) { //向scanID增大的方向查找
                        if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5) {  //非相邻线
                            break;  //找到与最邻近点相距3条线的特征点时跳出
                        }

                        //计算遍历点与最邻近点的距离(平方)
                        pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                     (laserCloudCornerLast->points[j].x - pointSel.x) +
                                     (laserCloudCornerLast->points[j].y - pointSel.y) *
                                     (laserCloudCornerLast->points[j].y - pointSel.y) +
                                     (laserCloudCornerLast->points[j].z - pointSel.z) *
                                     (laserCloudCornerLast->points[j].z - pointSel.z);

                        //确保两个点不在同一条scan上（相邻线查找应该可以用scanID == closestPointScan +/- 1 来做）
                        if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) {
                            //距离更近，要小于初始值5米
                            if (pointSqDis < minPointSqDis2) {
                                //更新最小距离与点序
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }
                    }

                    //同理，向scanID减小的方向查找(三条线)，找次临近点
                    for (int j = closestPointInd - 1; j >= 0; j--) {
                        if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) {
                            break;
                        }

                        pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                     (laserCloudCornerLast->points[j].x - pointSel.x) +
                                     (laserCloudCornerLast->points[j].y - pointSel.y) *
                                     (laserCloudCornerLast->points[j].y - pointSel.y) +
                                     (laserCloudCornerLast->points[j].z - pointSel.z) *
                                     (laserCloudCornerLast->points[j].z - pointSel.z);

                        if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }
                    }
                }

                //记住组成线的点序
                pointSearchCornerInd1[i] = closestPointInd; //kd-tree最近距离点，-1表示未找到满足的点
                pointSearchCornerInd2[i] = minPointInd2;    //另一个最近的，-1表示未找到满足的点
            }

            //大于等于0，不等于-1，说明两个点都找到了
            if (pointSearchCornerInd2[i] >= 0) {
                tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
                tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

                //选择的特征点记为O，kd-tree最近距离点记为A，另一个最近距离点记为B
                float x0 = pointSel.x;
                float y0 = pointSel.y;
                float z0 = pointSel.z;
                float x1 = tripod1.x;
                float y1 = tripod1.y;
                float z1 = tripod1.z;
                float x2 = tripod2.x;
                float y2 = tripod2.y;
                float z2 = tripod2.z;

                float m11 = ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1));
                float m22 = ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1));
                float m33 = ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1));

                //向量OA = (x0 - x1, y0 - y1, z0 - z1), 向量OB = (x0 - x2, y0 - y2, z0 - z2)，向量AB = （x1 - x2, y1 - y2, z1 - z2）
                //向量OA OB的向量积(即叉乘)为：
                //|  i      j      k  |
                //|x0-x1  y0-y1  z0-z1|
                //|x0-x2  y0-y2  z0-z2|
                //模为：
                float a012 = sqrt(m11 * m11  + m22 * m22 + m33 * m33);

                //两个最近距离点之间的距离，即向量AB的模
                float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                //AB方向的单位向量与OAB平面的单位法向量的向量积在各轴上的分量（d的方向）
                //x轴分量i
                float la =  ((y1 - y2)*m11 + (z1 - z2)*m22) / a012 / l12;

                //y轴分量j
                float lb = -((x1 - x2)*m11 - (z1 - z2)*m33) / a012 / l12;

                //z轴分量k
                float lc = -((x1 - x2)*m22 + (y1 - y2)*m33) / a012 / l12;

                //点到线的距离，d = |向量OA 叉乘 向量OB|/|AB|
                float ld2 = a012 / l12;

                //权重计算，距离越大权重越小，距离越小权重越大，得到的权重范围<=1
                float s = 1;
                if (iterCount >= 5) { //5次迭代之后开始增加权重因素
                    s = 1 - 1.8 * fabs(ld2);
                }

                //考虑权重
                //只保留权重大的，也即距离比较小的点，同时也舍弃距离为零的
                if (s > 0.1 && ld2 != 0) {
                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    laserCloudOri->push_back(cornerPointsSharp->points[i]);
                    coeffSel->push_back(coeff);
                }
            }
        }
    }

    //寻找对应的平面点
    void findCorrespondingSurfFeatures(int iterCount) {
        int surfPointsFlatNum = surfPointsFlat->points.size();

        //对本次接收到的曲率最小的点,从上次接收到的点云曲率比较小的点中找三点组成平面，一个使用kd-tree查找，另外一个在同一线上查找满足要求的，第三个在不同线上查找满足要求的
        for (int i = 0; i < surfPointsFlatNum; i++) {
            TransformToStart(&surfPointsFlat->points[i], &pointSel);

            if (iterCount % 5 == 0) { //每迭代5次重新查找一次匹配点
                //kd-tree最近点查找，在经过体素栅格滤波之后的平面点中查找，一般平面点太多，滤波后最近点查找数据量小
                kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

                if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) {
                    closestPointInd = pointSearchInd[0];
                    int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);

                    float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist, minPointSqDis3 = nearestFeatureSearchSqDist;
                    for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
                        if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5) {
                            break;
                        }

                        pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                     (laserCloudSurfLast->points[j].x - pointSel.x) +
                                     (laserCloudSurfLast->points[j].y - pointSel.y) *
                                     (laserCloudSurfLast->points[j].y - pointSel.y) +
                                     (laserCloudSurfLast->points[j].z - pointSel.z) *
                                     (laserCloudSurfLast->points[j].z - pointSel.z);

                        //如果点的线号小于等于最近点的线号(应该最多取等，也即同一线上的点)
                        if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                              minPointSqDis2 = pointSqDis;
                              minPointInd2 = j;
                            }
                        } else { //如果点处在大于该线上
                            if (pointSqDis < minPointSqDis3) {
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }
                    }
                    //同理
                    for (int j = closestPointInd - 1; j >= 0; j--) {
                        if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5) {
                            break;
                        }

                        pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                     (laserCloudSurfLast->points[j].x - pointSel.x) +
                                     (laserCloudSurfLast->points[j].y - pointSel.y) *
                                     (laserCloudSurfLast->points[j].y - pointSel.y) +
                                     (laserCloudSurfLast->points[j].z - pointSel.z) *
                                     (laserCloudSurfLast->points[j].z - pointSel.z);

                        if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        } else {
                            if (pointSqDis < minPointSqDis3) {
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }
                    }
                }

                pointSearchSurfInd1[i] = closestPointInd; //kd-tree最近距离点,-1表示未找到满足要求的点
                pointSearchSurfInd2[i] = minPointInd2;    //同一线号上的距离最近的点，-1表示未找到满足要求的点
                pointSearchSurfInd3[i] = minPointInd3;    //不同线号上的距离最近的点，-1表示未找到满足要求的点
            }

            if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) { //找到了三个点

                tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]]; //A点
                tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]]; //B点
                tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]]; //C点

                //向量AB = (tripod2.x - tripod1.x, tripod2.y - tripod1.y, tripod2.z - tripod1.z)
                //向量AC = (tripod3.x - tripod1.x, tripod3.y - tripod1.y, tripod3.z - tripod1.z)

                //向量AB AC的向量积（即叉乘），得到的是法向量
                //x轴方向分向量i
                float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
                         - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
                //y轴方向分向量j
                float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
                         - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
                //z轴方向分向量k
                float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
                         - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
                float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

                //法向量的模
                float ps = sqrt(pa * pa + pb * pb + pc * pc);

                //pa pb pc为法向量各方向上的单位向量
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                //点到面的距离：向量OA与与法向量的点积除以法向量的模
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                //同理计算权重(L-M法的阻尼因子)
                float s = 1;
                if (iterCount >= 5) {
                    s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                            + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
                }

                //考虑权重. 保存原始点与相应的系数
                if (s > 0.1 && pd2 != 0) {
                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    laserCloudOri->push_back(surfPointsFlat->points[i]);
                    // 每个特征点对应的Jaccobian矩阵的三个元素都保存在coeffSel中
                    coeffSel->push_back(coeff);
                }
            }
        }
    }

    //从平面对应点计算相对位姿变换
    bool calculateTransformationSurf(int iterCount) {
        int pointSelNum = laserCloudOri->points.size();

        //计算matA,matB矩阵
        cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

        //transformCur还是上一帧的值，这里利用了上一帧的变换做初值
        float srx = sin(transformCur[0]);
        float crx = cos(transformCur[0]);
        float sry = sin(transformCur[1]);
        float cry = cos(transformCur[1]);
        float srz = sin(transformCur[2]);
        float crz = cos(transformCur[2]);
        float tx = transformCur[3];
        float ty = transformCur[4];
        float tz = transformCur[5];

        float a1 = crx*sry*srz; float a2 = crx*crz*sry; float a3 = srx*sry; float a4 = tx*a1 - ty*a2 - tz*a3;
        float a5 = srx*srz; float a6 = crz*srx; float a7 = ty*a6 - tz*crx - tx*a5;
        float a8 = crx*cry*srz; float a9 = crx*cry*crz; float a10 = cry*srx; float a11 = tz*a10 + ty*a9 - tx*a8;

        float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz;
        float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry;

        float c1 = -b6; float c2 = b5; float c3 = tx*b6 - ty*b5; float c4 = -crx*crz; float c5 = crx*srz; float c6 = ty*c5 + tx*-c4;
        float c7 = b2; float c8 = -b1; float c9 = tx*-b2 - ty*-b1;

        for (int i = 0; i < pointSelNum; i++) {
            pointOri = laserCloudOri->points[i];
            coeff = coeffSel->points[i];

            float arx = (-a1*pointOri.x + a2*pointOri.y + a3*pointOri.z + a4) * coeff.x
                      + (a5*pointOri.x - a6*pointOri.y + crx*pointOri.z + a7) * coeff.y
                      + (a8*pointOri.x - a9*pointOri.y - a10*pointOri.z + a11) * coeff.z;

            float arz = (c1*pointOri.x + c2*pointOri.y + c3) * coeff.x
                      + (c4*pointOri.x - c5*pointOri.y + c6) * coeff.y
                      + (c7*pointOri.x + c8*pointOri.y + c9) * coeff.z;

            float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

            float d2 = coeff.intensity;

            matA.at<float>(i, 0) = arx;
            matA.at<float>(i, 1) = arz;
            matA.at<float>(i, 2) = aty;
            matB.at<float>(i, 0) = -0.05 * d2;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        //求解matAtA * matX = matAtB，最小二乘计算(QR分解法)
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            //特征值1*3矩阵
            cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
            //特征向量3*3矩阵
            cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

            //求解特征值/特征向量
            cv::eigen(matAtA, matE, matV);  //计算矩阵的特征向量E及特征向量的反对称阵V
            matV.copyTo(matV2);

            isDegenerate = false;
            //特征值取值门槛
            float eignThre[3] = {10, 10, 10};
            for (int i = 2; i >= 0; i--) {  //从小到大查找
                if (matE.at<float>(0, i) < eignThre[i]) { //特征值太小，则认为处在兼并环境中，发生了退化
                    for (int j = 0; j < 3; j++) { //对应的特征向量置为0
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            //计算P矩阵
            matP = matV.inv() * matV2;
        }

        if (isDegenerate) { //如果发生退化，只使用预测矩阵P计算
            cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        //累加每次迭代的旋转平移量
        //这里的变换看似是Rx，Rz, ty,实际上在畸变矫正环节已经把点云的坐标系改成相机坐标系模式了
        //所以实际上这里是利用平面特征点计算R_y(pitch), R_x(roll)和t_z
        transformCur[0] += matX.at<float>(0, 0);
        transformCur[2] += matX.at<float>(1, 0);
        transformCur[4] += matX.at<float>(2, 0);

        for(int i=0; i<6; i++) {
            if(isnan(transformCur[i])) //判断是否非数字
                transformCur[i]=0;
        }

        //计算旋转平移量，如果很小就停止迭代
        float deltaR = sqrt(
                            pow(rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(rad2deg(matX.at<float>(1, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(2, 0) * 100, 2));

        //迭代终止条件
        if (deltaR < 0.1 && deltaT < 0.1) {
            return false;
        }
        return true;
    }

    //从边缘对应点计算相对位姿变换
    bool calculateTransformationCorner(int iterCount) {
        int pointSelNum = laserCloudOri->points.size();

        cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

        float srx = sin(transformCur[0]);
        float crx = cos(transformCur[0]);
        float sry = sin(transformCur[1]);
        float cry = cos(transformCur[1]);
        float srz = sin(transformCur[2]);
        float crz = cos(transformCur[2]);
        float tx = transformCur[3];
        float ty = transformCur[4];
        float tz = transformCur[5];

        float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = tx*-b1 + ty*-b2 + tz*b3;
        float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tz*b7 - ty*b6 - tx*b5;

        float c5 = crx*srz;

        for (int i = 0; i < pointSelNum; i++) {

            pointOri = laserCloudOri->points[i];
            coeff = coeffSel->points[i];

            float ary = (b1*pointOri.x + b2*pointOri.y - b3*pointOri.z + b4) * coeff.x
                      + (b5*pointOri.x + b6*pointOri.y - b7*pointOri.z + b8) * coeff.z;

            float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

            float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

            float d2 = coeff.intensity;

            matA.at<float>(i, 0) = ary;
            matA.at<float>(i, 1) = atx;
            matA.at<float>(i, 2) = atz;
            matB.at<float>(i, 0) = -0.05 * d2;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[3] = {10, 10, 10};
            for (int i = 2; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 3; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate) {
            cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        //累加每次迭代的旋转平移量
        //这里的变换看似是Ry，tx, tz,实际上在畸变矫正环节已经把点云的坐标系改成相机坐标系模式了
        //所以实际上这里是利用边缘特征点计算R_z(yaw), t_y和t_x
        transformCur[1] += matX.at<float>(0, 0);
        transformCur[3] += matX.at<float>(1, 0);
        transformCur[5] += matX.at<float>(2, 0);

        for(int i=0; i<6; i++) {
            if(isnan(transformCur[i]))
                transformCur[i]=0;
        }

        float deltaR = sqrt(
                            pow(rad2deg(matX.at<float>(0, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(1, 0) * 100, 2) +
                            pow(matX.at<float>(2, 0) * 100, 2));

        if (deltaR < 0.1 && deltaT < 0.1) {
            return false;
        }
        return true;
    }

    //L-M法迭代计算位姿变换，这是之前未解耦的算法
    //这里用calculateTransformationSurf和calculateTransformationCorner代替
    bool calculateTransformation(int iterCount) {
        int pointSelNum = laserCloudOri->points.size();

        //计算matA,matB矩阵
        cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        float srx = sin(transformCur[0]);
        float crx = cos(transformCur[0]);
        float sry = sin(transformCur[1]);
        float cry = cos(transformCur[1]);
        float srz = sin(transformCur[2]);
        float crz = cos(transformCur[2]);
        float tx = transformCur[3];
        float ty = transformCur[4];
        float tz = transformCur[5];

        float a1 = crx*sry*srz; float a2 = crx*crz*sry; float a3 = srx*sry; float a4 = tx*a1 - ty*a2 - tz*a3;
        float a5 = srx*srz; float a6 = crz*srx; float a7 = ty*a6 - tz*crx - tx*a5;
        float a8 = crx*cry*srz; float a9 = crx*cry*crz; float a10 = cry*srx; float a11 = tz*a10 + ty*a9 - tx*a8;

        float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = tx*-b1 + ty*-b2 + tz*b3;
        float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tz*b7 - ty*b6 - tx*b5;

        float c1 = -b6; float c2 = b5; float c3 = tx*b6 - ty*b5; float c4 = -crx*crz; float c5 = crx*srz; float c6 = ty*c5 + tx*-c4;
        float c7 = b2; float c8 = -b1; float c9 = tx*-b2 - ty*-b1;

        for (int i = 0; i < pointSelNum; i++) {
            pointOri = laserCloudOri->points[i];
            coeff = coeffSel->points[i];

            float arx = (-a1*pointOri.x + a2*pointOri.y + a3*pointOri.z + a4) * coeff.x
                      + (a5*pointOri.x - a6*pointOri.y + crx*pointOri.z + a7) * coeff.y
                      + (a8*pointOri.x - a9*pointOri.y - a10*pointOri.z + a11) * coeff.z;

            float ary = (b1*pointOri.x + b2*pointOri.y - b3*pointOri.z + b4) * coeff.x
                      + (b5*pointOri.x + b6*pointOri.y - b7*pointOri.z + b8) * coeff.z;

            float arz = (c1*pointOri.x + c2*pointOri.y + c3) * coeff.x
                      + (c4*pointOri.x - c5*pointOri.y + c6) * coeff.y
                      + (c7*pointOri.x + c8*pointOri.y + c9) * coeff.z;

            float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

            float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

            float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

            float d2 = coeff.intensity;

            matA.at<float>(i, 0) = arx;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arz;
            matA.at<float>(i, 3) = atx;
            matA.at<float>(i, 4) = aty;
            matA.at<float>(i, 5) = atz;
            matB.at<float>(i, 0) = -0.05 * d2;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        //求解(A^T)A*X=(A^T)B
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            //特征值1*6矩阵
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            //特征向量6*6矩阵
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            //求解特征值/特征向量
            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            //特征值取值门槛
            float eignThre[6] = {10, 10, 10, 10, 10, 10};
            for (int i = 5; i >= 0; i--) { //从小到大查找
                if (matE.at<float>(0, i) < eignThre[i]) { //特征值太小，则认为处在兼并环境中，发生了退化
                    for (int j = 0; j < 6; j++) { //对应的特征向量置为0
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            //计算P矩阵
            matP = matV.inv() * matV2;
        }

        if (isDegenerate) { //如果发生退化，只使用预测矩阵P计算
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        //累加每次迭代的旋转平移量
        transformCur[0] += matX.at<float>(0, 0);
        transformCur[1] += matX.at<float>(1, 0);
        transformCur[2] += matX.at<float>(2, 0);
        transformCur[3] += matX.at<float>(3, 0);
        transformCur[4] += matX.at<float>(4, 0);
        transformCur[5] += matX.at<float>(5, 0);

        for(int i=0; i<6; i++) {
            if(isnan(transformCur[i])) //判断是否非数字
                transformCur[i]=0;
        }

        //计算旋转平移量，如果很小就停止迭代
        float deltaR = sqrt(
                            pow(rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        //迭代终止条件
        if (deltaR < 0.1 && deltaT < 0.1) {
            return false;
        }
        return true;
    }

    //LM法执行前的初始化工作
    void checkSystemInitialization() {
        //将cornerPointsLessSharp与laserCloudCornerLast交换,目的保存cornerPointsLessSharp的值下轮使用
        pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTemp;

        //将surfPointLessFlat与laserCloudSurfLast交换，目的保存surfPointsLessFlat的值下轮使用
        laserCloudTemp = surfPointsLessFlat;
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTemp;

        //使用上一帧的特征点构建kd-tree
        kdtreeCornerLast->setInputCloud(laserCloudCornerLast);  //所有的边沿点集合
        kdtreeSurfLast->setInputCloud(laserCloudSurfLast);      //所有的平面点集合

        laserCloudCornerLastNum = laserCloudCornerLast->points.size();
        laserCloudSurfLastNum = laserCloudSurfLast->points.size();

        //将cornerPointsLessSharp和surfPointLessFlat点也即边沿点和平面点分别发送给laserMapping
        sensor_msgs::PointCloud2 laserCloudCornerLast2;
        pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
        laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
        laserCloudCornerLast2.header.frame_id = "/camera";
        pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

        sensor_msgs::PointCloud2 laserCloudSurfLast2;
        pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
        laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
        laserCloudSurfLast2.header.frame_id = "/camera";
        pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

        //记住原点的翻滚角和俯仰角
        transformSum[0] += imuPitchStart;
        transformSum[2] += imuRollStart;

        systemInitedLM = true;
    }

    //将当前时刻的IMU数据作为初始估计
    void updateInitialGuess() {
        imuPitchLast = imuPitchCur;
        imuYawLast = imuYawCur;
        imuRollLast = imuRollCur;

        imuShiftFromStartX = imuShiftFromStartXCur;
        imuShiftFromStartY = imuShiftFromStartYCur;
        imuShiftFromStartZ = imuShiftFromStartZCur;

        imuVeloFromStartX = imuVeloFromStartXCur;
        imuVeloFromStartY = imuVeloFromStartYCur;
        imuVeloFromStartZ = imuVeloFromStartZCur;

        if (imuAngularFromStartX != 0 || imuAngularFromStartY != 0 || imuAngularFromStartZ != 0) {
            transformCur[0] = - imuAngularFromStartY;
            transformCur[1] = - imuAngularFromStartZ;
            transformCur[2] = - imuAngularFromStartX;
        }

        //T平移量的初值赋值为加减速的位移量，为其梯度下降的方向（沿用上次转换的T（一个sweep匀速模型），同时在其基础上减去匀速运动位移，即只考虑加减速的位移量）
        if (imuVeloFromStartX != 0 || imuVeloFromStartY != 0 || imuVeloFromStartZ != 0) {
            transformCur[3] -= imuVeloFromStartX * scanPeriod;
            transformCur[4] -= imuVeloFromStartY * scanPeriod;
            transformCur[5] -= imuVeloFromStartZ * scanPeriod;
        }
    }

    //计算相对位姿变换入口
    //Levenberg-Marquardt算法(L-M method)，非线性最小二乘算法，最优化算法的一种.这里最多迭代25次
    void updateTransformation() {
        //边缘点数量小于10，或平面点数量小于100则跳过
        if (laserCloudCornerLastNum < 10 || laserCloudSurfLastNum < 100) {
            ROS_WARN("Too less features for update transformation...");
            return;
        }

        //由平面点迭代计算位姿变换
        for (int iterCount1 = 0; iterCount1 < 25; iterCount1++) {
            laserCloudOri->clear(); //存放平面特征点，每次迭代前会清空
            coeffSel->clear();      //存放平面特征点的权重值，每次迭代前会清空

            findCorrespondingSurfFeatures(iterCount1);

            if (laserCloudOri->points.size() < 10)
                continue;
            if (calculateTransformationSurf(iterCount1) == false)
                break;
        }

        //由边缘点迭代计算位姿变换
        for (int iterCount2 = 0; iterCount2 < 25; iterCount2++) {
            laserCloudOri->clear(); //存放边缘特征点，每次迭代前会清空
            coeffSel->clear();      //存放边缘特征点的权重值，每次迭代前会清空

            findCorrespondingCornerFeatures(iterCount2);

            if (laserCloudOri->points.size() < 10)
                continue;
            if (calculateTransformationCorner(iterCount2) == false)
                break;
        }
    }

    //对两种位姿做积分,求累计变换（全局位姿）
    void integrateTransformation() {
        float rx, ry, rz, tx, ty, tz;
        //求相对于原点的旋转量
        AccumulateRotation(transformSum[0], transformSum[1], transformSum[2],
                           -transformCur[0], -transformCur[1], -transformCur[2], rx, ry, rz);

        float x1 = cos(rz) * (transformCur[3] - imuShiftFromStartX)
                 - sin(rz) * (transformCur[4] - imuShiftFromStartY);
        float y1 = sin(rz) * (transformCur[3] - imuShiftFromStartX)
                 + cos(rz) * (transformCur[4] - imuShiftFromStartY);
        float z1 = transformCur[5] - imuShiftFromStartZ;

        float x2 = x1;
        float y2 = cos(rx) * y1 - sin(rx) * z1;
        float z2 = sin(rx) * y1 + cos(rx) * z1;

        //求相对于原点的平移量
        tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
        ty = transformSum[4] - y2;
        tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

        //根据IMU修正旋转量
        PluginIMURotation(rx, ry, rz, imuPitchStart, imuYawStart, imuRollStart,
                          imuPitchLast, imuYawLast, imuRollLast, rx, ry, rz);

        //得到世界坐标系下的变换矩阵
        transformSum[0] = rx;
        transformSum[1] = ry;
        transformSum[2] = rz;
        transformSum[3] = tx;
        transformSum[4] = ty;
        transformSum[5] = tz;
    }

    //发布帧间匹配算出的位姿变换
    void publishOdometry() {
        //欧拉角转换成四元数，这里把Roll，Pitch，Yaw顺序调回来了
        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum[2], -transformSum[0], -transformSum[1]);

        //publish四元数和平移量，又以相机坐标系表示
        laserOdometry.header.stamp = cloudHeader.stamp;
        laserOdometry.pose.pose.orientation.x = -geoQuat.y;
        laserOdometry.pose.pose.orientation.y = -geoQuat.z;
        laserOdometry.pose.pose.orientation.z = geoQuat.x;
        laserOdometry.pose.pose.orientation.w = geoQuat.w;
        laserOdometry.pose.pose.position.x = transformSum[3];
        laserOdometry.pose.pose.position.y = transformSum[4];
        laserOdometry.pose.pose.position.z = transformSum[5];
        pubLaserOdometry.publish(laserOdometry);

        //广播新的平移旋转之后的坐标系(rviz)
        laserOdometryTrans.stamp_ = cloudHeader.stamp;
        laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        laserOdometryTrans.setOrigin(tf::Vector3(transformSum[3], transformSum[4], transformSum[5]));
        tfBroadcaster.sendTransform(laserOdometryTrans);
    }

    //调整离群点云的坐标系，变回原始的激光坐标系下
    void adjustOutlierCloud() {
        PointType point;
        int cloudSize = outlierCloud->points.size();
        for (int i = 0; i < cloudSize; ++i) {
            point.x = outlierCloud->points[i].y;
            point.y = outlierCloud->points[i].z;
            point.z = outlierCloud->points[i].x;
            point.intensity = outlierCloud->points[i].intensity;
            outlierCloud->points[i] = point;
        }
    }

    //发布最终的特征点云
    void publishCloudsLast() {
        updateImuRollPitchYawStartSinCos();

        //对点云的曲率比较大和比较小的点投影到扫描结束位置
        int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
        for (int i = 0; i < cornerPointsLessSharpNum; i++) {
            TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
        }

        int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
        for (int i = 0; i < surfPointsLessFlatNum; i++) {
            TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
        }

        //投影到扫描结束位置的点作为last点保存等下个点云进来进行匹配
        //这样下一帧的点投影到扫描开始位置，两帧点的坐标系就一致了
        pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTemp;

        laserCloudTemp = surfPointsLessFlat;
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTemp;

        laserCloudCornerLastNum = laserCloudCornerLast->points.size();
        laserCloudSurfLastNum = laserCloudSurfLast->points.size();

        //点足够多就构建kd-tree，否则弃用此帧，沿用上一帧数据的kd-tree
        if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
        }

        frameCount++;

        //按照跳帧数publich边沿点，平面点以及全部点给laserMapping(每隔一帧发一次)
        //每间隔一个点云数据相对点云最后一个点进行畸变校正
        if (frameCount >= skipFrameNum + 1) {
            frameCount = 0;

            adjustOutlierCloud();
            sensor_msgs::PointCloud2 outlierCloudLast2;
            pcl::toROSMsg(*outlierCloud, outlierCloudLast2);
            outlierCloudLast2.header.stamp = cloudHeader.stamp;
            outlierCloudLast2.header.frame_id = "/camera";
            pubOutlierCloudLast.publish(outlierCloudLast2);

            sensor_msgs::PointCloud2 laserCloudCornerLast2;
            pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
            laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
            laserCloudCornerLast2.header.frame_id = "/camera";
            pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

            sensor_msgs::PointCloud2 laserCloudSurfLast2;
            pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
            laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
            laserCloudSurfLast2.header.frame_id = "/camera";
            pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
        }
    }

    //特征匹配和位姿计算主函数
    void runFeatureAssociation() {
        if (newSegmentedCloud && newSegmentedCloudInfo && newOutlierCloud &&
            std::abs(timeNewSegmentedCloudInfo - timeNewSegmentedCloud) < 0.05 &&
            std::abs(timeNewOutlierCloud - timeNewSegmentedCloud) < 0.05) {

            newSegmentedCloud = false;
            newSegmentedCloudInfo = false;
            newOutlierCloud = false;
        } else {
            return;
        }

        frameNum++;
        auto t1 = std::chrono::steady_clock::now();
        adjustDistortion();

        calculateSmoothness();

        markOccludedPoints();

        extractFeatures();

        publishCloud();

        if (!systemInitedLM) {
            checkSystemInitialization();
            return;
        }

        updateInitialGuess();

        updateTransformation();

        integrateTransformation();

        // for debug
        auto t2 = std::chrono::steady_clock::now();
        if (saveDataForDebug) {
            double dt = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            tatalRunTime += dt;

            if (frameNum % 50 == 0)
                ROS_INFO("[FeatureAssociat]Frame %d time cost: %f, Averange time cost: %f", frameNum, dt, tatalRunTime/(double)frameNum);
        }

        publishOdometry();

        publishCloudsLast();
    }
};




int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m Feature Association Started.");

    FeatureAssociation FA;

    ros::Rate rate(200);
    while (ros::ok()) {
        ros::spinOnce();

        FA.runFeatureAssociation();

        rate.sleep();
    }

    ros::spin();
    return 0;
}
