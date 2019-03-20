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

#include "utility.h"
#include <opencv2/highgui/highgui.hpp>

class ImageProjection{
private:
    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;

    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubOutlierCloud;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;   //输入点云

    pcl::PointCloud<PointType>::Ptr fullCloud;      //理论上具有完全分辨率的点云，实际上存在很多无效点
    pcl::PointCloud<PointType>::Ptr fullInfoCloud;  //就存了fullCloud中点的深度值信息

    pcl::PointCloud<PointType>::Ptr groundCloud;    //代表平面的点云
    pcl::PointCloud<PointType>::Ptr segmentedCloud; //分割出来的点云,包括有效聚类点和部分平面点
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure; //纯有效聚类点,不包含平面点
    pcl::PointCloud<PointType>::Ptr outlierCloud;   //离群点

    PointType nanPoint;

    //标签的含义：0为初始值，指未分类；-1指无效点，包括离群点和地面点；
    //1以上是聚类的序号，相同标签指同一个聚类；999999指聚类不够的点，视为离群点
    cv::Mat rangeMat;   //存储点的距离range，初始化为FLT_MAX
    cv::Mat labelMat;   //存储点的分类标签，初始化为0
    cv::Mat groundMat;  //地面点标记矩阵，初始化为0
    int labelCount;     //聚类标签序号，初始化为1，最后值为多少，说明有多少个聚类

    float startOrientation; //当前帧点云的起始角度
    float endOrientation;   //当前帧点云的结束角度

    cloud_msgs::cloud_info segMsg;  //分割出来的点云的部分信息
    std_msgs::Header cloudHeader;

    //以下是分割中邻域搜索要用的变量
    std::vector<std::pair<uint8_t, uint8_t> > neighborIterator;

    uint16_t *allPushedIndX;
    uint16_t *allPushedIndY;

    uint16_t *queueIndX;    //索引队列
    uint16_t *queueIndY;

    int frameNum = 0;
    double tatalRunTime = 0.0;

public:
    ImageProjection() : nh("~") {
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &ImageProjection::cloudHandler, this);

        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info> ("/segmented_cloud_info", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        resetParameters();
    }

    void allocateMemory() {
        laserCloudIn.reset(new pcl::PointCloud<PointType>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

        segMsg.startRingIndex.assign(N_SCAN, 0);
        segMsg.endRingIndex.assign(N_SCAN, 0);

        segMsg.segmentedCloudGroundFlag.assign(N_SCAN*Horizon_SCAN, false);
        segMsg.segmentedCloudColInd.assign(N_SCAN*Horizon_SCAN, 0);
        segMsg.segmentedCloudRange.assign(N_SCAN*Horizon_SCAN, 0);

        //上下左右四个近邻的索引差值
        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
    }

    void resetParameters() {
        laserCloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }

    ~ImageProjection() {}

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
        cloudHeader = laserCloudMsg->header;
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
    }

    void saveImagePointcloud() {
        //save image
        cv::Mat rangeMatDisplay = cv::Mat(N_SCAN, Horizon_SCAN, CV_8UC1, cv::Scalar::all(0));;
        for (int i = 0; i < N_SCAN; ++i) {
            for (int j = 0; j < Horizon_SCAN; ++j) {
                double p = rangeMat.at<double>(i,j);
                if (p < 0.5 ) p = 0.0;
                else if (p > 150.0) p = 255.0;
                else p *= 255.0 / 150.0;
                rangeMatDisplay.at<uchar>(i,j) = static_cast<uchar>(p);
            }
        }
        if (cv::imwrite("/home/vance/test_frame_50_image.jpg", rangeMatDisplay)) {
//            cv::imshow("Display", rangeMatDisplay);
//            cv::waitKey(200);
            ROS_INFO("write image successed.");
        }
        else
            ROS_ERROR("write image failed.");

        //save pointcloud
        fullCloud->width = 1;
        fullCloud->height = fullCloud->points.size();
        groundCloud->width = 1;
        groundCloud->height = groundCloud->points.size();
        segmentedCloud->width = 1;
        segmentedCloud->height = segmentedCloud->points.size();
        outlierCloud->width = 1;
        outlierCloud->height = outlierCloud->points.size();
        segmentedCloudPure->width = 1;
        segmentedCloudPure->height = segmentedCloudPure->points.size();
        pcl::io::savePCDFile("/home/vance/test_frame_50_full_cloud.pcd", *fullCloud);
        pcl::io::savePCDFile("/home/vance/test_frame_50_ground_cloud.pcd", *groundCloud);
        pcl::io::savePCDFile("/home/vance/test_frame_50_segmented_cloud.pcd", *segmentedCloud);
        pcl::io::savePCDFile("/home/vance/test_frame_50_outlier_cloud.pcd", *outlierCloud);
        pcl::io::savePCDFile("/home/vance/test_frame_50_segmentedCloudPure.pcd", *segmentedCloudPure);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
        frameNum++;
        auto t1 = std::chrono::steady_clock::now();

        copyPointCloud(laserCloudMsg);
        findStartEndAngle();
        projectPointCloud();
        groundRemoval();
        cloudSegmentation();

        auto t2 = std::chrono::steady_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        if (saveDataForDebug) {
            tatalRunTime += dt;
            if (frameNum % 50 == 0) {
                ROS_INFO("[ImageProjection]Frame %d time cost: %f, Averange time cost: %f", frameNum, dt, tatalRunTime/(double)frameNum);

            if (frameNum == 50)
                saveImagePointcloud();
            }
        }

        publishCloud();
        resetParameters();
    }

    //找到当前帧点云的起始和终止角度
    void findStartEndAngle() {
        //因为velodyne雷达是顺时针旋转的，所以角度要加上负号
        segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        segMsg.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                         laserCloudIn->points[laserCloudIn->points.size() - 2].x) + 2 * M_PI;
        if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
            segMsg.endOrientation -= 2 * M_PI;
        } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
            segMsg.endOrientation += 2 * M_PI;
        segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
    }

    //将点云按照线数和分辨率投影到一个64*1800的平面上
    void projectPointCloud() {
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize;
        PointType thisPoint;

        cloudSize = laserCloudIn->points.size();
        for (size_t i = 0; i < cloudSize; ++i) {
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;

            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI; //与水平面夹角
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y;      //对应像素的行标号
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI; //水平角度
            //0是Lidar坐标系-y轴方向，即-90度处为0，+90度处是900，0度处是1350，沿着逆时针计算列号
            columnIdn = -round((horizonAngle - 90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            rangeMat.at<float>(rowIdn, columnIdn) = range;          //像素值

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0; //整数部分为行号，小数部分为列号

            index = columnIdn  + rowIdn * Horizon_SCAN;             //像素标号

            fullCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range;         //点的距离存在fullInfoCloud的intensity里
        }
    }

    //去除平面
    void groundRemoval() {
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;

        //同一列不同行的点之间的垂直夹角小于10°即定为平面点。
        for (size_t j = 0; j < Horizon_SCAN; ++j) {
            for (size_t i = 0; i < groundScanInd; ++i) {
                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;

                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1) {
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }

                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

                if (abs(angle - sensorMountAngle) <= 10) {
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }

        for (size_t i = 0; i < N_SCAN; ++i) {
            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX) {
                    labelMat.at<int>(i,j) = -1; //-1指地面点和无限远点（无效点），即可以去除的点
                }
            }
        }
        if (pubGroundCloud.getNumSubscribers() != 0 || saveDataForDebug) {
            for (size_t i = 0; i <= groundScanInd; ++i) {
                for (size_t j = 0; j < Horizon_SCAN; ++j) {
                    if (groundMat.at<int8_t>(i,j) == 1)
                        groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                }
            }
        }
    }

    //点云分割，分割出来到下一阶段使用的点有：
    //1、全部有效聚类点(标签>=1且!=999999)；
    //2、部分地面点(groundMat.at(i,j)==1)中的头5列、尾5列和逢5那一列的；
    //部分无效聚类点中会被当做离群点(线数不代表地面的，且逢5那一列的)
    void cloudSegmentation() {
        //对所有点进行聚类，打上标签
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
                if (labelMat.at<int>(i,j) == 0)
                    labelComponents(i, j);

        int sizeOfSegCloud = 0;
        for (size_t i = 0; i < N_SCAN; ++i) {
            segMsg.startRingIndex[i] = sizeOfSegCloud - 1 + 5;  //每一线起始点的索引，去掉前5个点

            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1) {
                    if (labelMat.at<int>(i,j) == 999999) {
                        if (i > groundScanInd && j % 5 == 0) {
                            outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                            continue;
                        } else {
                            continue;
                        }
                    }
                    if (groundMat.at<int8_t>(i,j) == 1) {
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                            continue;
                    }
                    segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                    segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                    segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    ++sizeOfSegCloud;
                }
            }

            segMsg.endRingIndex[i] = sizeOfSegCloud - 1 - 5;
        }

        if (pubSegmentedCloudPure.getNumSubscribers() != 0 || saveDataForDebug) {
            for (size_t i = 0; i < N_SCAN; ++i) {
                for (size_t j = 0; j < Horizon_SCAN; ++j) {
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999) {
                        segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                    }
                }
            }
        }
    }

    //点云聚类打标签
    void labelComponents(int row, int col) {
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY;
        bool lineCountFlag[N_SCAN] = {false};

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;

        //queueSize指的是在特征处理时还未处理好的点的数量，
        //因此该while循环是在尝试检测该特定点的周围的点的几何特征
        while(queueSize > 0) {
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;

            //检查上下左右四个邻点
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter) {
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;

                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;

                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;

                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                //d1与d2分别是该特定点与某邻点的深度
                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY),
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY),
                              rangeMat.at<float>(thisIndX, thisIndY));

                //该迭代器的first是0则是水平方向上的邻点，否则是竖直方向上的
                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

                //这个angle其实是该特定点与某邻点的连线与XOZ平面的夹角，这个夹角代表了局部特征的敏感性
                angle = atan2(d2*sin(alpha), (d1 - d2*cos(alpha)));

                //如果夹角大于pi/3，则将这个邻点纳入到局部特征中，该邻点可以用来配准使用
                if (angle > segmentTheta) {
                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }


        bool feasibleSegment = false;
        //当邻点数目达到30后，则该帧雷达点云的几何特征配置成功
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
        else if (allPushedIndSize >= segmentValidPointNum) {
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;
        }

        if (feasibleSegment == true) {
            ++labelCount;   //有效聚类，标签号加1
        } else {
            for (size_t i = 0; i < allPushedIndSize; ++i) {
                //无效聚类，全部打上999999
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }


    void publishCloud() {
        segMsg.header = cloudHeader;
        pubSegmentedCloudInfo.publish(segMsg);

        sensor_msgs::PointCloud2 laserCloudTemp;

        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubOutlierCloud.publish(laserCloudTemp);

        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubSegmentedCloud.publish(laserCloudTemp);

        if (pubFullCloud.getNumSubscribers() != 0) {
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullCloud.publish(laserCloudTemp);
        }

        if (pubGroundCloud.getNumSubscribers() != 0) {
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubGroundCloud.publish(laserCloudTemp);
        }

        if (pubSegmentedCloudPure.getNumSubscribers() != 0) {
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }

        if (pubFullInfoCloud.getNumSubscribers() != 0) {
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullInfoCloud.publish(laserCloudTemp);
        }
    }
};




int main(int argc, char** argv) {

    ros::init(argc, argv, "lego_loam");

    ImageProjection IP;

    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    ros::spin();
    return 0;
}
