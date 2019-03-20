#include "utility.h"
#include <cv_bridge/cv_bridge.h>
#include <fast/fast.h>
#include <opencv2/features2d/features2d.hpp>

class SimpleVO {
private:
    ros::NodeHandle nh;
    ros::Subscriber subImageMsg;
    ros::Subscriber subImageDepthMsg;
    ros::Publisher pubVisualOdom;
    ros::Publisher pubImageShow;

    std_msgs::Header imageHeader;
    std_msgs::Header imageDepthHeader;

    cv_bridge::CvImage bridge;
    cv::Mat imageCurr, imageLast, imageShow, imageDepthCurr, imageDepthLast, fastLast;
    cv::Mat kMat, dMat; // 相机参数矩阵
    cv::Mat imageEig, imageTmp /*pyrCur, pyrLast*/; // 图像金字塔
    std::vector<cv::Mat> imagePyrCurr, imagePyrLast;

    bool firstFrame, systemReady;
    bool newImageMsg, newImageDepthMsg;
    double timeImageCurr, timeImageDepthCurr;

    float transformCur[6];
    std::vector<float*> transforms;

    const int nPyrLevels = 8;
    const int iniThFAST = 20;
    const int minThFAST = 7;
    const size_t maxFeatureNumPerSubregion = 20; // 每个子区域的最大特征数
    const size_t xSubregionNum = 8; // 宽度上的区域划分数, for kitti
    const size_t ySubregionNum = 6; // 高度上的区域划分数
    const size_t totalSubregionNum = xSubregionNum * ySubregionNum; // 总区域数
    const size_t xBoundary = 20, yBoundary = 20; // 左右和上下预留的边界像素量

    const double subregionWidth = (640 - 2 * xBoundary) / static_cast<double>(xSubregionNum); // 单个子区域的宽度
    const double subregionHeight = (480 - 2 * yBoundary) / static_cast<double>(ySubregionNum); // 单个子区域的高度
    const double maxTrackDis = 100.0;  // 光流最大距离

    std::vector<std::vector<cv::KeyPoint>> featuresCurr, featuresLast; // 存放两帧的特征点
    std::vector<cv::KeyPoint> featuresSub;    // 存放图像某个子区域内检测到的特征点
    std::vector<cv::KeyPoint> toDistributeFeatures;
    std::vector<unsigned char> featuresStatus; // 光流追踪到的特征点的标志
//    std::vector<float> featucv::resError;    // 光流追踪到的特征点的误差

    size_t featuresIndFromStart; // 特征点的相对第一个点的索引
    size_t totalFeatureNum; // 总特征点数
    std::vector<size_t> featuresInd; // 所有特征点的相对索引
    std::vector<size_t> subregionFeatureNum; // 每个子区域的特征点数

public:
    SimpleVO(): nh("~") {
        subImageMsg = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1, &SimpleVO::imageMsgHandler, this);
        subImageDepthMsg = nh.subscribe<sensor_msgs::Image>("/camera/depth_registered/image_raw", 1, &SimpleVO::imageDepthMsgHandler, this);

        pubVisualOdom = nh.advertise<nav_msgs::Odometry>("/simpleVO", 5);

        for (int i = 0; i < 6; ++i)
            transformCur[i] = 0;

//        kMat = cv::Mat(3, 3, CV_64FC1, kImage);
//        dMat = cv::Mat(4, 1, CV_64FC1, dImage);

        firstFrame = true;
        systemReady = true;
        featuresIndFromStart = 0; // 特征点的相对第一个点的索引
        totalFeatureNum = 0; // 一帧图像的总特征点数
        subregionFeatureNum.resize(static_cast<size_t>(totalSubregionNum), 0); // 每个子区域的特征点数


    }

    void imageMsgHandler(const sensor_msgs::Image::ConstPtr& imageMsg) {
        imageHeader = imageMsg->header;

        timeImageCurr = imageHeader.stamp.toSec();

        imageCurr = cv_bridge::toCvShare(imageMsg, "mono8")->image;

        newImageMsg = true;
    }

    void imageDepthMsgHandler(const sensor_msgs::Image::ConstPtr& imageDepthMsg) {
        imageDepthHeader = imageDepthMsg->header;

        timeImageDepthCurr = imageDepthHeader.stamp.toSec();

        imageDepthCurr = cv_bridge::toCvShare(imageDepthMsg, "mono8")->image;

        newImageDepthMsg = true;
    }

    void fastDetect(const std::vector<cv::Mat>& imagePyr, std::vector<std::vector<cv::KeyPoint>>& AllFeatures) {
        AllFeatures.resize(nPyrLevels);



    }

    /// 为图像计算特征点，更新了 totalFeatureNum 和 subregionFeatureNum 两个变量的值
    void extractFeatures() {
        featuresIndFromStart = totalFeatureNum = 0;
        subregionFeatureNum.resize(totalSubregionNum, 0);
        if (!featuresSub.empty()) featuresSub.clear();
        if (!featuresInd.empty()) featuresInd.clear();
        if (!featuresCurr.empty()) featuresCurr.clear();

        for (size_t level = 0; level < nPyrLevels; ++level) {
            // 对每个子区域进行特征提取，分区域有助于特征点均匀分布
            for (size_t i = 0; i < xSubregionNum; i++) {
                for (size_t j = 0; j < ySubregionNum; j++) {
                    size_t ind = xSubregionNum * i + j;  // ind指向当前的subregion编号
                    size_t numToFind = maxFeatureNumPerSubregion;

                    size_t subregionLeft = xBoundary + subregionWidth * i;
                    size_t subregionTop = yBoundary + subregionHeight * j;

//                    std::vector<cv::KeyPoint> featuresSub;
                    cv::FAST(imagePyrCurr[level].rowRange(subregionTop, subregionTop+subregionHeight).colRange(subregionLeft, subregionLeft+subregionWidth),
                             featuresSub, iniThFAST, true);
                    if (featuresSub.empty()) {
                        cv::FAST(imagePyrCurr[level].rowRange(subregionTop, subregionTop+subregionHeight).colRange(subregionLeft, subregionLeft+subregionWidth),
                                 featuresSub, minThFAST, true);
                    }

                    if (!featuresSub.empty()) {
                        for (auto fit = featuresSub.begin(); fit != featuresSub.end(); fit++) {
                            (*fit).pt.x += i * subregionWidth;
                            (*fit).pt.y += j * subregionHeight;
                            toDistributeFeatures.push_back(*fit);
                        }
                    }
                }
            }

            std::vector<cv::KeyPoint> &keypoints = featuresCurr[level];
            keypoints.reserve(200);
//            keypoints = DistributeOctTree(toDistributeFeatures, minBorderX, maxBorderX,
//                                          minBorderY, maxBorderY, mnFeaturesPerLevel[level], level);

//            const int scaledPatchSize = PATCH_SIZE * mvScaleFactor[level];

            // Add border to coordinates and scale information
            const int nkps = keypoints.size();
            for (int i = 0; i < nkps; i++) {
                keypoints[i].pt.x += subregionWidth;
                keypoints[i].pt.y += subregionHeight;
                keypoints[i].octave = level;
                keypoints[i].size = 1 / pow(1.2, level);
            }
        }
    }

    void calculateTransformation() {

    }

    void publishOdometry() {


    }


    void run() {
        if (systemReady && newImageMsg && newImageDepthMsg &&
            std::abs(timeImageCurr - timeImageDepthCurr) < 0.05) {

            systemReady = false;
            newImageMsg = false;
            newImageDepthMsg = false;
        } else
            return;

        extractFeatures();

        calculateTransformation();

        publishOdometry();
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simpleVO");

    ROS_INFO("\033[1;32m---->\033[0m Simple Visual Odometry Started.");

    SimpleVO svo;

    ros::Rate rate(200);
    while (ros::ok()) {
        ros::spinOnce();

        svo.run();

        rate.sleep();
    }

    ros::spin();

    return 0;
}
