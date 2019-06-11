#include "utility.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/flann/flann.hpp>

using namespace std;
using namespace cv;

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

    float transformCur[6];
    vector<float*> transforms;


    Mat debugShowImage;
    Mat imageCurr, imageLast, imageDepthCurr, imageDepthLast, imageShow, show;
    Mat descripCurr, descripLast;
//    Mat kMat, dMat; // 相机参数矩阵
//    Mat imageEig, imageTmp;

    bool firstFrame, systemReady;
    bool newImageMsg, newImageDepthMsg;
    double timeImageCurr, timeImageDepthCurr;

    const int iniThFAST = 25;
    const int minThFAST = 15;
//    const int maxFeatureNumPerSubregion = 20; // 每个子区域的最大特征数
    const int xSubregionNum = 15; // 宽度上的区域划分数, for kitti
    const int ySubregionNum = 8; // 高度上的区域划分数
    const int totalSubregionNum = xSubregionNum * ySubregionNum; // 总区域数
    const int xBoundary = 20, yBoundary = 20; // 左右和上下预留的边界像素量

    const int imageRows = 370, imageCols = 1226;
    const int subregionWidth = static_cast<int>((imageCols - 2 * xBoundary) / xSubregionNum);
    const int subregionHeight = static_cast<int>((imageRows - 2 * yBoundary) / ySubregionNum);
    vector<size_t> subregionFeatureNum; // 每个子区域的特征点数
    size_t featuresIndFromStart; // 特征点的相对第一个点的索引
    size_t totalFeatureNum; // 总特征点数

    vector<KeyPoint> featuresCurr, featuresLast; // 存放两帧的特征点
    vector<KeyPoint> featuresSub;    // 存放图像某个子区域内检测到的特征点
    vector<DMatch> goodMatches, RansacMatches;

    size_t frameNum = 0;

public:
    SimpleVO(): nh("~") {
        subImageMsg = nh.subscribe<sensor_msgs::Image>("/kitti/camera_color_left/image_raw", 1, &SimpleVO::imageMsgHandler, this);
//        subImageMsg = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1, &SimpleVO::imageMsgHandler, this);

        subImageDepthMsg = nh.subscribe<sensor_msgs::Image>("/camera/depth_registered/image_raw", 1, &SimpleVO::imageDepthMsgHandler, this);

        pubVisualOdom = nh.advertise<nav_msgs::Odometry>("/simpleVO", 5);
        pubImageShow = nh.advertise<sensor_msgs::Image>("/image_show", 5);

        for (int i = 0; i < 6; ++i)
            transformCur[i] = 0;

//        imageRows = 480;
//        imageCols = 640;
//        kMat = Mat(3, 3, CV_64FC1, kImage);
//        dMat = Mat(4, 1, CV_64FC1, dImage);

        imageCurr = Mat(imageRows, imageCols, CV_8UC1, Scalar::all(0));
        imageLast = Mat(imageRows, imageCols, CV_8UC1, Scalar::all(0));
        imageDepthCurr = Mat(imageRows, imageCols, CV_32FC1, Scalar::all(0.0));
        imageDepthLast = Mat(imageRows, imageCols, CV_32FC1, Scalar::all(0.0));

        firstFrame = true;
        systemReady = true;
        newImageMsg = false;
        newImageDepthMsg = false;


        featuresIndFromStart = 0; // 特征点的相对第一个点的索引
        totalFeatureNum = 0; // 一帧图像的总特征点数
        subregionFeatureNum.resize(static_cast<size_t>(totalSubregionNum), 0);
    }

    void imageMsgHandler(const sensor_msgs::Image::ConstPtr& imageMsg) {
        ROS_INFO("Get an image message.");
        imageHeader = imageMsg->header;

        timeImageCurr = imageHeader.stamp.toSec();

        try {
            imageShow = cv_bridge::toCvShare(imageMsg, "bgr8")->image;
            show = imageShow.clone();
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            newImageMsg = false;
            return;
        }
        cvtColor(imageShow, imageCurr, CV_BGR2GRAY);
        debugShowImage = imageShow.clone();

        newImageMsg = true;
    }

    void imageDepthMsgHandler(const sensor_msgs::Image::ConstPtr& imageDepthMsg) {
        imageDepthHeader = imageDepthMsg->header;

        timeImageDepthCurr = imageDepthHeader.stamp.toSec();

        try {
            imageDepthCurr = cv_bridge::toCvShare(imageDepthMsg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            newImageDepthMsg = false;
            return;
        }

        newImageDepthMsg = true;
    }

    void computeAngle(const Mat& image, vector<KeyPoint>& keypoints) {
        int half_patch_size = 8;
        for (auto &kp : keypoints) {
            int m_10 = 0, m_01 = 0;
            for (int u = -half_patch_size; u <= half_patch_size-1; ++u) {
                for (int v = -half_patch_size; v <= half_patch_size-1; ++v) {
                    int u1 = cvRound(kp.pt.x) + u, v1 = cvRound(kp.pt.y) + v;
                    if (u1 >= 0 && u1 < image.cols && v1 >= 0 && v1 < image.rows) {
                        m_10 += u  * image.at<uchar>(v1, u1);
                        m_01 += v  * image.at<uchar>(v1, u1);
                    } else continue;
                }
            }
            kp.angle = fastAtan2((float)m_01, (float)m_10);
        }
        return;
    }

    /// 为图像计算特征点，更新了 totalFeatureNum 和 subregionFeatureNum 两个变量的值
    void extractFeatures() {
        subregionFeatureNum.resize(totalSubregionNum, 0);
        featuresCurr.clear();

        // 对每个子区域进行特征提取，分区域有助于特征点均匀分布
        for (int j = 0; j < ySubregionNum; j++) {       // j-row
            for (int i = 0; i < xSubregionNum; i++) {   // i-col
                featuresSub.clear();

                int ind = xSubregionNum * j + i;  // ind指向当前的subregion编号
                int subregionLeft = xBoundary + subregionWidth * i;
                int subregionTop = yBoundary + subregionHeight * j;

                FAST(imageCurr.rowRange(subregionTop, subregionTop+subregionHeight).colRange(subregionLeft, subregionLeft+subregionWidth),
                     featuresSub, iniThFAST, true,
                     FastFeatureDetector::TYPE_9_16);

                if (featuresSub.empty()) {
                    FAST(imageCurr.rowRange(subregionTop, subregionTop+subregionHeight).colRange(subregionLeft, subregionLeft+subregionWidth),
                         featuresSub, minThFAST, true,
                         FastFeatureDetector::TYPE_9_16);
                }

                if (!featuresSub.empty()) {
                    for (auto fit = featuresSub.begin(); fit != featuresSub.end(); fit++) {
                        (*fit).pt.x += subregionLeft;   // x - col
                        (*fit).pt.y += subregionTop;    // y - row
                        if ((*fit).pt.x < 0 || (*fit).pt.x >= imageCols ||
                            (*fit).pt.y < 0 || (*fit).pt.y >= imageRows) {
                            ROS_WARN("Delete a keypoint for out of range.");
                            continue;
                        }

                        featuresCurr.push_back(*fit);
                    }
                }
                subregionFeatureNum[ind] = featuresSub.size();
            }
        }

//        computeAngle(imageCurr, featuresCurr);

        Ptr<ORB> orb = ORB::create();
        orb->compute(imageCurr, featuresCurr, descripCurr);

        ROS_INFO("Frame %d, Features: %d", frameNum, featuresCurr.size());
    }

    /// working on it
//    void calculateTransformation() {
//        return;
//    }

    /// working on it
//    void publishOdometry() {
//        return;
//    }

    void publishOutputImage() {
        for (auto& fea : featuresCurr) {
            Point2f pixel = fea.pt;
            if (pixel.x < 0 || pixel.x >= imageCols ||
                pixel.y < 0 || pixel.y >= imageRows) {
                ROS_WARN("error feature position! skip!");
                continue;
            }
            try {
                circle(imageShow, pixel, 2, Scalar(0, 255, 0), -1);
            } catch (cv::Exception& e) {
                ROS_ERROR("cv exception: %s", e.what());
            }
//            circle(imageShow, pixel, 2, Scalar(0, 255, 0), -1);
        }

        bridge.image = imageShow;
        bridge.header = imageHeader;
        bridge.encoding = "bgr8";
        pubImageShow.publish(bridge.toImageMsg());
    }

    void featureMatching() {
//        vector<DMatch> matches;
        vector<vector<DMatch>> knnMatches;
        goodMatches.clear();
        RansacMatches.clear();

//        cv::Mat desp_map;
//        for (int i = 0; i < descripCurr.size(); ++i) {
//            descripCurrMat = Mat::zeros(256, descripCurr.size(), CV_8UC1);
//            for (int j = 0; i < 256; ++j)
//                descripCurrMat.at<uchar>(j, i) = descripCurr[i][j];
//        }

        FlannBasedMatcher matcher(new cv::flann::LshIndexParams(20, 10, 2));
        matcher.knnMatch(descripLast, descripCurr, knnMatches, 2);

        // 选择最好的匹配点
        for (size_t r = 0; r < knnMatches.size(); ++r) {
            if (knnMatches[r].size() < 2)
                continue;
            if (knnMatches[r][0].distance > 0.9*knnMatches[r][1].distance )
                continue;
            goodMatches.push_back(knnMatches[r][0]);
        }

        // RANSAC
        vector<Point2f> k1, k2;
        vector<uchar> inliners;
        for (size_t i = 0; i < goodMatches.size(); i++) {
            int q = goodMatches[i].queryIdx;
            int t = goodMatches[i].trainIdx;
            k1.push_back(featuresLast[q].pt);
            k2.push_back(featuresCurr[t].pt);
        }
        Mat F = findFundamentalMat(k1, k2, inliners, FM_RANSAC);
        for (size_t i = 0; i < goodMatches.size(); i++) {
            if (inliners[i])
                RansacMatches.push_back(goodMatches[i]);
        }

         ROS_INFO("RANSAC/good/all matches size: %d / %d / %d", RansacMatches.size(), goodMatches.size(), knnMatches.size());
    }

    //交换前后帧数据
    void swapData() {
        try {
            imageLast.release();
            imageLast = imageCurr.clone();
//            imageDepthLast.release();
//            imageDepthLast = imageDepthCurr.clone(); //深度图像数据交换会segfault
    //        imageCurr.copyTo(imageLast);
    //        imageDepthCurr.copyTo(imageDepthLast);

            featuresLast.swap(featuresCurr);

            descripLast.release();
            descripLast = descripCurr.clone();
        } catch (cv::Exception& e) {
            ROS_ERROR("cv exception: %s", e.what());
//            systemReady = false;
            return;
        }
        systemReady = true;
    }

    void run() {
        if (systemReady && newImageMsg /*&& newImageDepthMsg &&
            abs(timeImageCurr - timeImageDepthCurr) < 0.05*/) {

            systemReady = false;
            newImageMsg = false;
//            newImageDepthMsg = false;
        } else
            return;

        frameNum++;

        extractFeatures();

        publishOutputImage();

        if (firstFrame) {
            swapData();
            firstFrame = false;
            return;
        }


//        featureMatching();

//        calculateTransformation();

//        publishOdometry();

        if (saveDataForDebug && frameNum > 200 && frameNum % 20 == 0) {
//            ROS_INFO("Features current: %d, and last: %d", featuresCurr.size(), featuresLast.size());

            Mat showMatches, showMatches2;
            drawMatches(imageLast, featuresLast, imageCurr, featuresCurr,
                        goodMatches, showMatches);
            drawMatches(imageLast, featuresLast, imageCurr, featuresCurr,
                        RansacMatches, showMatches2);

            imshow("Matches", showMatches);
            waitKey(10);
            stringstream ss, ss2, ss3, ss4;
            ss << "/home/vance/matches/" << frameNum << ".jpg";
            ss2 << "/home/vance/matches/" << frameNum << "_ransac.jpg";
            imwrite(ss.str(), showMatches);
            imwrite(ss2.str(), showMatches2);

            ss3 << "/home/vance/matches/frame_" << frameNum << "_sep.jpg";
            imwrite(ss3.str(), imageShow);

            vector<KeyPoint> featureTmp;
            FAST(imageCurr, featureTmp, 1.5*iniThFAST, true, FastFeatureDetector::TYPE_9_16);
            for (auto& fea : featureTmp) {
                Point2f pixel = fea.pt;
                circle(show, pixel, 2, Scalar(0, 255, 0), -1);
            }
            ss4 << "/home/vance/matches/frame_" << frameNum << ".jpg";
            imwrite(ss4.str(), show);
        }

        swapData();
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simpleVO");

    ROS_INFO("\033[1;32m---->\033[0m Simple Visual Odometry Started.");

    SimpleVO simvo;

    ros::Rate rate(30);
    while (ros::ok()) {
        ros::spinOnce();

        simvo.run();

        rate.sleep();
    }

    ros::spin();

    return 0;
}
