#include "utility.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/flann/flann.hpp>

using namespace std;
using namespace cv;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "testMatch");
    ros::NodeHandle nh;

    Mat img1 = imread("/home/vance/slam_ws/slambook_course/L5/code/1.png", 0);
    Mat img2 = imread("/home/vance/slam_ws/slambook_course/L5/code/2.png", 0);

    // 1. 初始化
    vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    Ptr<ORB> orb = ORB::create();

    // 2. 提取特征点
    orb->detect(img1, keypoints1);
    orb->detect(img2, keypoints2);

    // 3. 计算特征描述符
    orb->compute(img1, keypoints1, descriptors1);
    orb->compute(img2, keypoints2, descriptors2);

    // 4. 对两幅图像的BRIEF描述符进行匹配，使用BFMatch，Hamming距离作为参考
    vector<DMatch> matches;
    vector<vector<DMatch>> knnMatches;

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
//    FlannBasedMatcher matcher;
    matcher->knnMatch(descriptors1, descriptors2, knnMatches, 2);

    cout << "matches: " << knnMatches.size() << endl;
    for (size_t r = 0; r < knnMatches.size(); ++r) {
        if (knnMatches[r].size() < 2)
            continue;
        if (knnMatches[r][0].distance > 0.8*knnMatches[r][1].distance )
            continue;
        matches.push_back(knnMatches[r][0]);
    }

    Mat showMatches;
    drawMatches(img1, keypoints1, img2, keypoints2,
                matches, showMatches);
    imshow("Matches", showMatches);
    waitKey(0);


    cv::Ptr<DescriptorMatcher> BfMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    vector<DMatch> BfMatches;
    BfMatcher->match(descriptors1, descriptors2, BfMatches);
    double max_dist = 0; double min_dist = 100;
    for(int i = 0; i < descriptors1.rows; i++) {
        double dist = BfMatches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );

    std::vector< DMatch > good_matches;
    for (int i = 0; i < descriptors1.rows; i++) {
        if( BfMatches[i].distance <= std::max(2*min_dist, 20.0) )
            good_matches.push_back( BfMatches[i]);
    }

    Mat show;
    drawMatches(img1, keypoints1, img2, keypoints2, good_matches, show);
    imshow("Good Matches", show);
    waitKey(0);

    return 0;
}
