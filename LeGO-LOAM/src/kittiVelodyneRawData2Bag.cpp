#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

#include <vector>
#include <iostream>
#include <fstream>


using namespace pcl;
using namespace std;

fstream input;

void loadPointCloudData(string infile, pcl::PointCloud<PointXYZI>::Ptr& points) {
    input.open(infile.c_str(), ios::in | ios::binary);
    if(!input.good()){
        cerr << "Could not read file: " << infile << endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, ios::beg);

    for (int i=0; input.good() && !input.eof(); i++) {
        PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        points->push_back(point);
    }
    input.close();
}


// for kitti odometry 03 sequence
int main(int argc, char **argv){
    ros::init(argc, argv, "veloData2Bag");
    ros::NodeHandle nh;

    if (argc < 3) {
        cerr << "Usage: executable_file raw_data_directory output_bag_file\n";
        return -1;
    }

    ///The file to read from
    string directory(argv[1]);
    string outfile(argv[2]);
    string infile = directory + "/times.txt";
    cout << "directory: " << directory << endl;
    cout << "infile: " << infile << endl;
    cout << "outfile: " << outfile << endl;

    ifstream fTimes;
    fTimes.open(infile.c_str());
    if (!fTimes.is_open()) {
        cerr << "\nTime file: " << infile.c_str() << " does not exist! " << endl;
        return -1;
    }

    int frames = 0;
    vector<double> times;
    while(!fTimes.eof()) {
        frames++;
        double t;
        fTimes >> t;
        times.push_back(t);
    }
    times.pop_back();
    cout << "Read tatal " << --frames << " frames." << endl;

    rosbag::Bag bag;
    bag.open(outfile, rosbag::bagmode::Write);
//    bag.setCompression(rosbag::CompressionType::Uncompressed);

    pcl::PointCloud<PointXYZI>::Ptr points;
    sensor_msgs::PointCloud2::Ptr pointcloud;
    for (int i = 0; i < frames; ++i) {
        points.reset(new pcl::PointCloud<PointXYZI>);
        pointcloud.reset(new sensor_msgs::PointCloud2);

        char filename[128];
        sprintf(filename, "%06d", i);
        string infile(directory + "/velodyne_points/" + filename + ".bin");
        cout << "Loading data from file: " << infile << endl;

        loadPointCloudData(infile, points);

        pcl::toROSMsg(*points, *pointcloud);

        ros::Time time = ros::Time::now();
        std_msgs::Header header;
        header.frame_id = "velo_link";
        header.stamp = time;
        pointcloud->header = header;



//        float freq = 0.1;

        bag.write<sensor_msgs::PointCloud2>("/kitti/velo/pointcloud",
                                            time,
                                            *pointcloud);
    }

    bag.close();

    return 0;
}
