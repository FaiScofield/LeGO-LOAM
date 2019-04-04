#include "lego_loam/saveMap.h"
#include "utility.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_map_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<lego_loam::saveMap>("saveMap");

    lego_loam::saveMap srv;
    srv.request.save = atoi(argv[1]);
    if (client.call(srv) && srv.response.success) {
        ROS_INFO("Service called.");
    } else {
        ROS_ERROR("Failed to call service.");
    }

    ros::spin();

    return 0;
}
