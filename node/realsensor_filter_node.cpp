
#include "pointcloud_test/realsensor_filter.h"

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"realsensor_filter_node");
    ros::NodeHandlePtr nh(new ros::NodeHandle);
    ros::AsyncSpinner spinner(2);

    spinner.start();
    RealSensorFilter filter;
    return 0;
}