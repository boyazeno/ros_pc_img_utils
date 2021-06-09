#include "pcl/point_cloud.h"
#include "pcl/ros/conversions.h"

#include "opencv/core.hpp"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <iostream>
#include <string>
#include <vector>

class Filter
{
    public:
    Filter(){}
    Filter(const std::string&  pc_topic, const int& filter_times);


    private:
    int filter_times_;
    std::string pc_topic_;

};