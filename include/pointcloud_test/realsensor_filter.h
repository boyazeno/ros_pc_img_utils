#include "pointcloud_test/io_processer.hpp"
// PCL
#include "pcl/point_cloud.h"
#include "pcl/ros/conversions.h"
//OpenCV
#include "opencv2/core.hpp"
//ROS
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
//librealsenser2
#include "librealsense2/rs.hpp"
//std
#include <iostream>
#include <string>
#include <vector>

class RealSensorFilter
{
    public:
        enum StreamType{IMGRGB=1, IMGDEPTH=2, IMGCONFIDENCE=3};

        RealSensorFilter(){}
        RealSensorFilter(ros::NodeHandlePtr& nh) : nh_(nh){}

        bool startPipeline(const StreamType& stream_type)
        {
            switch (stream_type)
            {
            case StreamType::IMGRGB:
                this->cfg_.enable_stream(RS2_STREAM_COLOR, 640, 320, RS2_FORMAT_BGR8, 30);
                break;
            case StreamType::IMGDEPTH:
                this->cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 320, RS2_FORMAT_Z16, 30);
                break;
            case StreamType::IMGCONFIDENCE:
                this->cfg_.enable_stream(RS2_STREAM_CONFIDENCE, 640, 320, RS2_FORMAT_ANY, 30);
                break;
            default:
                this->cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 320, RS2_FORMAT_Z16, 30);
                break;
            }
            try
            {
                this->pipe_.start(this->cfg_);
            }
            catch(const rs2::error& e)
            {
                ROS_ERROR_STREAM("ERROR: cannot start pipeline in filter. Message: "<< e.what());
                return false;
            }
            return true;
        }

        bool getPointCloudOriginal(rs2::pointcloud& pc_out)
        {
            pc_out = this->original_pc_;
            return true;
        }

        bool getPointCloudFiltered(rs2::pointcloud& pc_out)
        {
            pc_out = this->filtered_pc_;
            return true;
        }


    private:
        ros::NodeHandlePtr nh_;
        rs2::pointcloud original_pc_;
        rs2::pointcloud filtered_pc_;

        rs2::pipeline pipe_;
        rs2::config cfg_;

        bool initParam();

};