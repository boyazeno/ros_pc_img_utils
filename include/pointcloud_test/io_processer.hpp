#ifndef _IO_PROCESSER_H_
#define _IO_PROCESSER_H_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include "pcl/io/pcd_io.h"

#include "opencv2/core.hpp"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "cv_bridge/cv_bridge.h"

#include <iostream>
#include <string>
#include <vector>
#include <boost/bind.hpp>
#include <boost/function.hpp>

class IOProcesser
{
    public:
    IOProcesser(){}

    bool setNodeHandle()
    {
        this->nh_.reset(new ros::NodeHandle());
        return true;
    }

    bool setNodeHandle(ros::NodeHandlePtr& nh)
    {
        this->nh_ = nh;
        return true;
    }

    bool setFramePC(const std::string& frame_img)
    {
        this->frame_img_ = frame_img;
        return true;
    }

    bool setFrameImg(const std::string& frame_pc)
    {
        this->frame_pc_ = frame_pc;
        return true;
    }

    bool setPublisherImg(const std::string& topic_name="ioprocesser_image_out")
    {
        this->pub_img_ = this->nh_->advertise<sensor_msgs::Image>(topic_name, 2, true);
        return true;
    }

    bool setPublisherPC(const std::string& topic_name="ioprocesser_pointcloud_out")
    {
        this->pub_pc_ = this->nh_->advertise<sensor_msgs::PointCloud2>(topic_name, 2, true);
        return true;
    }

    bool setSubscriberImg(const boost::function<bool(sensor_msgs::ImagePtr)>& execute_func, const std::string& topic_name="ioprocesser_img_out")
    {
        this->sub_img_ = this->nh_->subscribe<sensor_msgs::Image>(topic_name,1,boost::bind(&IOProcesser::callbackImg, this, _1, &execute_func));
        return true;
    }

     bool setSubscriberPC(const std::string& topic_name="ioprocesser_pointcloud_out")
    {
        this->sub_pc_ = this->nh_->subscribe<sensor_msgs::PointCloud2>(topic_name,1,boost::bind(&IOProcesser::callbackPC, this, _1));
        return true;
    }

    bool publishImg(const sensor_msgs::ImagePtr& img)
    {
        this->pub_img_.publish(img);
        return true;
    }

    bool publishImg(const Eigen::MatrixXd& img);

    bool publishImg(const cv::Mat& img, const std::string& encoding= sensor_msgs::image_encodings::BGR8)
    {
        cv_bridge::CvImage cv_img;
        cv_img.image = img;
        cv_img.header.frame_id = this->frame_img_;
        cv_img.header.stamp = ros::Time::now();
        cv_img.encoding = encoding;
        return this->publishImg(cv_img.toImageMsg());
    }

    bool publishPC(const sensor_msgs::PointCloud2& pc)
    {
        this->pub_pc_.publish(pc);
        return true;
    }

    bool publishPC(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc)
    {
        sensor_msgs::PointCloud2 pc_msg;
        pcl::toROSMsg(*pc, pc_msg);
        pc_msg.header.frame_id = this->frame_pc_;
        return this->publishPC(pc_msg);
    }

    bool publishPC(const Eigen::MatrixX3d& pc, const int& width=-1, const int& height=-1)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        pc_pcl->getMatrixXfMap() = pc.cast<float>();
        if(width>0)
        {
            pc_pcl->width = width;
        }
        else
        {
            pc_pcl->width = pc.rows();
        }
        pc_pcl->height = height;
        return this->publishPC(pc_pcl);   
    }

    private:
    ros::NodeHandlePtr nh_;
    ros::Subscriber sub_pc_;
    ros::Subscriber sub_img_;
    ros::Publisher pub_pc_;
    ros::Publisher pub_img_;

    sensor_msgs::ImagePtr img_msg_;
    sensor_msgs::PointCloud2Ptr pc_msg_;

    std::string frame_pc_;
    std::string frame_img_;

    void callbackImg(sensor_msgs::ImageConstPtr& img_msg, const boost::function<bool(sensor_msgs::ImagePtr)>& execute_func)
    {
        *img_msg_ = *img_msg;
        if(!execute_func(img_msg_))
        {
            ROS_WARN_STREAM("Cannot execute processing function inside callback function at"<<ros::Time::now());
        }
        return;
    }

    void callbackPC(sensor_msgs::PointCloud2ConstPtr& pc_msg)
    {
        *pc_msg_ = *pc_msg;
        return;
    }

    public:

    static bool savePointCloud2MsgAsPcd(const sensor_msgs::PointCloud2ConstPtr& pc_msg, const std::string& path)
    {
        pcl::PCLPointCloud2::Ptr pc_pcl;
        pcl_conversions::toPCL(*pc_msg, *pc_pcl);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl_xyzrgb;
        pcl::fromPCLPointCloud2<pcl::PointXYZRGB>(*pc_pcl, *pc_pcl_xyzrgb);
        if(pcl::io::savePCDFileASCII (path, *pc_pcl_xyzrgb)==-1)
        {
            PCL_ERROR("Couldn't save file to path %s", path.c_str());
        }
        return true;
    }

    static bool saveImgMsgAsPng(const sensor_msgs::ImageConstPtr& img, const std::string& path)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return false;
        }
        
        return true;
    }

    template<typename T>
    static bool loadPcdAsPointCloud2Msg(const std::string& c, sensor_msgs::PointCloud2Ptr& pc_out)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_pcl_xyz;
        loadPcdAsPointCloudPCL<T>(c, *pc_pcl_xyz);
        pcl::toROSMsg(*pc_pcl_xyz, pc_out);
        return true;
    }

    template<typename T>
    static bool loadPcdAsPointCloudPCL(const std::string& path, pcl::PointCloud<T>& pc_out)
    {
        if(pcl::io::loadPCDFile<T> (path, *pc_out)==-1)
        {
            PCL_ERROR("Couldn't load file from path %s", path.c_str());
        }
        return true;
    }

    static bool loadPngAsMatCV(const std::string& path, sensor_msgs::PointCloud2Ptr& pc_out)
    {

        return true;
    }

    static bool loadPngAsImgMsg(const std::string& path, sensor_msgs::PointCloud2Ptr& pc_out)
    {

        return true;
    }

    static bool loadPngAsMatEigen(const std::string& path, sensor_msgs::PointCloud2Ptr& pc_out)
    {

        return true;
    }


    
}; // IOProcesser


#endif