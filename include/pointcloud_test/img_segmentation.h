#ifndef _IMG_SEGMENTATION_H_
#define _IMG_SEGMENTATION_H_
// OpenCV
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

// ROS
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

// std
#include <iostream>
#include <string>

// pointcloud_test
#include "pointcloud_test/io_processer.hpp"

class ImgSegmentor
{
    public:
    ImgSegmentor(){}

    bool filter();

    bool setParam();

    bool setImgOriginal();

    bool setImgScene();

    bool getImgMaskCV();

    bool getImgMaskMsg();


    private:
    cv_bridge::CvImagePtr img_original_ptr_;
    cv_bridge::CvImagePtr img_scene_ptr_;
    cv::Mat img_original_mat_;
    IOProcesser io_processer_;


    bool toMatCV(const sensor_msgs::ImagePtr& img_msg, cv_bridge::CvImagePtr& img_out);

    bool adjustLight(cv::Mat &img, int blockSize);
};




#endif