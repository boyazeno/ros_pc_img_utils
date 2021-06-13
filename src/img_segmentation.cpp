#include "pointcloud_test/img_segmentation.h"

bool ImgSegmentor::filter()
{

}

bool ImgSegmentor::setImgOriginal()
{

}

bool ImgSegmentor::setImgScene()
{

}

bool ImgSegmentor::toMatCV(const sensor_msgs::ImagePtr& img_msg,  cv_bridge::CvImagePtr&  img_out)
{
    img_out = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    if(this->img_original_ptr_->image.empty())
    {
        img_out.reset();
        ROS_ERROR("Cannot change Image message to cv Mat!");
        return false;
    }
    return true;
}


bool adjustLight(cv::Mat &img, int blockSize)
{
    if(img.channels()!=3)
    {
        ROS_ERROR("Need input image to be RGB.");
        return false;
    }

    // convert to HSV
    cv::Mat img_hsv;
    cv::cvtColor(img, img_hsv, cv::COLOR_RGB2HSV);
    std::vector<cv::Mat> img_splited;

    // Split the image and get the value channel for adjustment
    cv::split(img_hsv, img_splited);
    cv::Mat img_v = img_splited[2];

    // Subsplit into smaller image with block size
	double average = cv::mean(img_v)[0];
	int rows_new = ceil(double(img_v.rows) / double(blockSize));
	int cols_new = ceil(double(img_v.cols) / double(blockSize));
    
    // Get the average value of each block
	cv::Mat blockimg;
	blockimg = cv::Mat::zeros(rows_new, cols_new, CV_32FC1);
	for (int i = 0; i < rows_new; i++)
	{
		for (int j = 0; j < cols_new; j++)
		{
			int rowmin = i*blockSize;
			int rowmax = (i + 1)*blockSize;
			if (rowmax > img_v.rows) rowmax = img_v.rows;
			int colmin = j*blockSize;
			int colmax = (j + 1)*blockSize;
			if (colmax > img_v.cols) colmax = img_v.cols;
			cv::Mat imgROI = img_v(cv::Range(rowmin, rowmax), cv::Range(colmin, colmax));
			double temaver = cv::mean(imgROI)[0];
			blockimg.at<float>(i, j) = temaver;
		}
	}
	blockimg = blockimg - average;
	cv::Mat blockimg_resized;
	resize(blockimg, blockimg_resized, img.size(), (0, 0), (0, 0), cv::INTER_CUBIC);
	cv::Mat img2;
	img_v.convertTo(img2, CV_32FC1);
	cv::Mat img_adjusted = img2 - blockimg_resized;

    // Replace the value channel and merge back to HSV
	img_splited[2] = img_adjusted;
    cv::merge(img_splited, img_hsv);

    // Convert back to RGB
    cv::cvtColor(img_hsv, img, cv::COLOR_HSV2RGB);
    return true;
}