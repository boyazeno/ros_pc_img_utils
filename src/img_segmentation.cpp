#include "pointcloud_test/img_segmentation.h"

bool ImgSegmentor::filter(const sensor_msgs::ImagePtr& img_ptr)
{   
    // Get background image at first
    if(this->is_first_frame_)
    {
        if(!this->setImgOriginal(img_ptr))
        {
            retry_++;
            if(retry_>retry_max_)
            {
                ROS_ERROR("After retry still didn't get any image as background!");
                return false;
            }
            ROS_WARN("Did not get image, retrying...");
            return true;
        }
        ROS_INFO("Got background image!");
        this->is_first_frame_ = true;
        return true;
    }

    // Use background image to find the object region
    cv::Mat img_masked;
    //TODO 
    // 1. add color filter
    // 2. add light adjustment
    // 3. compare pixelwise similarity, generate masks
    // 4. do open+close(erosion, dilation, erosion), or pick kontour > min_threshold
    // 5. mask can be used as weight for graps choosing
    cv::Mat img_hsv;
    cv_bridge::CvImageConstPtr img_bridge = cv_bridge::toCvShare(img_ptr, sensor_msgs::image_encodings::RGB8);
    cv::cvtColor(img_bridge->image, img_hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels_hsv;
    cv::split(img_hsv, channels_hsv);
    img_hsv==img_hsv;
    
    this->io_processer_.publishImg(img_masked);
    return true;
}

bool ImgSegmentor::setImgOriginal(const sensor_msgs::ImagePtr& img)
{
    img_original_ptr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
    if(img_original_ptr_->image.empty())
    {
        ROS_ERROR("Cannot get initial image as background!");
        return false;
    }
    return true;
}

bool ImgSegmentor::setParam()
{
    this->is_first_frame_ = false;
    this->retry_ = 0;
    ros::param::param<int>("~retry_background_img", retry_max_, 5); 
    std::string topic_name_sub;
    ros::param::param<std::string>("~topic_name_sub", topic_name_sub, "image_camera"); 
    std::string topic_name_pub;
    ros::param::param<std::string>("~topic_name_pub", topic_name_pub, "image_out"); 
    std::string frame_img;
    ros::param::param<std::string>("~frame_img", frame_img, "camera"); 
    std::string frame_pc;
    ros::param::param<std::string>("~frame_pc", frame_pc, "camera"); 
    this->io_processer_.setNodeHandle();
    this->io_processer_.setFrameImg(frame_img);
    this->io_processer_.setFramePC(frame_pc);
    this->io_processer_.setSubscriberImg(boost::bind(&ImgSegmentor::filter, this, _1), topic_name_sub);
    this->io_processer_.setPublisherImg(topic_name_pub);
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
    cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
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
    cv::cvtColor(img_hsv, img, cv::COLOR_HSV2BGR);
    return true;
}

bool _compareImgsWithThreshold(cv::Mat& mask , const cv::Mat& a, const cv::Mat& b, const int& threshold)
{
    if((a.cols!=b.cols) || (a.rows!=b.rows))
    {
        throw std::range_error("Input images not match!");
        return false;
    }
    cv::Mat mask_m(a.rows,a.cols, CV_8U, 0);
    mask = mask_m;
    for(size_t i=0; i<a.rows; ++i)
    {
        const uchar* pixrowA = a.ptr<uchar>(i);
        const uchar* pixrowB = b.ptr<uchar>(i);
        uchar* pixrowM = mask.ptr<uchar>(i);
        for(size_t j=0; j<a.cols; ++j)
        {
            pixrowM[j] = std::abs(pixrowA[j]-pixrowB[j])<=threshold? 255:0;   
        }
    }
    return true;
}