#include "foe_visualization.h"

namespace foe_visualization {

FoeVisualization::FoeVisualization(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh)
{
    has_image_ = false;

    FoE_sub_ = nh.subscribe("FoE", 1, &FoeVisualization::FocusOfExpansionCallback, this);

    image_transport::ImageTransport it_(nh_);
    image_sub_ = it_.subscribe("dvs_rendering", 1, &FoeVisualization::imageCallback, this);
    low_freq_image_pub_ = it_.advertise("low_freq_rendering", 1);

    timer_ = nh.createTimer(ros::Duration(0.03), &FoeVisualization::process_image, this);
}

FoeVisualization::~FoeVisualization()
{
  low_freq_image_pub_.shutdown();
}


void FoeVisualization::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{

    // only use this image if previous one was already sent
    if (!has_image_)
    {
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        last_image_ = cv_ptr->image;
        has_image_ = true;
    }
}

void FoeVisualization::FocusOfExpansionCallback(const cpp_foe::FoE::ConstPtr& msg) 
{
    foe_[0] = msg->x;
    foe_[1] = msg->y;
}

void FoeVisualization::process_image(const ros::TimerEvent& event)
{    
    if (has_image_) {
        cv_bridge::CvImage cv_image;
        cv::circle(last_image_, cv::Point( foe_[0], foe_[1] ), 10, CV_RGB( 125, 125, 0 ) );
        last_image_.copyTo(cv_image.image);
        cv_image.encoding = "bgr8";
        low_freq_image_pub_.publish(cv_image.toImageMsg());
        has_image_ = false;
    }
}



} // namespace
