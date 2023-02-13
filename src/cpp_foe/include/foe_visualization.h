#ifndef FOE_VISUALIZATION_H_
#define FOE_VISUALIZATION_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "cpp_foe/FoE.h"

namespace foe_visualization
{

class FoeVisualization {
    public:
        FoeVisualization(ros::NodeHandle & nh, ros::NodeHandle nh_private);
        virtual ~FoeVisualization();
        void process_image(const ros::TimerEvent& event);

    private:
        ros::NodeHandle nh_;

        void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
        void FocusOfExpansionCallback(const cpp_foe::FoE::ConstPtr& msg);


        image_transport::Publisher low_freq_image_pub_;

        image_transport::Subscriber image_sub_;
        bool has_image_;
        cv::Mat last_image_;

        ros::Subscriber FoE_sub_;
        int foe_ [2];
        

        ros::Timer timer_;

};

} // namespace

#endif // FOE_VISUALIZATION_H_
