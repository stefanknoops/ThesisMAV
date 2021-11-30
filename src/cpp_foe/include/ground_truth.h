#pragma once

// ROS stuff
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

// OpenCV
#include <cv_bridge/cv_bridge.h>

// Geometry library
#include "relative_localization/pose.h"

namespace ground_truth {

class GroundTruth {
 public:
  GroundTruth(ros::NodeHandle& nh, ros::NodeHandle nh_private);
  ~GroundTruth();

  void paintOptitrackDrone(cv_bridge::CvImage& image);

 private:
  ros::NodeHandle nh_;

  void optitrackBebopCallback(const geometry_msgs::PoseStamped& msg);

  ros::Subscriber bebop_sub_;

  Pose bebop_pose_;

  int map_size_u_ = 180;
  int map_size_v_ = 249;
  int downscale_ = 1;
  float h_fov_ = 1.5;      // horizontal fov @ 1m [in meters]
  float v_fov_ = 2;        // vertical fov @ 1m [in meters]
  int bebop_size_h_ = 30;  // bebop horizontal size @ 1m [in pixels]
  int bebop_size_v_ = 10;  // bebop horizontal size @ 1m [in pixels]
  const Quaternion camera_rot_y_ = Quaternion(Euler(0, -M_PI / 2, 0));
  const Quaternion camera_rot_z_ = Quaternion(Euler(0, 0, M_PI / 2));
  Quaternion camera_offset_ = Quaternion(Euler());
  bool display_bebop_;
};

}  // namespace ground_truth
