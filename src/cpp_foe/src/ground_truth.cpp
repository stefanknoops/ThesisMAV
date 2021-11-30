#include "relative_localization/ground_truth.h"

namespace ground_truth {

GroundTruth::GroundTruth(ros::NodeHandle& nh, ros::NodeHandle nh_private)
    : nh_(nh), bebop_pose_(), neuromav_pose_(), display_bebop_(true) {
  // Setup subscribers and publishers
  optitrack_sub_ = nh_.subscribe("/optitrack/pose", 1,
                             &GroundTruth::optitrackCallback, this);
}

GroundTruth::~GroundTruth() {
  // Shut down subscribers
  optitrack_sub_.shutdown();
}

void GroundTruth::optitrackCallback(
    const geometry_msgs::PoseStamped& msg) {
  Vector3 pos;
  Quaternion ori;

  // We still don't use the orientation of the bebop for anything
  pos.setX(msg.pose.position.x);
  pos.setY(msg.pose.position.y);
  pos.setZ(msg.pose.position.z);
  ori.setX(msg.pose.orientation.x);
  ori.setY(msg.pose.orientation.y);
  ori.setZ(msg.pose.orientation.z);
  ori.setW(msg.pose.orientation.w);

  bebop_pose_.setPos(pos);
  bebop_pose_.setOri(ori);
}


void GroundTruth::paintTrueFoE(cv_bridge::CvImage& image) {
  Pose default_pose;

  // If the poses have been initialized
  if (bebop_pose_ != default_pose) {
    // Get neuromav's orientation
    Quaternion neuromav_ori = neuromav_pose_.getOri();

    // The camera's orientation is rotated 90deg wrt the neuromav
    Vector3 camera_n =
        neuromav_ori * (camera_rot_y_ + camera_offset_) * Vector3(1, 0, 0);
    Vector3 camera_v =
        neuromav_ori * (camera_rot_y_ + camera_offset_) * Vector3(0, 1, 0);
    Vector3 camera_h =
        neuromav_ori * (camera_rot_y_ + camera_offset_) * Vector3(0, 0, 1);

    // Get relative location of bebop and neuromav drones
    const Pose rel_pose = bebop_pose_ - neuromav_pose_;
    const Vector3 rel_pos = rel_pose.getPos();

    bool inFoV = false;
    double c_x, c_y, w, h;

    // Check if the bebop's relative position is in the FoV of the neuromav
    // Method obtained from:
    // https://math.stackexchange.com/questions/4144827/determine-if-a-point-is-in-a-cameras-field-of-view-3d
    const float p1 = dot(rel_pos, camera_n);

    if (p1 > 0) {
      const Vector3 proyection = (rel_pos / p1) - camera_n;
      const float h1 = dot(proyection, camera_h) + h_fov_ / 2;
      const float v1 = dot(proyection, camera_v) + v_fov_ / 2;

      // If bebop is in FoV, get coordinates on screen
      if ((h1 > 0) && (h1 < h_fov_) && (v1 > 0) && (v1 < v_fov_)) {
        inFoV = true;
        c_x = h1 / h_fov_;
        c_y = (v_fov_ - v1) / v_fov_;
      }
    }

    float bebop_distance = rel_pos.getModule();
    w = bebop_size_h_ / downscale_ / bebop_distance / map_size_u_;
    h = bebop_size_v_ / downscale_ / bebop_distance / map_size_v_;

    // If in FoV, draw bebop on screen
    if (inFoV) {
      int min_x = map_size_u_ * (c_x - w / 2);
      int max_x = map_size_u_ * (c_x + w / 2);
      int min_y = map_size_v_ * (c_y - h / 2);
      int max_y = map_size_v_ * (c_y + h / 2);

      // Paint corresponding pixels
      if (display_bebop_) {
        for (int x = min_x; x <= max_x; x++) {
          if (x > 0 && x < map_size_u_) {
            if (min_y > 0) {
              image.image.at<cv::Vec3b>(cv::Point(x, min_y)) =
                  cv::Vec3b(255, 0, 0);
            }
            if (max_y < map_size_v_) {
              image.image.at<cv::Vec3b>(cv::Point(x, max_y)) =
                  cv::Vec3b(255, 0, 0);
            }
          }
        }

        for (int y = min_y; y <= max_y; y++) {
          if (y > 0 && y < map_size_v_) {
            if (min_x > 0) {
              image.image.at<cv::Vec3b>(cv::Point(min_x, y)) =
                  cv::Vec3b(255, 0, 0);
            }
            if (max_x < map_size_u_) {
              image.image.at<cv::Vec3b>(cv::Point(max_x, y)) =
                  cv::Vec3b(255, 0, 0);
            }
          }
        }
      }
    }
  }
}

}  // namespace ground_truth