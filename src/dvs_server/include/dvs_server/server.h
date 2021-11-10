/**
 * This file is part of the odroid_ros_dvs package - MAVLab TU Delft
 * 
 *   MIT License
 *
 *   Copyright (c) 2020 MAVLab TU Delft
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 * 
 * */

#ifndef DVS_SERVER_H_
#define DVS_SERVER_H_

#include <ros/ros.h>
#include <string>

#include <std_msgs/Float32.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <iostream>
#include <fstream>

#include <sensor_msgs/CameraInfo.h>

#include <sensor_msgs/Imu.h>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#define KEY 666
#define N_EVENTS 1

namespace dvs_server
{

class Server {
public:
  Server(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  virtual ~Server();

private:
  ros::NodeHandle nh_;
  // Getting the Events
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  // Getting the IMU data
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  // Releasing stats
  //void publishStats();

  ros::Subscriber event_sub_;
  ros::Subscriber imu_sub_;

  struct IMUCameraSHM {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyr_x;
    float gyr_y;
    float gyr_z;
  };
  IMUCameraSHM myIMU;

  struct EventsSHM {
    int x;
    int y;
    bool p;
  };
  EventsSHM myEVENTS;

  void* ptr_SHM;
  std::ofstream DVS_rec_file;
  std::ofstream IMU_rec_file;

};

} // namespace

#endif // DVS_SERVER_H_
