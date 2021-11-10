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

#include "dvs_server/server.h"

namespace dvs_server {

Server::Server(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh){
	// setup subscribers and publishers
	event_sub_ = nh_.subscribe("events", 1, &Server::eventsCallback, this);
	imu_sub_ = nh_.subscribe("imu", 1, &Server::imuCallback, this);
	// data record
	//std::ofstream DVS_rec_file;
	DVS_rec_file.open("DVS_recording.txt");
	IMU_rec_file.open("IMU_recording.txt");

	// setup communication with SHM
        int mem_ID;
        mem_ID = shmget(KEY,sizeof(myEVENTS),0666|IPC_CREAT);
        if(mem_ID<0){
        	ROS_WARN("Error executing SHMGET");
                exit(1);
        }else{
        	ROS_INFO("SHMGET:\t Success!  ");
		ptr_SHM = shmat(mem_ID,NULL,0);
		if(ptr_SHM==(void*)-1){
                	ROS_WARN("Error executing SHMAT");
                }else{
                        ROS_INFO("SHMAT:\t Success!  ");
		}
	}
}

Server::~Server()
{
	shmdt(ptr_SHM);
	DVS_rec_file.close();
	IMU_rec_file.close();
	std::cout << "Server stopped." << std::endl;
}

void Server::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	IMU_rec_file << (msg->header.stamp.toNSec()/1000) << ",";
    IMU_rec_file << (int) msg->linear_acceleration.x << ",";
    IMU_rec_file << (int) msg->linear_acceleration.y << ",";
    IMU_rec_file << (int) msg->linear_acceleration.z << ",";
	IMU_rec_file << (int) msg->angular_velocity.x << ",";
	IMU_rec_file << (int) msg->angular_velocity.y << ",";
	IMU_rec_file << (int) msg->angular_velocity.z << std::endl;
}

void Server::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{

	for (int i=0; i<msg->events.size(); ++i) {
    		DVS_rec_file << (msg->events[i].ts.toNSec()/1000) << ",";
		DVS_rec_file << (int) msg->events[i].x << ",";
                DVS_rec_file << (int) msg->events[i].y << ",";
                DVS_rec_file << (int) msg->events[i].polarity << std::endl;
	}

}

} // namespace
