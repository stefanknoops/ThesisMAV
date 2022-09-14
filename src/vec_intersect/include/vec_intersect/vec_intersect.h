/**
 * This file is part of the foe_estimator package - MAVLab TU Delft
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

#ifndef CPP_FOE_H_
#define CPP_FOE_H_

#include <string>
#include <iostream>
#include <fstream>
#include <mutex>
#include <numeric>

#include "ros/ros.h"

// ROS messages
#include "dvs_of_msg/FlowPacketMsg.h"
#include "dvs_of_msg/FlowPacketMsgArray.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "vec_intersect/FoE.h"
#include "std_msgs/Int16.h"

// Required by FOE estimation
#include <cstddef>
#include <cstdlib>
//#include "estimateFoECPP.h"

#include "signum.h"
#include <random>

// Mutex to lock during looping through arrays
std::mutex prepMutex;

// FoE publisher and message
ros::Publisher FoE_pub;
vec_intersect::FoE FoE_msg;

std::vector<double> FoE_hist_x{};
std::vector<double> FoE_hist_y{};
double FoE_x;
double FoE_y;

int FOV_X = 240;
int FOV_Y = 180;

int min_vectors = 10;
int num_average = 5;
int AmountOfIterations = 100;
int MaxFailures = 1;
double rate_ = 1000;

double beforetime = 0;

// Processing rate (Hz) for FoE estimation
double period_ = 1.0 / rate_ * 1e9; // in nSec

// Optic flow packet structure
struct FlowPacket
{
    int x;
    int y;
    int t;

    double u;
    double v;
    double ru;
    double rv;
};

// Optic Flow arrays for storing temp and final OF vectors
std::vector<FlowPacket> myOF;
std::vector<FlowPacket> final_buffer;
std::vector<FlowPacket> myOFBuf;

// Initialize counters
uint64_t last_ts = 0;
double prev_time = 0.0;

// Logging function for incoming OF
void log_OF(std::vector<FlowPacket> *myOF);

// Array for storing OF (COLUMN MAJOR)
static std::vector<FlowPacket> fillOpticFlowArray();

void estimateFoECPP(std::vector<FlowPacket> OpticFlow, double *FoE_x, double *FoE_y);
std::ofstream FoE_rec_file;
std::ofstream FoE_flow_rec_file;
std::ofstream HP_log_file;
std::ofstream HL_log_file;
std::ofstream OT_log_file;

std::ofstream timelog;


inline const std::string currentDateTime(void)
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y_%m_%d-%H_%M_%S", &tstruct);
    return buf;
}

void optitrackCallback(const geometry_msgs::PoseStamped::ConstPtr &msg); // gets the optic flow from the


#endif // CPP_FOE_H_