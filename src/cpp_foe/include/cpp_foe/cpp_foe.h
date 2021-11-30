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

#include "ros/ros.h"

// ROS messages
#include "dvs_of_msg/FlowPacketMsg.h"
#include "dvs_of_msg/FlowPacketMsgArray.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int64MultiArray.h"


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
std_msgs::Int64MultiArray FoE_msg;
double FoE_x;
double FoE_y;

// Optic flow packet structure
struct FlowPacket {
    int x;
    int y;
    int t;

    double u;
    double v;

};

// Optic Flow arrays for storing temp and final OF vectors
std::vector<FlowPacket> myOF;
std::vector<FlowPacket> final_buffer;
std::vector<FlowPacket> myOFBuf;

// Initialize counters 
uint64_t last_ts = 0;
double prev_time = 0.0;

// Processing rate (Hz) for FoE estimation
double rate_ = 40;
double period_ = 1.0 / rate_;

// Logging function for incoming OF
void log_OF(std::vector<FlowPacket> *myOF);

// Array for storing OF (COLUMN MAJOR)
static std::vector<FlowPacket> fillOpticFlowArray();

void estimateFoECPP(std::vector<FlowPacket> OpticFlow, double  *FoE_x, double *FoE_y);


#endif // CPP_FOE_H_