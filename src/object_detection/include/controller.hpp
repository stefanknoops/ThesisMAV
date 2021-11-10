/**
 * This file is part of the object_detection package - MAVLab TU Delft
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

#pragma once
#include <thread>
#include <math.h>
#include <ros/ros.h>

#include <mutex>

#include <iostream>
#include <fstream>

#include "std_msgs/Int32.h"

// Required by clustering algorithm
#include <cstddef>
#include <cstdlib>
#include "prepEstimateClustersCPP.h"

#include "EstimateClusterTTC.h"

#include "CollisionCheck.h"

// DBSCAN header
#include "dbscan.h"

// struct FlowPacket
// {
//     int x;
//     int y;
//     int t;

//     float u;
//     float v;
// };

class Controller
{
private:
    std::thread control_job_;

public:
    int cnt;

    // Initialize counters
    int64_t last_ts = 0;
    double prev_roll = 0;

    Controller();
    ~Controller();

    void control_job();
    void clusterServer();

    // DBSCAN data struct
    struct vec2f
    {
        float data[2];
        float operator[](int idx) const { return data[idx]; } ;
    };

    // Array for storing OF (COLUMN MAJOR)
    std::vector<FlowPacket> fillOpticFlowArray();
    std::vector<Controller::vec2f> fillDBSCANdata();

    // MATLAB array for storing cluster Idx

    // Record clustering
    std::ofstream Cluster_rec_file;

    // Prep arrays for detection function
    std::vector<FlowPacket> optic_flow;
    std::vector<double> x_norm;
    std::vector<double> y_norm;
    std::vector<double> u_norm;
    std::vector<double> v_norm;
    std::vector<double> TTC;
    std::vector<double> ang_norm;
    std::vector<double> TTC_norm;

    std::vector<std::vector<double>> boxes;

    void log_OF(std::vector<FlowPacket> *myOF);
    std::vector<FlowPacket> final_buffer;
    std::vector<FlowPacket> myOF;
    std::vector<FlowPacket> myOFBuf;

    // Mutex to lock during looping through arrays
    std::mutex prepMutex;

    // FoE message
    std_msgs::Int32 FoE_msg;
    int32_t FoE_x;

    ros::Publisher roll_pub;
    std_msgs::Int32 roll_msg;

    double roll_command = 0;
    double col_timer = 0;
    bool rolling = false;
};
