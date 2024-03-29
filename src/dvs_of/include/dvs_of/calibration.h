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

#ifndef CALIBRATION_H_
#define CALIBRATION_H_
#include <string>

#include <cinttypes>

#include <ros/ros.h>
#include "dvs_of/calibDVS240_sim.h"
#include "dvs_of/calibDVS240_222.h"
#define DVS_N_PIXELS_X 240
#define DVS_N_PIXELS_Y 180

namespace dvs_of {

    /**
     * Returns an undistorted pixel X-coordinate based on the
     * dvsCalibration struct.
     * Includes bounds checking for input X- and Y- addresses.
     *
     * @param x X-address of pixel
     * @param y Y-address of pixel
     * @return Undistorted pixel location
     */
    float dvsGetUndistortedPixelX(uint16_t x, uint16_t y);

    /**
     * Returns an undistorted pixel Y-coordinate based on the
     * dvsCalibration struct.
     * Includes bounds checking for input X- and Y- addresses.
     *
     * @param x X-address of pixel
     * @param y Y-address of pixel 
     * @return Undistorted pixel location
     */
    float dvsGetUndistortedPixelY(uint16_t x, uint16_t y);

    static std::vector<std::vector<float>> undistortionMapX;
    static std::vector<std::vector<float>> undistortionMapY;
    void initializeUmap(std::string calib);

    //float undistortionMapX[DVS_N_PIXELS_X][DVS_N_PIXELS_Y];
    //float undistortionMapY[DVS_N_PIXELS_X][DVS_N_PIXELS_Y];
} // namespace

#endif // CALIBRATION_H_