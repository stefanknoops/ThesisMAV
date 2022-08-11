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

#include <dvs_of/calibration.h>

namespace dvs_of
{

    void initializeUmap(std::string calib)
    {
        if (calib == "cam_222")
        {
            ROS_INFO("Setting calibration to %s", calib.c_str());

            undistortionMapX = undistortionMapX_222;
            undistortionMapY = undistortionMapY_222;
            ROS_INFO("Calibration map successfully set");
        }
        else
        {
            ROS_INFO("Setting calibration to simulation distortion map");

            undistortionMapX = undistortionMapX_sim;
            undistortionMapY = undistortionMapY_sim;

            if (calib == "sim")
            {
                ROS_INFO("Calibration map successfully set");
            }
            else
            {
                ROS_WARN("Could not find calibration map. Set to simulation map instead");
            }
        }
    }

    float dvsGetUndistortedPixelX(uint16_t x, uint16_t y)
    {
        if (x < DVS_N_PIXELS_X && y < DVS_N_PIXELS_Y)
        {
            return undistortionMapX[x][y];
        }
        else
        {
            return 0.f;
        }
    }

    float dvsGetUndistortedPixelY(uint16_t x, uint16_t y)
    {
        if (x < DVS_N_PIXELS_X && y < DVS_N_PIXELS_Y)
        {
            return undistortionMapY[x][y];
        }
        else
        {
            return 0.f;
        }
    }

} // namespace
