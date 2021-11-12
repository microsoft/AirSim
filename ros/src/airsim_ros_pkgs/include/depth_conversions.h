/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#ifndef DEPTH_IMAGE_PROC_DEPTH_CONVERSIONS
#define DEPTH_IMAGE_PROC_DEPTH_CONVERSIONS

#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_traits.h>

#include <limits>

namespace depth_image_proc {

    typedef sensor_msgs::PointCloud2 PointCloud;

// Handles float or uint16 depths
    template<typename T>
    void convert(
            const sensor_msgs::ImageConstPtr &depth_msg,
            PointCloud::Ptr &cloud_msg,
            const image_geometry::PinholeCameraModel &model,
            double range_max = 0.0) {
        // Use correct principal point from calibration
        float center_x = model.cx();
        float center_y = model.cy();

        // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
        double unit_scaling = DepthTraits<T>::toMeters(T(1));
        float constant_x = unit_scaling / model.fx();
        float constant_y = unit_scaling / model.fy();
        float bad_point = std::numeric_limits<float>::quiet_NaN();

        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
        const T *depth_row = reinterpret_cast<const T *>(&depth_msg->data[0]);
        int row_step = depth_msg->step / sizeof(T);
        for (int v = 0; v < (int) cloud_msg->height; ++v, depth_row += row_step) {
            for (int u = 0; u < (int) cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z) {
                T depth = depth_row[u];

                // Missing points denoted by NaNs
                if (!DepthTraits<T>::valid(depth)) {
                    if (range_max != 0.0) {
                        depth = DepthTraits<T>::fromMeters(range_max);
                    } else {
                        *iter_x = *iter_y = *iter_z = bad_point;
                        continue;
                    }
                }

                // Fill in XYZ
                *iter_x = (u - center_x) * depth * constant_x;
                *iter_y = (v - center_y) * depth * constant_y;
                *iter_z = DepthTraits<T>::toMeters(depth);
            }
        }
    }


/**
 * Copy and paste
 */

    template<typename T>
    void convert(const sensor_msgs::ImageConstPtr &depth_msg,
                 const sensor_msgs::ImageConstPtr &rgb_msg,
                 const sensor_msgs::PointCloud2::Ptr &cloud_msg,
                 int red_offset, int green_offset, int blue_offset, int color_step,
                 image_geometry::PinholeCameraModel model_) {


        // Use correct principal point from calibration
        float center_x = model_.cx();
        float center_y = model_.cy();

        // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
        double unit_scaling = DepthTraits<T>::toMeters(T(1));
        float constant_x = unit_scaling / model_.fx();
        float constant_y = unit_scaling / model_.fy();
        float bad_point = std::numeric_limits<float>::quiet_NaN();

        const T *depth_row = reinterpret_cast<const T *>(&depth_msg->data[0]);
        int row_step = depth_msg->step / sizeof(T);
        const uint8_t *rgb = &rgb_msg->data[0];
        int rgb_skip = rgb_msg->step - rgb_msg->width * color_step;

        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(*cloud_msg, "a");

        for (int v = 0; v < int(cloud_msg->height); ++v, depth_row += row_step, rgb += rgb_skip) {
            for (int u = 0; u <
                            int(cloud_msg->width); ++u, rgb += color_step, ++iter_x, ++iter_y, ++iter_z, ++iter_a, ++iter_r, ++iter_g, ++iter_b) {
                T depth = depth_row[u];

                // Check for invalid measurements
                if (!DepthTraits<T>::valid(depth)) {
                    *iter_x = *iter_y = *iter_z = bad_point;
                } else {
                    // Fill in XYZ
                    *iter_x = (u - center_x) * depth * constant_x;
                    *iter_y = (v - center_y) * depth * constant_y;
                    *iter_z = DepthTraits<T>::toMeters(depth);
                }

                // Fill in color
                *iter_a = 255;
                *iter_r = rgb[red_offset];
                *iter_g = rgb[green_offset];
                *iter_b = rgb[blue_offset];
            }
        }
    }



} // namespace depth_image_proc




#endif
