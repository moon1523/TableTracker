#ifndef FUNCTIONS_HH_
#define FUNCTIONS_HH_

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include <algorithm>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <limits>

#include <k4a/k4a.hpp>
#include <k4arecord/playback.h>
#include <Utilities.h>
#include <turbojpeg.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


using namespace std;
using namespace cv;
using namespace Eigen;

struct color_point_t
{
    int16_t xyz[3];
    uint8_t rgb[3];
};

k4a_device_configuration_t get_default_config();
cv::Mat color_to_opencv(const k4a_image_t im);
cv::Mat depth_to_opencv(const k4a_image_t im);
k4a_image_t color_to_depth(k4a_transformation_t transformation_handle,
							   const k4a_image_t depth_image,
							   const k4a_image_t color_image);
k4a_image_t create_depth_image_like(const k4a_image_t im);
k4a_image_t create_color_image_like(const k4a_image_t im);
k4a_image_t create_point_cloud_based_color(const k4a_image_t im);
k4a_image_t create_point_cloud_based_depth(const k4a_image_t im);
k4a_image_t Convert_Color_MJPG_To_BGRA(k4a_image_t color_image);

class Timer
{
public:
    Timer() : start_(0), time_(0) {}

    void start() {
    	start_ = cv::getTickCount();
    }
    void stop() {
        CV_Assert(start_ != 0);
        int64 end = cv::getTickCount();
        time_ += end - start_;
        start_ = 0;
    }

    double time() {
        double ret = time_ / cv::getTickFrequency();
        time_ = 0;
        return ret;
    }

private:
    int64 start_, time_;
};







#endif
