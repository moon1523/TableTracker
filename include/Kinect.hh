#pragma once
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <limits>

#include <k4a/k4a.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

static k4a_device_configuration_t get_default_config()
{
	k4a_device_configuration_t camera_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	camera_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	camera_config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
	camera_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	camera_config.camera_fps = K4A_FRAMES_PER_SECOND_15;
	camera_config.subordinate_delay_off_master_usec = 0;
	camera_config.synchronized_images_only = true;

	return camera_config;
}

static cv::Mat color_to_opencv(const k4a_image_t im)
{
	cv::Mat cv_image_with_alpha(k4a_image_get_height_pixels(im), k4a_image_get_width_pixels(im), CV_8UC4, (void*)k4a_image_get_buffer(im));
    cv::Mat cv_image_no_alpha;
    cv::cvtColor(cv_image_with_alpha, cv_image_no_alpha, cv::COLOR_BGRA2BGR);
    return cv_image_no_alpha;
}

static cv::Mat depth_to_opencv(const k4a_image_t im)
{
    return cv::Mat(k4a_image_get_height_pixels(im),
    			   k4a_image_get_width_pixels(im),
				   CV_16U,
        (void*)k4a_image_get_buffer(im),
        static_cast<size_t>(k4a_image_get_stride_bytes(im)));
}



static k4a_image_t color_to_depth(k4a_transformation_t transformation_handle,
									   const k4a_image_t depth_image,
									   const k4a_image_t color_image)
{
    int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
    int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
    k4a_image_t transformed_color_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                 depth_image_width_pixels,
                                                 depth_image_height_pixels,
                                                 depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
                                                 &transformed_color_image))
    {
        printf("Failed to create transformed color image\n");
        exit(1);
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation_handle,
                                                                               depth_image,
                                                                               color_image,
                                                                               transformed_color_image))
    {
        printf("Failed to compute transformed color image\n");
        exit(1);
    }

    return transformed_color_image;
}

static k4a_image_t create_depth_image_like(const k4a_image_t im)
{
	k4a_image_t img;
	k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
					 k4a_image_get_width_pixels(im),
					 k4a_image_get_height_pixels(im),
					 k4a_image_get_width_pixels(im) * static_cast<int>(sizeof(uint16_t)),
					 &img);
	return img;
}

static k4a_image_t create_color_image_like(const k4a_image_t im)
{
	k4a_image_t img;
	k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
					 k4a_image_get_width_pixels(im),
					 k4a_image_get_height_pixels(im),
					 k4a_image_get_width_pixels(im) * 4 * static_cast<int>(sizeof(uint8_t)),
					 &img);
	return img;
}




static cv::Mat create_xy_table(const k4a_calibration_t calibration)
{
	k4a_float2_t p;
	k4a_float3_t ray;

	int width = calibration.depth_camera_calibration.resolution_width;
	int height = calibration.depth_camera_calibration.resolution_height;

	cv::Mat xy_table = cv::Mat::zeros(height,width, CV_32FC2);
	float* xy_table_data = (float*)xy_table.data;
	int valid;

	for (int y = 0, idx = 0; y < height; y++)
	{
		p.xy.y = (float)y;
		for (int x = 0; x < width; x++, idx++)
		{
			p.xy.x = (float)x;
			k4a_calibration_2d_to_3d(&calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);
			if (valid) {
				xy_table_data[idx*2] = ray.xyz.x;
				xy_table_data[idx*2+1] = ray.xyz.y;
			} else {
				xy_table_data[idx*2] = nanf("");
				xy_table_data[idx*2+1] = nanf("");
			}
		}
	}
	return xy_table;
}
