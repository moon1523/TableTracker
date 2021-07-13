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
#include <k4arecord/playback.h>
#include <Utilities.h>
#include <turbojpeg.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


using namespace std;

static k4a_device_configuration_t get_default_config()
{
	k4a_device_configuration_t camera_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	camera_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	camera_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
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
static k4a_image_t create_point_cloud_based(const k4a_image_t im)
{
	k4a_image_t img;

	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
					 k4a_image_get_width_pixels(im),
					 k4a_image_get_height_pixels(im),
					 k4a_image_get_width_pixels(im) * 3 * (int)sizeof(int16_t),
					 &img);
	return img;
}




static cv::Mat create_color_xy_table(const k4a_calibration_t calibration)
{
	k4a_float2_t p;
	k4a_float3_t ray;

	int width = calibration.color_camera_calibration.resolution_width;
	int height = calibration.color_camera_calibration.resolution_height;

	cv::Mat xy_table = cv::Mat::zeros(height,width, CV_32FC2);
	float* xy_table_data = (float*)xy_table.data;
	int valid;

	for (int y = 0, idx = 0; y < height; y++)
	{
		p.xy.y = (float)y;
		for (int x = 0; x < width; x++, idx++)
		{
			p.xy.x = (float)x;
			k4a_calibration_2d_to_3d(&calibration, &p, 1.f, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &ray, &valid);
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

static cv::Mat create_depth_xy_table(const k4a_calibration_t calibration)
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

struct color_point_t
{
    int16_t xyz[3];
    uint8_t rgb[3];
};

void tranformation_helpers_write_point_cloud(const k4a_image_t point_cloud_image,
                                             const k4a_image_t color_image,
                                             const char *file_name)
{
    std::vector<color_point_t> points;

    int width = k4a_image_get_width_pixels(point_cloud_image);
    int height = k4a_image_get_height_pixels(color_image);

    int16_t *point_cloud_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_cloud_image);
    uint8_t *color_image_data = k4a_image_get_buffer(color_image);

    for (int i = 0; i < width * height; i++)
    {
        color_point_t point;
        point.xyz[0] = point_cloud_image_data[3 * i + 0];
        point.xyz[1] = point_cloud_image_data[3 * i + 1];
        point.xyz[2] = point_cloud_image_data[3 * i + 2];
        if (point.xyz[2] == 0)
        {
            continue;
        }

        point.rgb[0] = color_image_data[4 * i + 0];
        point.rgb[1] = color_image_data[4 * i + 1];
        point.rgb[2] = color_image_data[4 * i + 2];
        uint8_t alpha = color_image_data[4 * i + 3];

        if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0)
        {
            continue;
        }

        points.push_back(point);
    }

#define PLY_START_HEADER "ply"
#define PLY_END_HEADER "end_header"
#define PLY_ASCII "format ascii 1.0"
#define PLY_ELEMENT_VERTEX "element vertex"

    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << PLY_START_HEADER << std::endl;
    ofs << PLY_ASCII << std::endl;
    ofs << PLY_ELEMENT_VERTEX << " " << points.size() << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "property uchar red" << std::endl;
    ofs << "property uchar green" << std::endl;
    ofs << "property uchar blue" << std::endl;
    ofs << PLY_END_HEADER << std::endl;
    ofs.close();

    std::stringstream ss;
    for (size_t i = 0; i < points.size(); ++i)
    {
        // image data is BGR
        ss << (float)points[i].xyz[0] << " " << (float)points[i].xyz[1] << " " << (float)points[i].xyz[2];
        ss << " " << (float)points[i].rgb[2] << " " << (float)points[i].rgb[1] << " " << (float)points[i].rgb[0];
        ss << std::endl;
    }
    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

static bool point_cloud_depth_to_color(k4a_transformation_t transformation_handle,
                                       const k4a_image_t depth_image,
                                       const k4a_image_t color_image)
{
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);

    k4a_image_t transformed_depth_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * (int)sizeof(uint16_t),
                                                 &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        return false;
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * 3 * (int)sizeof(int16_t),
                                                 &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
                                                                              transformed_depth_image,
                                                                              K4A_CALIBRATION_TYPE_COLOR,
                                                                              point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

//    tranformation_helpers_write_point_cloud(point_cloud_image, color_image, file_name.c_str());

    k4a_image_release(transformed_depth_image);
    k4a_image_release(point_cloud_image);

    return true;
}

static bool point_cloud_depth_to_color(k4a_transformation_t transformation_handle,
                                       const k4a_image_t depth_image,
                                       const k4a_image_t color_image,
									   string file_name)
{
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);

    k4a_image_t transformed_depth_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * (int)sizeof(uint16_t),
                                                 &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        return false;
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * 3 * (int)sizeof(int16_t),
                                                 &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
                                                                              transformed_depth_image,
                                                                              K4A_CALIBRATION_TYPE_COLOR,
                                                                              point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

    tranformation_helpers_write_point_cloud(point_cloud_image, color_image, file_name.c_str());

    k4a_image_release(transformed_depth_image);
    k4a_image_release(point_cloud_image);

    return true;
}

pair<Quaternionf, Vector3f> ReadCharucoData(string fileName)
{
	// Read World Coordinate
	Quaternionf quat;
	Vector3f tvec;
	ifstream ifs(fileName);
	string dump;

	if(!ifs.is_open()) {
		cerr << "fail to read" << endl;
		exit(1);
	}

	while(getline(ifs,dump)) {
		stringstream ss(dump);
		ss >> dump;
		float x,y,z,w;
		if (dump == "q") {
			ss >> x >> y >> z >> w;
			quat = Quaternionf(w,x,y,z);
		}
		else if (dump =="t") {
			ss >> x >> y >> z;
			tvec << x, y, z;
		}
	}
	return make_pair(quat, tvec);
}

int CHARUCO_SYNC(int argc, char** argv)
{
	//tracking option configuration3
	string detParm("detector_params.yml");
	string camParm("kinect2160.yml");

	// Start camera
	k4a_device_t device = nullptr;
	VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

	// Start camera. Make sure depth camera is enabled.
	k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_2160P;
	deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED; // No need for depth during calibration
	deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_15;     // Don't use all USB bandwidth
	VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

	// Get calibration information
	k4a_calibration_t sensorCalibration;
	VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
		   "Get depth camera calibration failed!");
//	int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
//	int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

	// Synchronization
	CharucoSync sync;
	sync.SetParameters(camParm, detParm);
	sync.SetScalingFactor(0.4f);
	bool getData(false);
	waitKey(1000);
	while (1)
	{
		k4a_capture_t sensorCapture = nullptr;
		k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0);

		if (getCaptureResult == K4A_WAIT_RESULT_FAILED)
		{
			std::cout << "Get img capture returned error: " << getCaptureResult << std::endl;
			break;
		}
		else if (getCaptureResult == K4A_WAIT_RESULT_TIMEOUT)
			continue;

		Mat color;
		Vec3d rvec, tvec;
		k4a_image_t color_img = k4a_capture_get_color_image(sensorCapture);
		color = color_to_opencv(color_img);
		k4a_image_release(color_img);
		k4a_capture_release(sensorCapture);
		sync.EstimatePose(color, rvec, tvec);

		sync.Render();
		char key = waitKey(1);
		if (key == 'q')
			break;
		else if (key == 'a')
		{
			sync.ShowAvgValue(color);
			char key2 = waitKey(0);
			if(key2=='q') break;
		}
		else if (key == 'c')
			sync.ClearData();
		else if (key == 'g')
			sync.TickSwitch();
	}
	k4a_device_close(device);
	sync.WriteTransformationData(string(argv[2]));

	return EXIT_SUCCESS;
}

int PLAYBACK_RECORD_CHARUCO_SYNC(char* fileName)
{
	cout << ">> Playback Record CHARUCO_SYNC" << endl;
	int timestamp = 1000;
	k4a_playback_t playback = NULL;
	k4a_calibration_t sensorCalibration;
	k4a_transformation_t transformation = NULL;
	k4a_capture_t capture = NULL;
	k4a_image_t color_image = NULL;
	k4a_image_t uncompressed_color_image = NULL;

	k4a_result_t result;
	k4a_stream_result_t stream_result;

	result = k4a_playback_open(fileName, &playback);
	if (result != K4A_RESULT_SUCCEEDED || playback == NULL) {
		printf("Failed to open recording %s\n", fileName); exit(1);
	}

	result = k4a_playback_seek_timestamp(playback, timestamp * 1000, K4A_PLAYBACK_SEEK_BEGIN);
	if (result != K4A_RESULT_SUCCEEDED)	{
		printf("Failed to seek timestamp %d\n", timestamp); exit(1);
	}
	printf("   Seeking to timestamp: %d/%d (ms)\n",
		   timestamp,
		   (int)(k4a_playback_get_recording_length_usec(playback) / 1000));

	if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &sensorCalibration)) {
		printf("Failed to get calibration\n"); exit(1);
	}
	cout << "   Record Length (s): " << (k4a_playback_get_recording_length_usec(playback) / (float)1000000) << endl;
	transformation = k4a_transformation_create(&sensorCalibration);
	cout << "   Color resolution: " << sensorCalibration.color_camera_calibration.resolution_width << " x "
								    << sensorCalibration.color_camera_calibration.resolution_height << endl;
	cout << "   Depth resolution: " << sensorCalibration.depth_camera_calibration.resolution_width << " x "
								    << sensorCalibration.depth_camera_calibration.resolution_height << endl << endl;

	//tracking option configuration3
	string detParm("detector_params.yml");
	string camParm("kinect2160.yml");

	// Synchronization
	CharucoSync sync;
	sync.SetParameters(camParm, detParm);
	sync.SetScalingFactor(0.4f);
	bool getData(false);
	waitKey(1000);
	while (1)
	{
		stream_result = k4a_playback_get_next_capture(playback, &capture);
		if (stream_result == K4A_STREAM_RESULT_EOF) {
			cout << "   Last capture" << endl; break;
		}
		color_image = k4a_capture_get_color_image(capture);
		// Convert color frame from mjpeg to bgra
		k4a_image_format_t format;
		format = k4a_image_get_format(color_image);
		if (format != K4A_IMAGE_FORMAT_COLOR_MJPG) {
			printf("Color format not supported. Please use MJPEG\n"); exit(1);
		}

		int color_width, color_height;
		color_width = k4a_image_get_width_pixels(color_image);
		color_height = k4a_image_get_height_pixels(color_image);

		if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
													 color_width,
													 color_height,
													 color_width * 4 * (int)sizeof(uint8_t),
													 &uncompressed_color_image))
		{
			printf("Failed to create image buffer\n"); exit(1);
		}

		tjhandle tjHandle;
		tjHandle = tjInitDecompress();
		if (tjDecompress2(tjHandle,
						  k4a_image_get_buffer(color_image),
						  static_cast<unsigned long>(k4a_image_get_size(color_image)),
						  k4a_image_get_buffer(uncompressed_color_image),
						  color_width,
						  0, // pitch
						  color_height,
						  TJPF_BGRA,
						  TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0)
		{
			printf("Failed to decompress color frame\n");
			if (tjDestroy(tjHandle))
			{
				printf("Failed to destroy turboJPEG handle\n");
			}
			exit(1);
		}
		if (tjDestroy(tjHandle))
		{
			printf("Failed to destroy turboJPEG handle\n");
		}


		Mat color;
		Vec3d rvec, tvec;
		color = color_to_opencv(uncompressed_color_image);
		k4a_image_release(uncompressed_color_image);
		k4a_capture_release(capture);
		sync.EstimatePose(color, rvec, tvec);

		sync.Render();
		char key = waitKey(1);
		if (key == 'q')
			break;
		else if (key == 'a')
		{
			sync.ShowAvgValue(color);
			char key2 = waitKey(0);
			if(key2=='q') break;
		}
		else if (key == 'c')
			sync.ClearData();
		else if (key == 'g')
			sync.TickSwitch();
	}
	k4a_playback_close(playback);
	sync.WriteTransformationData("test");

	return EXIT_SUCCESS;
}



int PLAYBACK_RECORD(char* fileName)
{
	cout << ">> Playback Record" << endl;
	int timestamp = 1000;
	k4a_playback_t playback = NULL;
	k4a_calibration_t calibration;
	k4a_transformation_t transformation = NULL;
	k4a_capture_t capture = NULL;
	k4a_image_t depth_image = NULL;
	k4a_image_t color_image = NULL;
	k4a_image_t point_image = NULL;
	k4a_image_t uncompressed_color_image = NULL;
	k4a_image_t colorlike_depth_image = NULL;

	k4a_result_t result;
	k4a_stream_result_t stream_result;

	result = k4a_playback_open(fileName, &playback);
	if (result != K4A_RESULT_SUCCEEDED || playback == NULL) {
		printf("Failed to open recording %s\n", fileName); exit(1);
	}

	result = k4a_playback_seek_timestamp(playback, timestamp * 1000, K4A_PLAYBACK_SEEK_BEGIN);
	if (result != K4A_RESULT_SUCCEEDED)	{
		printf("Failed to seek timestamp %d\n", timestamp); exit(1);
	}
	printf("   Seeking to timestamp: %d/%d (ms)\n",
		   timestamp,
		   (int)(k4a_playback_get_recording_length_usec(playback) / 1000));

	if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &calibration)) {
		printf("Failed to get calibration\n"); exit(1);
	}
	cout << "   Record Length (s): " << (k4a_playback_get_recording_length_usec(playback) / (float)1000000) << endl;
	transformation = k4a_transformation_create(&calibration);
	cout << "   Color resolution: " << calibration.color_camera_calibration.resolution_width << " x "
								 << calibration.color_camera_calibration.resolution_height << endl;
	cout << "   Depth resolution: " << calibration.depth_camera_calibration.resolution_width << " x "
								 << calibration.depth_camera_calibration.resolution_height << endl << endl;


	auto qtData = ReadCharucoData("kinect");
	Mat xy_table = create_color_xy_table(calibration);
	TableTracker tableTracker(xy_table, qtData.first, qtData.second);
	cv::Vec3d ocrDat(0,0,0); // lat, long, height

	bool isCenter(false);
	cout << ">> Stream loop start !!" << endl;
	while(1)
	{
		stream_result = k4a_playback_get_next_capture(playback, &capture);

		if (stream_result == K4A_STREAM_RESULT_EOF) {
			cout << "   Last capture" << endl;
			result = k4a_playback_seek_timestamp(playback, timestamp * 1000, K4A_PLAYBACK_SEEK_BEGIN);
		}
		// Fetch frame
		depth_image = k4a_capture_get_depth_image(capture);
		color_image = k4a_capture_get_color_image(capture);

		// Convert color frame from mjpeg to bgra
		k4a_image_format_t format;
		format = k4a_image_get_format(color_image);
		if (format != K4A_IMAGE_FORMAT_COLOR_MJPG) {
			printf("Color format not supported. Please use MJPEG\n"); exit(1);
		}

		int color_width, color_height;
		color_width = k4a_image_get_width_pixels(color_image);
		color_height = k4a_image_get_height_pixels(color_image);

		if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
													 color_width,
													 color_height,
													 color_width * 4 * (int)sizeof(uint8_t),
													 &uncompressed_color_image))
		{
			printf("Failed to create image buffer\n"); exit(1);
		}

		tjhandle tjHandle;
		tjHandle = tjInitDecompress();
		if (tjDecompress2(tjHandle,
						  k4a_image_get_buffer(color_image),
						  static_cast<unsigned long>(k4a_image_get_size(color_image)),
						  k4a_image_get_buffer(uncompressed_color_image),
						  color_width,
						  0, // pitch
						  color_height,
						  TJPF_BGRA,
						  TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0)
		{
			printf("Failed to decompress color frame\n");
			if (tjDestroy(tjHandle))
			{
				printf("Failed to destroy turboJPEG handle\n");
			}
			exit(1);
		}
		if (tjDestroy(tjHandle))
		{
			printf("Failed to destroy turboJPEG handle\n");
		}

		// Compute color point cloud by warping depth image into color camera geometry
//		string outFile = "./plytest/pcd"+to_string(idx++) + ".ply";
//		cout << outFile << endl;
//		if (point_cloud_depth_to_color(transformation, depth_image, uncompressed_color_image, outFile) == false)
//		{
//			printf("Failed to transform depth to color\n");
//			exit(1);
//		}

		point_image = create_point_cloud_based(uncompressed_color_image);
		colorlike_depth_image = create_depth_image_like(uncompressed_color_image);
		k4a_transformation_depth_image_to_color_camera(transformation, depth_image, colorlike_depth_image);
		k4a_transformation_depth_image_to_point_cloud(transformation, colorlike_depth_image, K4A_CALIBRATION_TYPE_COLOR, point_image);

		tableTracker.SetNewImage(uncompressed_color_image, point_image);
		if(!isCenter)
		{
			isCenter = tableTracker.FindTableCenter();
		}
		else
		{
			tableTracker.TransTableOrigin(ocrDat(0), ocrDat(1), ocrDat(2));
			tableTracker.ProcessCurrentFrame();
			tableTracker.Render();
		}


		char key = (char)waitKey(1);
		if (key == 'g') {
			cout << "Generate Point Cloud Data" << endl;

		}
		else if (key == 's') {
			cout <<" Set table origin " << endl;
			float x,y,z;
			cin >> x >> y >> z;
			tableTracker.TransTableOrigin(x, y, z);
		}
		else if (key == 'q') {
			break;
		}

		k4a_capture_release(capture);
		k4a_image_release(color_image);
		k4a_image_release(depth_image);
		k4a_image_release(uncompressed_color_image);
		k4a_image_release(point_image);
		k4a_image_release(colorlike_depth_image);
	}
	k4a_playback_close(playback);

	return EXIT_SUCCESS;
}
