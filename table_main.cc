// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "TableTracker.hh"
#include "CharucoSync.hh"
#include "Kinect.hh"
#include <Utilities.h>


int CHARUCO_SYNC(int argc, char** argv);
pair<Quaternionf, Vector3f> ReadCharucoData(string fileName);

void PrintUsage()
{
    cout << ">> Usage" << endl;
    cout << "  ./table_tracker" << endl;
    cout << "  ./table_tracker -sync [outputfile]" << endl;
}

int main(int argc, char** argv)
{
	PrintUsage();
	if(argc > 1 && string(argv[1])=="-sync") return CHARUCO_SYNC(argc,argv);

	auto qtData = ReadCharucoData("kinect");

	// Read OCR data
	cv::Vec3d ocrDat(0,0,0); // lat, long, height

	// Start camera
	k4a_device_t device = nullptr;
	VERIFY(k4a_device_open(K4A_DEVICE_DEFAULT, &device), "Open K4A Device failed!");

	// Start camera. Make sure depth camera is enabled.
	k4a_device_configuration_t deviceConfig = get_default_config();
	VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

	// Get calibration information
	k4a_calibration_t sensorCalibration;
	VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
		   "Get depth camera calibration failed!");

	k4a_capture_t sensorCapture = NULL;
	k4a_image_t color_img = NULL;
	k4a_image_t depth_img = NULL;
	k4a_image_t point_img = NULL;
	k4a_image_t colorlike_depth_img = NULL;
	k4a_transformation_t k4aTransform = NULL;
	k4aTransform = k4a_transformation_create(&sensorCalibration);

	Mat xy_table = create_color_xy_table(sensorCalibration);
	TableTracker tableTracker(xy_table, qtData.first, qtData.second);

	//	tableTracker.Read_K4A_MKV_Record("rec0610.mkv", xy_table);

	bool isCenter(false);
	cout << ">> Main Loop Start" << endl;
	while(1)
	{
		k4a_device_get_capture(device, &sensorCapture, 1000);
		color_img = k4a_capture_get_color_image(sensorCapture);
		depth_img = k4a_capture_get_depth_image(sensorCapture);
		point_img = create_point_cloud_based(color_img);
		colorlike_depth_img = create_depth_image_like(color_img);
		k4a_transformation_depth_image_to_color_camera(k4aTransform, depth_img, colorlike_depth_img);
		k4a_transformation_depth_image_to_point_cloud(k4aTransform, colorlike_depth_img, K4A_CALIBRATION_TYPE_COLOR, point_img);

		tableTracker.SetNewImage(color_img, point_img);
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

		k4a_image_release(color_img);
		k4a_image_release(depth_img);
		k4a_image_release(colorlike_depth_img);
		k4a_image_release(point_img);
		k4a_capture_release(sensorCapture);
	}

	k4a_transformation_destroy(k4aTransform);
	k4a_device_close(device);
	cout << ">> EXIT_SUCCESS" << endl;

	return EXIT_SUCCESS;
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
	deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_15;
	deviceConfig.subordinate_delay_off_master_usec = 0;
	deviceConfig.synchronized_images_only = true;
	VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

	// Get calibration information
	k4a_calibration_t sensorCalibration;
	VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
		   "Get depth camera calibration failed!");
	int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
	int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

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
