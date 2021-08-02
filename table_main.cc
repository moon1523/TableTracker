// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "TableTracker.hh"
#include "CharucoSync.hh"
#include "Kinect.hh"
#include <malloc.h>

int TABLE_TRACKING(int argc, char** argv);

void PrintUsage()
{
    cout << ">> Usage" << endl;
    cout << "  ./table_tracker -track" << endl;
    cout << "  ./table_tracker -sync  [outputfile]" << endl;
    cout << "  ./table_tracker -psync [record.mkv]" << endl;
    cout << "  ./table_tracker -play  [record.mkv]" << endl << endl;
}

int main(int argc, char** argv)
{
	PrintUsage();
	if(argc > 1 && string(argv[1])=="-track")
		return TABLE_TRACKING(argc, argv);
	else if(argc > 1 && string(argv[1])=="-sync")
		return CHARUCO_SYNC(argc,argv);
	else if(argc > 1 && string(argv[1])=="-psync")
		return PLAYBACK_RECORD_CHARUCO_SYNC(argv[2]);
	else if(argc > 1 && string(argv[1])=="-play")
		return PLAYBACK_RECORD(argv[2]);
	else
		cout << "Check the Argument" << endl;

	return EXIT_SUCCESS;
}

int TABLE_TRACKING(int argc, char** argv)
{
	cout << "### TABLE TRACKING" << endl;
	auto qtData = ReadCharucoData("./Coord/20210730_LAB");

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
	k4a_image_t color_image = NULL;
	k4a_image_t depth_image = NULL;
	k4a_image_t point_image = NULL;
	k4a_image_t colorlike_depth_image = NULL;
	k4a_transformation_t k4aTransform = NULL;
	k4aTransform = k4a_transformation_create(&sensorCalibration);

	Mat xy_table = create_color_xy_table(sensorCalibration);
	TableTracker tableTracker((int)PatientTable::TestDevice, xy_table, qtData.first, qtData.second);


	bool isCenter(false);
	cout << ">> Table Tracking" << endl;

	while(1)
	{
		k4a_device_get_capture(device, &sensorCapture, 1000);
		color_image = k4a_capture_get_color_image(sensorCapture);
		depth_image = k4a_capture_get_depth_image(sensorCapture);
		point_image = create_point_cloud_based(color_image);
		colorlike_depth_image = create_depth_image_like(color_image);
		k4a_transformation_depth_image_to_color_camera(k4aTransform, depth_image, colorlike_depth_image);
		k4a_transformation_depth_image_to_point_cloud(k4aTransform, colorlike_depth_image, K4A_CALIBRATION_TYPE_COLOR, point_image);

		tableTracker.SetNewImage(color_image, point_image);
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
			transformation_helpers_write_point_cloud(point_image, color_image, "pcd.ply");

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

		k4a_image_release(color_image);
		k4a_image_release(depth_image);
		k4a_image_release(colorlike_depth_image);
		k4a_image_release(point_image);
		k4a_capture_release(sensorCapture);
	}

	k4a_transformation_destroy(k4aTransform);
	k4a_device_close(device);
	cout << ">> EXIT_SUCCESS" << endl;

	return EXIT_SUCCESS;
}

