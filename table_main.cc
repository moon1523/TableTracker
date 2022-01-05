// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "TableTracker.hh"
#include "CharucoSync.hh"
#include "Functions.hh"

int TABLE_TRACKING(int, char**);
int CHARUCO_SYNC(int, char**);
int RECORD_TRACKING(int, char**);
int RECORD_CHARUCO_SYNC(int, char**);

void PrintUsage()
{
    cout << "  >> Usage" << endl;
    cout << "    ./table_tracker -track  -c [config.yml] -o [output] (-v) (-m)" << endl;
    cout << "    ./table_tracker -rtrack -c [config.yml] -r [record.mkv] -o [output] (-v) (-m)" << endl;
    cout << "    ./table_tracker -sync   -o [output] -s [s_factor]" << endl;
    cout << "    ./table_tracker -rsync  -r [record.mkv] -o [output] -s [s_factor]" << endl << endl;

    cout << "    -v: color view" << endl;
    cout << "    -m: mask view" << endl;
    cout << "    -s: scaling factor" << endl;
    cout << "    [config.yml] default = ConfigData.yml" << endl;
    cout << "    [output]     default = results.out " << endl;
    cout << "    [s_factor]   default = 0.3" << endl;
    cout << "    [record.mkv] can be acquired by k4aviewer" << endl << endl;

    cout << "  >> Key down" << endl;
    cout << "     Tracking mode" << endl;
    cout << "       'g'     : Generate Point Cloud PLY File for the present frame (pcd, tf_pcd)" << endl;
    cout << "       'o', 'p': Decrease/Increase bot margin" << endl;
    cout << "       '[', ']': Decrease/Increase top margin" << endl;
    cout << "       's'     : Set OCR data manually" << endl;
    cout << "       'q'     : Exit" << endl;
    cout << "     ChArUco Sync mode" << endl;
    cout << "       'a'     : Show average data" << endl;
    cout << "       'c'     : Clear the accumulative data" << endl;
    cout << "       't'     : Start or Stop the data accumulation" << endl;
    cout << "       'q'     : Exit" << endl << endl;
}

int main(int argc, char** argv)
{
	PrintUsage();
	if(argc > 1 && string(argv[1])=="-track")
		return TABLE_TRACKING(argc, argv);
	else if(argc > 1 && string(argv[1])=="-sync")
		return CHARUCO_SYNC(argc, argv);
	else if(argc > 1 && string(argv[1])=="-rtrack")
		return RECORD_TRACKING(argc, argv);
	else if(argc > 1 && string(argv[1])=="-rsync")
		return RECORD_CHARUCO_SYNC(argc, argv);
	else
		cout << "Check the Argument" << endl;

	return EXIT_SUCCESS;
}

int TABLE_TRACKING(int argc, char** argv)
{
	string configName = "ConfigData_test.yml";
	string outputName = "results.out";
	bool isColor = false;
	bool isMask = false;
	for (int i=2; i<argc; i++) {
		if ( string(argv[i]) == "-c") {
			configName = argv[i+1];
			i++;
		}
		else if ( string(argv[i]) == "-o" ) {
			outputName = argv[i+1];
			i++;
		}
		else if ( string(argv[i]) == "-v" ) {
			isColor = true;
		}
		else if ( string(argv[i]) == "-m" ) {
			isMask = true;
		}
	}
	cout << "== TABLE TRACKING =======================" << endl;
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
	k4a_transformation_t transformation = NULL;
	transformation = k4a_transformation_create(&sensorCalibration);

	int image_width  = sensorCalibration.color_camera_calibration.resolution_width;
	int image_height = sensorCalibration.color_camera_calibration.resolution_height;

	TableTracker tableTracker("track.coord", configName, outputName, image_width,image_height);

	Vector3d ocr(0,0,0); // lat, long, height
	tableTracker.ColorView(isColor);
	tableTracker.MaskView(isMask);
	cout << "*** STREAM LOOP START !!" << endl;
	while(true)
	{
		Timer timer; timer.start();

		// 1. Capture image
		k4a_device_get_capture(device, &sensorCapture, 1000);
		depth_image = k4a_capture_get_depth_image(sensorCapture);
		k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, image_width, image_height, image_width * 3 * (int)sizeof(uint16_t), &point_image);
		k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, image_width, image_height, image_width * (int)sizeof(uint16_t), &colorlike_depth_image);
		k4a_transformation_depth_image_to_color_camera(transformation, depth_image, colorlike_depth_image);
		k4a_transformation_depth_image_to_point_cloud(transformation, colorlike_depth_image, K4A_CALIBRATION_TYPE_COLOR, point_image);

		// 2. Read OCR data

		// 3. Set OCR data
		tableTracker.SetOCR(ocr);

		// 4. Set Image
		if(isColor) {
			color_image = k4a_capture_get_color_image(sensorCapture);
			tableTracker.SetNewImage(color_image, depth_image, point_image);
		}
		else
			tableTracker.SetNewImage(depth_image, point_image);

		// 5. Process
		tableTracker.ProcessCurrentFrame();
		timer.stop();

		// 6. Render
		tableTracker.Render(timer.time());

		char key = (char)waitKey(1);
		if (key == 'g') {
			tableTracker.GeneratePointCloudFile();
		}
		else if (key == 'o') {
			tableTracker.DecreaseBotMargin();
		}
		else if (key == 'p') {
			tableTracker.IncreaseBotMargin();
		}
		else if (key == '[') {
			tableTracker.DecreaseTopMargin();
		}
		else if (key == ']') {
			tableTracker.IncreaseTopMargin();
		}
		else if (key == 's') {
			cout <<"Set the OCR data (lateral, longitudinal, height, unit: mm) " << endl;
			cin >> ocr(0) >> ocr(1) >> ocr(2);
		}
		else if (key == 'q') {
			break;
		}

		// 7. Release
		k4a_capture_release(sensorCapture);
		k4a_image_release(depth_image);
		k4a_image_release(colorlike_depth_image);
		k4a_image_release(point_image);

		if(isColor) k4a_image_release(color_image);
	}

	k4a_transformation_destroy(transformation);
	k4a_device_close(device);

	return EXIT_SUCCESS;
}

int RECORD_TRACKING(int argc, char** argv)
{
	char* recordName;
	string configName = "ConfigData.yml";
	string outputName = "results.out";
	bool isColor = false;
	bool isMask = false;
	for (int i=2; i<argc; i++) {
		if ( string(argv[i]) == "-c") {
			configName = argv[i+1];
			i++;
		}
		else if ( string(argv[i]) == "-r" ) {
			recordName = argv[i+1];
			i++;
		}
		else if ( string(argv[i]) == "-o" ) {
			outputName = argv[i+1];
			i++;
		}
		else if ( string(argv[i]) == "-v" ) {
			isColor = true;
		}
		else if ( string(argv[i]) == "-m" ) {
			isMask = true;
		}
	}


	cout << "== RECORD TRACKING =======================" << endl;
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

	result = k4a_playback_open(recordName, &playback);
	if (result != K4A_RESULT_SUCCEEDED || playback == NULL) {
		printf("Failed to open recording %s\n", recordName); exit(1);
	}

	result = k4a_playback_seek_timestamp(playback, timestamp * 1000, K4A_PLAYBACK_SEEK_BEGIN);
	if (result != K4A_RESULT_SUCCEEDED)	{
		printf("Failed to seek timestamp %d\n", timestamp); exit(1);
	}

	cout << "  >> Record Information" << endl;
	printf("     1) Seeking to timestamp: %d/%d (ms)\n",
		   timestamp,
		   (int)(k4a_playback_get_recording_length_usec(playback) / 1000));

	if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &calibration)) {
		printf("Failed to get calibration\n"); exit(1);
	}

	cout << "     2) Record Length (s): " << (k4a_playback_get_recording_length_usec(playback) / (float)1000000) << endl;
	transformation = k4a_transformation_create(&calibration);
	int image_width  = calibration.color_camera_calibration.resolution_width;
	int image_height = calibration.color_camera_calibration.resolution_height;

	cout << "     3) Color resolution: " << calibration.color_camera_calibration.resolution_width << " x "
								 << calibration.color_camera_calibration.resolution_height << endl;
	cout << "     4) Depth resolution: " << calibration.depth_camera_calibration.resolution_width << " x "
								 << calibration.depth_camera_calibration.resolution_height << endl << endl;

	TableTracker tableTracker("HYUMC.coord" ,configName, outputName, image_width, image_height);
//	Vector3d ocr(0,0,0); // unit: mm
	Vector3d ocr(-20,-140,0); // unit: mm
//	Vector3d ocr(-200,-200,0); // unit: mm
//	Vector3d ocr(-280,-960,0); // unit: mm

	tableTracker.ColorView(isColor);
	tableTracker.MaskView(isMask);
	cout << "*** STREAM LOOP START !!" << endl;
	while(true)
	{
		Timer timer; timer.start();

		// 1. Capture
		stream_result = k4a_playback_get_next_capture(playback, &capture);
		if (stream_result == K4A_STREAM_RESULT_EOF) {
			cout << "Last capture" << endl; break;
		}
		depth_image = k4a_capture_get_depth_image(capture);
		k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, image_width, image_height, image_width * 3 * (int)sizeof(uint16_t), &point_image);
		k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, image_width, image_height, image_width * (int)sizeof(uint16_t), &colorlike_depth_image);
		k4a_transformation_depth_image_to_color_camera(transformation, depth_image, colorlike_depth_image);
		k4a_transformation_depth_image_to_point_cloud(transformation, colorlike_depth_image, K4A_CALIBRATION_TYPE_COLOR, point_image);

		// 2. Read OCR data

		// 3. Set OCR data
		tableTracker.SetOCR(ocr);

		// 4. Set Image
		if(isColor) {
			color_image = k4a_capture_get_color_image(capture);
			uncompressed_color_image = Convert_Color_MJPG_To_BGRA(color_image);
			tableTracker.SetNewImage(uncompressed_color_image, depth_image, point_image);
		}
		else
			tableTracker.SetNewImage(depth_image, point_image);

		// 5. Process
		tableTracker.ProcessCurrentFrame();
		timer.stop();

		// 6. Render
		tableTracker.Render(timer.time());

		char key = (char)waitKey(1);
		if (key == 'g') {
			tableTracker.GeneratePointCloudFile();
		}
		else if (key == 'o') {
			tableTracker.DecreaseBotMargin();
		}
		else if (key == 'p') {
			tableTracker.IncreaseBotMargin();
		}
		else if (key == '[') {
			tableTracker.DecreaseTopMargin();
		}
		else if (key == ']') {
			tableTracker.IncreaseTopMargin();
		}
		else if (key == 's') {
			cout <<"Set the OCR data (lateral, longitudinal, height, unit: mm) " << endl;
			cin >> ocr(0) >> ocr(1) >> ocr(2);
		}
		else if (key == 'q') {
			break;
		}

		// 7. Release
		k4a_capture_release(capture);
		k4a_image_release(depth_image);
		k4a_image_release(colorlike_depth_image);
		k4a_image_release(point_image);

		if (isColor) {
			k4a_image_release(color_image);
			k4a_image_release(uncompressed_color_image);
		}

	}
	k4a_transformation_destroy(transformation);
	k4a_playback_close(playback);

	return EXIT_SUCCESS;
}

int CHARUCO_SYNC(int argc, char** argv)
{
	string outputName = "world_coord";
	float scalingFactor = 0.3;
	for (int i=2; i<argc; i++) {
		if ( string(argv[i]) == "-o" ) {
			outputName = argv[i+1];
			i++;
		}
		else if ( string(argv[i]) == "-s" ) {
			scalingFactor = atof(argv[i+1]);
		}
	}

	cout << "== CHARUO BOARD DETECTION =======================" << endl;
	//tracking option configuration
	string detParm("detector_params.yml");
	string camParm("kinect3072.yml");

	// Start camera
	k4a_device_t device = nullptr;
	VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

	// Start camera. Make sure depth camera is enabled.
	k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_3072P;
	deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED; // No need for depth during calibration
	deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_15;     // Don't use all USB bandwidth
	deviceConfig.subordinate_delay_off_master_usec = 0;
	deviceConfig.synchronized_images_only = true;
	VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

	// Get calibration information
	k4a_calibration_t sensorCalibration;
	VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
		   "Get depth camera calibration failed!");

	cout << "   Color resolution: " << sensorCalibration.color_camera_calibration.resolution_width << " x "
								    << sensorCalibration.color_camera_calibration.resolution_height << endl;
	cout << "   Depth resolution: " << sensorCalibration.depth_camera_calibration.resolution_width << " x "
								    << sensorCalibration.depth_camera_calibration.resolution_height << endl << endl;

	k4a_capture_t sensorCapture = NULL;
	// Synchronization
	CharucoSync sync((int)PatientTable::TestDevice);
	sync.SetParameters(camParm, detParm);
	sync.SetScalingFactor(scalingFactor);
	bool getData(false);
	waitKey(1000);
	while(true)
	{

		// 1. Capture
		k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0);

		if (getCaptureResult == K4A_WAIT_RESULT_FAILED)
		{
			std::cout << "Get img capture returned error: " << getCaptureResult << std::endl;
			break;
		}
		else if (getCaptureResult == K4A_WAIT_RESULT_TIMEOUT)
			continue;

		Vec3d rvec, tvec;
		k4a_image_t color_img = k4a_capture_get_color_image(sensorCapture);
		Mat color = color_to_opencv(color_img);

		// 2. Estimate Pose
		sync.EstimatePose(color, rvec, tvec);

		// 3. Render
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
		else if (key == 't')
			sync.TickSwitch();

		// 4. Release
		k4a_image_release(color_img);
		k4a_capture_release(sensorCapture);
	}
	k4a_device_close(device);

	cout << ">> Coordinate System is printed: " << outputName << endl;
	sync.WriteTransformationData(outputName);

	return EXIT_SUCCESS;
}

int RECORD_CHARUCO_SYNC(int argc, char** argv)
{
	char* recordName;
	string outputName = "world_coord";
	float scalingFactor = 0.3;
	for (int i=2; i<argc; i++) {
		if ( string(argv[i]) == "-r" ) {
			recordName = argv[i+1];
			i++;
		}
		else if ( string(argv[i]) == "-o" ) {
			outputName = argv[i+1];
			i++;
		}
		else if ( string(argv[i]) == "-s" ) {
			scalingFactor = atof(argv[i+1]);
		}
	}

	cout << "== PLAYBACK RECORD CHARUO BOARD DETECTION =======================" << endl;
	int timestamp = 1000;
	k4a_playback_t playback = NULL;
	k4a_calibration_t sensorCalibration;
	k4a_transformation_t transformation = NULL;
	k4a_capture_t capture = NULL;
	k4a_image_t color_image = NULL;
	k4a_image_t uncompressed_color_image = NULL;

	k4a_result_t result;
	k4a_stream_result_t stream_result;

	result = k4a_playback_open(recordName, &playback);
	if (result != K4A_RESULT_SUCCEEDED || playback == NULL) {
		printf("Failed to open recording %s\n", recordName); exit(1);
	}

	result = k4a_playback_seek_timestamp(playback, timestamp * 1000, K4A_PLAYBACK_SEEK_BEGIN);
	if (result != K4A_RESULT_SUCCEEDED)	{
		printf("Failed to seek timestamp %d\n", timestamp); exit(1);
	}
	cout << ">> Record Information" << endl;
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

	// Configuration
	string detParm("detector_params.yml");
	string camParm("kinect3072.yml");

	// Synchronization
	CharucoSync sync((int)PatientTable::AlluraXper);
	sync.SetParameters(camParm, detParm);
	sync.SetScalingFactor(scalingFactor);
	bool getData(false);
	waitKey(1000);

	while(true)
	{
		// 1. Capture
		stream_result = k4a_playback_get_next_capture(playback, &capture);
		if (stream_result == K4A_STREAM_RESULT_EOF) {
			cout << "   Last capture" << endl; break;
		}
		color_image = k4a_capture_get_color_image(capture);
		uncompressed_color_image = Convert_Color_MJPG_To_BGRA(color_image);

		Vec3d rvec, tvec;
		Mat color = color_to_opencv(uncompressed_color_image);

		// 2. Esimate pose
		sync.EstimatePose(color, rvec, tvec);

		// 3. Render
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
		else if (key == 't')
			sync.TickSwitch();

		// 4. Release
		k4a_capture_release(capture);
		k4a_image_release(color_image);
		k4a_image_release(uncompressed_color_image);
	}
	k4a_playback_close(playback);
	sync.WriteTransformationData(outputName);

	return EXIT_SUCCESS;
}

