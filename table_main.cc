// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "TableTracker.hh"
#include "CharucoSync.hh"
#include "Kinect.hh"
#include <Utilities.h>


int CHARUCO_SYNC(int argc, char** argv);

pair<Eigen::Vector4f, Eigen::Vector3f> ReadCharucoData(string fileName);

//void ContouringColorImg()
//{
//	Mat imgColor = imread("./record/color0001.png", IMREAD_COLOR);
//	imshow("out1", imgColor);
//	Mat imgGray = imread("./record/color0001.png", IMREAD_GRAYSCALE);
//	Mat bin;
//	cv::threshold(imgGray, bin, 200, 255, cv::THRESH_BINARY);
//	Mat edges = bin.clone();
//	imshow("out2", bin);
//	std::vector<std::vector<cv::Point>> contours, filteredContours;
//	cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
//	Mat cont = Mat::zeros(edges.rows, edges.cols, CV_8UC1);
//	cv::drawContours(cont, contours, -1, cv::Scalar(255));
//	imshow("out3", cont);
//	waitKey(0);
//}

Mat Read_K4A_MKV_Record(string fileName, Mat xy_table)
{
	int width = xy_table.cols;
	int height = xy_table.rows;

	Timer rec_timer;
	rec_timer.start();
	VideoCapture vcap(fileName);
	if ( !vcap.isOpened() )
		cerr << "Fail to read video record" << endl;

	Mat color_frame, depth_frame;
	vector<Mat> colorVec, depthVec;

	while(1)
	{
		vcap >> color_frame;
		if (color_frame.empty()) break;
		colorVec.push_back(color_frame);
	}

//	system(("ffmpeg -i " + fileName + " -map 0:1 -vsync 0 ./record/depth%d.png").c_str());

	cout << colorVec.size() << endl;

	for (size_t i=1; i<=colorVec.size(); i++) {
		string depthFile = "./record/depth" + to_string(i) + ".png";
		depth_frame = imread(depthFile, IMREAD_ANYDEPTH );
		depthVec.push_back(depth_frame);
//		imshow("depth", depth_frame);
//		char key = (char)waitKey(1000/30);
//		if (key == 'q') {
//			break;
//		}
	}

	Mat point_cloud = Mat::zeros(height, width, CV_32FC3);
	uint16_t* depth_data = (uint16_t*)depthVec[0].data;
	float* xy_table_data = (float*)xy_table.data;
	float* point_cloud_data = (float*)point_cloud.data;

	int point_count(0);
	for (int y=0, idx=0; y<height; y++) {
		for (int x=0; x<width; x++, idx++) {
			int channel = y * width * 3 + x * 3;
			if (depth_data[idx] != 0 && !isnan(xy_table_data[idx*2]) && !isnan(xy_table_data[idx*2+1]) )
			{
				float X = xy_table_data[idx*2]     * depth_data[idx];
				float Y = xy_table_data[idx*2 + 1] * depth_data[idx];
				float Z = depth_data[idx];
				float p[3] = {X,Y,Z};

				point_cloud_data[channel + 0] = xy_table_data[idx*2]     * depth_data[idx];// + calib_point[0];
				point_cloud_data[channel + 1] = xy_table_data[idx*2 + 1] * depth_data[idx];// + calib_point[1];
				point_cloud_data[channel + 2] = depth_data[idx];// + calib_point[2];
				point_count++;
			}
			else
			{
				point_cloud_data[channel + 0] = nanf("");
				point_cloud_data[channel + 1] = nanf("");
				point_cloud_data[channel + 2] = nanf("");
			}
		}
	}

	string file_name = "rec.ply";
    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << "ply" << std::endl;
    ofs << "format ascii 1.0" << std::endl;
    ofs << "element vertex "  << point_count << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "end_header" << std::endl;
    ofs.close();

    std::stringstream ss;
    for (int y=0, idx=0; y<height; y++) {
		for (int x=0; x<width; x++, idx++) {
			int channel = y * width * 3 + x * 3;
			if (isnan(point_cloud_data[channel + 0]) || isnan(point_cloud_data[channel + 1]) || isnan(point_cloud_data[channel + 2]) )
			{
				continue;
			}
			ss << (float)point_cloud_data[channel + 0] << " "
			   << (float)point_cloud_data[channel + 1] << " "
			   << (float)point_cloud_data[channel + 2] << std::endl;
		}
    }
    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());

	rec_timer.stop();
	cout << "Frame #: " << colorVec.size() << endl;
	cout << "Reading time: " << rec_timer.time() << endl;

//	system("rm ./record/*.png");

	exit(0);

	return point_cloud;
}

void PrintUsage()
{
    cout << "<Usage>" << endl;
    cout << "./table_tracker" << endl;
    cout << "./table_tracker -sync [outputfile]" << endl;
}

int main(int argc, char** argv)
{
//	ContouringColorImg();

	PrintUsage();
	if(argc > 1 && string(argv[1])=="-sync") return CHARUCO_SYNC(argc,argv);

	auto qtData = ReadCharucoData("worldXYZ.txt");

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

	int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
	int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

	Mat xy_table = create_xy_table(sensorCalibration);

//	Read_K4A_MKV_Record("rec0610.mkv", xy_table);

	k4a_capture_t sensorCapture = nullptr;
	k4a_image_t color_img = nullptr;
	k4a_image_t depth_img = nullptr;
	Mat color_mat, depth_mat, color_resize;

	TableTracker tableTracker(xy_table, qtData.first, qtData.second);

	int gCount(0);
	bool isCenter(false);
	while(1)
	{
		k4a_device_get_capture(device, &sensorCapture, 1000);
		color_img = k4a_capture_get_color_image(sensorCapture);
		depth_img = k4a_capture_get_depth_image(sensorCapture);

		color_mat = color_to_opencv(color_img);
		depth_mat = depth_to_opencv(depth_img);

		k4a_image_release(color_img);
		k4a_image_release(depth_img);
		k4a_capture_release(sensorCapture);

		if (!isCenter)
		{
			tableTracker.SetNewFrame(color_mat, depth_mat);
			isCenter = tableTracker.CalibrateTableCenter();


		}
		else
		{
			tableTracker.SetNewFrame(color_mat, depth_mat);
			tableTracker.SetTableOrigin(ocrDat(0), ocrDat(1), ocrDat(2));
			tableTracker.ProcessCurrentFrame();
			tableTracker.Render();


			char key = (char)waitKey(1);
			if (key == 'g') {
				int point_count = 0;
				cout << "Generate table point cloud" << endl;
				string name1 = "1_Point" + to_string(gCount) + ".ply";
				Mat point_cloud = tableTracker.GeneratePointCloud(depth_mat, xy_table, &point_count);
				tableTracker.WritePointCloud(name1, point_cloud, point_count);
				point_count = 0;
				string name2 = "2_Table" + to_string(gCount) + ".ply";
				Mat table_cloud = tableTracker.GenerateTablePointCloud(depth_mat, xy_table, &point_count);
				tableTracker.WritePointCloud(name2, table_cloud, point_count);
				gCount++;
			}
			else if (key == 's') {
				cout <<" Set table origin " << endl;
				float x,y,z;
				cin >> x >> y >> z;
				tableTracker.SetTableOrigin(x, y, z);
			}
			else if (key == 'q') {
				break;
			}
		}



	}
	return EXIT_SUCCESS;
}

int CHARUCO_SYNC(int argc, char** argv)
{
	//trackin option configuration3
	string detParm("detector_params.yml");
	string camParm("camera2160.yml");
	if (argc != 3) {
		cout << "<Usage>" << endl;
		cout << "./table_tracker -sync [outputfile]" << endl;
	}

	// Start camera
	k4a_device_t device = nullptr;
	VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

	// Start camera. Make sure depth camera is enabled.
	k4a_device_configuration_t deviceConfig = get_default_config();
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
	sync.WriteTransformationData(string(argv[1]));

	return EXIT_SUCCESS;
}

pair<Eigen::Vector4f, Eigen::Vector3f> ReadCharucoData(string fileName)
{
	// Read World Coordinate
	Eigen::Vector4f quat;
	Eigen::Vector3f tvec;
	ifstream ifs("worldXYZ.txt");
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
			quat << x, y, z, w;
		}
		else if (dump =="t") {
			ss >> x >> y >> z;
			tvec << x, y, z;
		}
	}
	return make_pair(quat, tvec);
}

