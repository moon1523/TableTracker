#include "TableTracker.hh"
#include <opencv2/core/eigen.hpp>

bool clicked;
Point2i P1, P2;
Rect cropRect;
float sfInv;

void onMouseCropImage(int event, int x, int y, int f, void *param);
Vector3d VectorRotatedByQuaternion(Vector3d v, Quaterniond q)
{
	Quaterniond p;
	p.w() = 0;
	p.vec() = v;
	Quaterniond rotatedP = q * p * q.inverse();
	Vector3d rotatedV = rotatedP.vec();
	return rotatedV;
}

TableTracker::TableTracker(int tableType, Mat xy_table, Quaterniond _quat, Vector3d _tvec)
: pTable(tableType), quat(_quat), tvec(_tvec),
  isColor(false), isCrop(false), isMask1(false), isMask2(false), isCenter(false), isFind(false), isMove(true), isRot(false),
  max_sim1(-FLT_MAX), max_sim2(-FLT_MAX)
{
	frameNo = 0;
	// color: 1280 x 720, depth: 640 x 576
	// All images use color resolutions.
	width  = xy_table.cols;
	height = xy_table.rows;

	switch((PatientTable)pTable)
	{
	case PatientTable::TestDevice:
		sf = 1.;
		init_position = Vector3d(0,0,0);
		long_width = 225;
		lat_width = 55;
		height_width = 27;
		floor_height = 300;
		top_margin = 50;
		bot_margin = 50;
		view_calib = Point2i(300,150);
		break;
	case PatientTable::AlluraXper:
		sf = 1/10.;
		init_position = Vector3d(-280, -960, -10); // lat, long, height
		long_width = 3200;
		lat_width = 500;
		height_width = 130;
		floor_height = 950;
		top_margin = 500;
		bot_margin = 150;
		view_calib = Point2i(1200,250); // +x: 1200 mm, +y: 250 mm
		break;
	}
	sfInv = 1.;

	tableCenter = Point2i(width*0.5, height*0.5);
	deg = 1;
	maxDeg = 90;
	transX = 0, transY = 0;

	axisX = VectorRotatedByQuaternion(Vector3d(1,0,0), quat);
	axisY = VectorRotatedByQuaternion(Vector3d(0,1,0), quat);
	axisZ = VectorRotatedByQuaternion(Vector3d(0,0,1), quat);

	cout << ">> World Coordinate (ChArUco Board)" << endl;
	cout << "  Origin: " << tvec(0) * 10 << " " << tvec(1) *10 << " " << tvec(2) * 10 << endl;
	cout << "  axis-X: " << axisX(0) << " " << axisX(1) << " " << axisX(2) << endl;
	cout << "  axis-Y: " << axisY(0) << " " << axisY(1) << " " << axisY(2) << endl;
	cout << "  axis-Z: " << axisZ(0) << " " << axisZ(1) << " " << axisZ(2) << endl << endl;

	float world_xdir[3];
	for (int i=0; i<3; i++) {
		world_origin[i] = tvec(i) * 10;
		world_normal[i] = axisZ(i);
		world_xdir[i] = axisX(i);
		ext_topPoint[i] = world_origin[i] + world_normal[i] * (floor_height + top_margin);
		ext_botPoint[i] = world_origin[i] + world_normal[i] * (floor_height - bot_margin);
	}
	vtkMath::Normalize(world_normal);
	vtkMath::Normalize(world_xdir);

	transform = vtkSmartPointer<vtkTransform>::New();
	transform = Transform_KinectView(world_normal, world_xdir);
	ocrData = Vector3d(0,0,0);
	maskSum = (long_width *sf) * (lat_width*sf) * 255;

	mask = cv::Mat::zeros(height, width, CV_8UC1);
	mask_data = (uchar*)mask.data;

	transform_mask = vtkSmartPointer<vtkTransform>::New();

	mask1 = Mat::zeros(height, width, CV_8UC1);
	mask2 = Mat::zeros(height, width, CV_8UC1);
	draw = Mat::zeros(height, width, CV_8UC3);

	cumAvgAngle = 0;
	frameAngle = 0;

	ofs.open("output.txt");
}

TableTracker::~TableTracker()
{
	ofs.close();
}

vtkSmartPointer<vtkTransform> TableTracker::Transform_KinectView(float z[3], float x[3])
{
	float axisZ[3];
	float v1[3];
	float v2[3] = {0,0,1};
	for (int i=0;i<3;i++) v1[i] = -z[i];
	vtkMath::Cross(v1, v2, axisZ);
	const float cosAZ = vtkMath::Dot(v1, v2);
	const float kZ = 1.0f / (1.0f + cosAZ);

	const double affTZ[16] = { (axisZ[0] * axisZ[0] * kZ) + cosAZ,    (axisZ[1] * axisZ[0] * kZ) - axisZ[2], (axisZ[2] * axisZ[0] * kZ) + axisZ[1], 0,
	           			       (axisZ[0] * axisZ[1] * kZ) + axisZ[2], (axisZ[1] * axisZ[1] * kZ) + cosAZ,    (axisZ[2] * axisZ[1] * kZ) - axisZ[0], 0,
					           (axisZ[0] * axisZ[2] * kZ) - axisZ[1], (axisZ[1] * axisZ[2] * kZ) + axisZ[0], (axisZ[2] * axisZ[2] * kZ) + cosAZ,    0,
							   0, 0, 0, 1 };
	vtkSmartPointer<vtkMatrix4x4> matZ = vtkSmartPointer<vtkMatrix4x4>::New();
	matZ->DeepCopy(affTZ);

	float axisX[3];
	float v3[3];
	float v4[3] = {0,1,0};
	for (int i=0;i<3;i++) v3[i] = x[i];
	vtkMath::Cross(v3, v4, axisX);
	const float cosAX = vtkMath::Dot(v3, v4);
	const float kX = 1.0f / (1.0f + cosAX);

	const double affTX[16] = { (axisX[0] * axisX[0] * kX) + cosAX,    (axisX[1] * axisX[0] * kX) - axisX[2], (axisX[2] * axisX[0] * kX) + axisX[1], 0,
	           			       (axisX[0] * axisX[1] * kX) + axisX[2], (axisX[1] * axisX[1] * kX) + cosAX,    (axisX[2] * axisX[1] * kX) - axisX[0], 0,
					           (axisX[0] * axisX[2] * kX) - axisX[1], (axisX[1] * axisX[2] * kX) + axisX[0], (axisX[2] * axisX[2] * kX) + cosAX,    0,
							   0, 0, 0, 1 };
	double affT[16];

	vtkSmartPointer<vtkMatrix4x4> matX = vtkSmartPointer<vtkMatrix4x4>::New();
	matX->DeepCopy(affTX);

	// Align the axes of Z and X for the world coordinate (charuco) system
	vtkSmartPointer<vtkMatrix4x4> mat44 = vtkSmartPointer<vtkMatrix4x4>::New();
	mat44->Multiply4x4(affTZ, affTZ, affT);

	transform->Identity();
	transform->SetMatrix(matZ);

	return transform;
}

Mat TableTracker::GenerateColorTablePointCloud(const k4a_image_t point_cloud_image, const k4a_image_t color_image)
{
	int16_t *point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(point_cloud_image);
	uint8_t *color_image_data = k4a_image_get_buffer(color_image);

	Mat point_cloud_2d = cv::Mat::zeros(height, width, CV_8UC1);
	uint8_t* point_cloud_2d_data = (uint8_t*)point_cloud_2d.data;

	float tf_point[3];
	float tf_worldOrigin[3]; // set to world origin
	float tf_worldNormal[3], tf_topPoint[3], tf_botPoint[3];
	transform->TransformPoint(world_origin, tf_worldOrigin);
	transform->TransformVector(world_normal, tf_worldNormal);
	transform->TransformPoint(ext_topPoint, tf_topPoint);
	transform->TransformPoint(ext_botPoint, tf_botPoint);


	for (int i = 0; i < width * height; i++)
	{
		float X = point_cloud_image_data[ 3 * i + 0 ];
		float Y = point_cloud_image_data[ 3 * i + 1 ];
		float Z = point_cloud_image_data[ 3 * i + 2 ];

		if (Z == 0) continue;

		int b = color_image_data[ 4 * i + 0 ];
		int g = color_image_data[ 4 * i + 1 ];
		int r = color_image_data[ 4 * i + 2 ];
		int alpha = color_image_data[ 4 * i + 3 ];
		if (r == 0 && g == 0 && b == 0 && alpha == 0) continue;

		float point[3] = {X,Y,Z};
		transform->TransformPoint(point, tf_point);

		float topPlane = tf_worldNormal[0]*(tf_point[0]-tf_topPoint[0]) + tf_worldNormal[1]*(tf_point[1]-tf_topPoint[1]) + tf_worldNormal[2]*(tf_point[2]-tf_topPoint[2]);
		float botPlane = tf_worldNormal[0]*(tf_point[0]-tf_botPoint[0]) + tf_worldNormal[1]*(tf_point[1]-tf_botPoint[1]) + tf_worldNormal[2]*(tf_point[2]-tf_botPoint[2]);

		// Extract Point Cloud Data
		if (topPlane > 0) continue;
		if (botPlane < 0) continue;

		int node_x = (int)( tf_point[0] - tf_worldOrigin[0] - view_calib.x ) * sf + width  * 0.5 ;
		int node_y = (int)( tf_point[1] - tf_worldOrigin[1] + view_calib.y ) * sf + height * 0.5 ;

		if (node_x > width) continue;
		if (node_y > height) continue;
		if (node_x < 0) continue;
		if (node_y < 0) continue;

		point_cloud_2d_data[ 3*(width*node_y + node_x) + 0] = b;
		point_cloud_2d_data[ 3*(width*node_y + node_x) + 1] = g;
		point_cloud_2d_data[ 3*(width*node_y + node_x) + 2] = r;

	}
	return point_cloud_2d;
}

Mat TableTracker::GenerateTablePointCloud(const k4a_image_t point_cloud_image, const k4a_image_t depth_image)
{
	int16_t *point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(point_cloud_image);
	uint8_t *depth_image_data = k4a_image_get_buffer(depth_image);

	Mat point_cloud_2d = cv::Mat::zeros(height, width, CV_8UC1);
	uint8_t* point_cloud_2d_data = (uint8_t*)point_cloud_2d.data;

	float tf_point[3];
	float tf_worldOrigin[3]; // set to world origin
	float tf_worldNormal[3], tf_topPoint[3], tf_botPoint[3];
	transform->TransformPoint(world_origin, tf_worldOrigin);
	transform->TransformVector(world_normal, tf_worldNormal);
	transform->TransformPoint(ext_topPoint, tf_topPoint);
	transform->TransformPoint(ext_botPoint, tf_botPoint);

	for (int i = 0; i < width * height; i++)
	{
		float X = point_cloud_image_data[ 3 * i + 0 ];
		float Y = point_cloud_image_data[ 3 * i + 1 ];
		float Z = point_cloud_image_data[ 3 * i + 2 ];

		if (Z == 0) continue;


		float point[3] = {X,Y,Z};
		transform->TransformPoint(point, tf_point);

		float topPlane = tf_worldNormal[0]*(tf_point[0]-tf_topPoint[0]) + tf_worldNormal[1]*(tf_point[1]-tf_topPoint[1]) + tf_worldNormal[2]*(tf_point[2]-tf_topPoint[2]);
		float botPlane = tf_worldNormal[0]*(tf_point[0]-tf_botPoint[0]) + tf_worldNormal[1]*(tf_point[1]-tf_botPoint[1]) + tf_worldNormal[2]*(tf_point[2]-tf_botPoint[2]);

		// Extract Point Cloud Data
		if (topPlane > 0) continue;
		if (botPlane < 0) continue;

		int node_x = (int)( tf_point[0] - tf_worldOrigin[0] - view_calib.x ) * sf + width  * 0.5 ;
		int node_y = (int)( tf_point[1] - tf_worldOrigin[1] + view_calib.y ) * sf + height * 0.5 ;

		if (node_x > width) continue;
		if (node_y > height) continue;
		if (node_x < 0) continue;
		if (node_y < 0) continue;

		point_cloud_2d_data[ width*node_y + node_x ] = 255;
	}
	return point_cloud_2d;
}

void TableTracker::ProcessCurrentFrame()
{
	if(isColor) {
		Mat colorPCD = GenerateColorTablePointCloud(point_img, color_img);
		add(colorPCD, draw, view);
		Mat grayPCD;
		cvtColor(colorPCD, grayPCD, COLOR_BGR2GRAY);
		threshold(grayPCD, binPCD, 10, 255, THRESH_BINARY);
	}
	else
	{
		binPCD = GenerateTablePointCloud(point_img, depth_img);
	}

	results = MatchingData(deg);

	// If table is not moved within 30 frames (i.e., OCR data is not changed), start to check the table rotate.
	if (!isMove && angleVec.size() > 30) {

		double max = *max_element(angleVec.begin(), angleVec.end());
		double min = *min_element(angleVec.begin(), angleVec.end());
		double nume = accumulate(angleVec.begin(), angleVec.end(), 0);
		double deno = angleVec.size();
		double prevAngle = angleVec[angleVec.size()-2];
		double currAngle = get<0>(results);

		if (max-min > 3)
		{
			isRot = true;
			angleVec.clear();
		}
		else
			isRot = false;
	}
}

void TableTracker::Render(double time)
{
	Mat match_mat = get<2>(results);
	double similarity = get<1>(results);

	if (!isMove && !isRot)
	{
		double nume = accumulate(angleVec.begin(), angleVec.end(), 0);
		double deno = angleVec.size();
		cumAvgAngle = floor(nume/deno + 0.5);

//		cout << frameNo << " " << nume << " " << deno << " " << cumAvgAngle << " " << get<0>(results) <<  endl;

		// If cumAvgAngle is between -3 and 3 degrees, set the angle 0 degree.
		if ( -3.0 <= cumAvgAngle && cumAvgAngle <= 3.0 )
			cumAvgAngle = 0.0;
		string str_deg = to_string(get<0>(results)) + "/" + to_string(cumAvgAngle);
		cv::bitwise_xor(maskVec[cumAvgAngle + 90], binPCD, match_mat);
		putText(match_mat, "Table Degree (Raw/Cum): " + str_deg,
				cv::Point(10,height-60), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);

		ofs << frameNo << "\t" << cumAvgAngle << endl;
	}
	else
	{
		putText(match_mat, "Table Degree (Raw): " + to_string(get<0>(results)),
				cv::Point(10,height-60), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);

		ofs << frameNo << "\t" << get<0>(results) << endl;
	}

	if (similarity < 0.5) {
		putText(match_mat, "Similarity is lower than 0.5, Severe occlusion is occurred.",
				cv::Point(10,60), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
	}
	putText(match_mat, "Frame #: " + to_string(frameNo++),
				cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
	putText(match_mat, "Similarity: " + to_string(similarity),
				cv::Point(10,height-90), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
	putText(match_mat, "Frame Time (s): " + to_string(time),
				cv::Point(10,height-30), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);

	circle(match_mat, tableCenter, 2.0, Scalar::all(150), 3, 8, 0);
	imshow("Match Results", match_mat);




	// Color view
	rectangle(view, mask1_p1, mask1_p2, CV_RGB(255,255,0), 2);
	rectangle(view, mask2_p1, mask2_p2, CV_RGB(255,255,0), 2);
	circle(view, tableCenter, 2.0, CV_RGB(255,0,0), 3, 8, 0);
	if(isColor) imshow("TableCenter", view);

	// Check the mask and table center
	//	Mat check = cv::Mat::zeros(height, width, CV_8UC3);
	//	rectangle(check, mask1_p1, mask1_p2, CV_RGB(255,255,0), 2);
	//	rectangle(check, mask2_p1, mask2_p2, CV_RGB(255,255,0), 2);
	//	circle(check, tableCenter, 2.0, CV_RGB(255,0,0), 3, 8, 0);
	//	imshow("check",check);

	// Re-initialize surface points, table center, mask
	for (int i=0;i<3;i++) {
		ext_topPoint[i] = world_origin[i] + world_normal[i] * (floor_height + top_margin);
		ext_botPoint[i] = world_origin[i] + world_normal[i] * (floor_height - bot_margin);
	}
	tableCenter = tableCenter_init;
	mask1_p1 = mask1_p1_init;
	mask1_p2 = mask1_p2_init;
}

tuple<double, double, Mat> TableTracker::MatchingData(int deg)
{
	Mat match_xor, match_and, match_mat;
	double max_sim(0);
	double match_deg(0);
	int idx(0);
	for (int i=-maxDeg/deg; i<=maxDeg/deg; i++)
	{
		// Matching masks and point cloud data
		cv::bitwise_and(maskVec[idx], binPCD, match_and);
		double similarity = cv::sum(match_and).val[0] / maskSum;

		// Find the most similar mask
		if (max_sim < similarity) {
			// bitwise_xor is used to view, not affect the actual matching calculation
			cv::bitwise_xor(maskVec[idx], binPCD, match_xor); match_mat = match_xor;
			max_sim = similarity;
			match_deg = (idx - maxDeg) * deg; // if 0 deg is matched ==> (90 - 90) * 1 deg
		}
		idx++;
	}

	// Accumulate Data (if table is moved or is rotated, data will be cleared.
	// Check the function TransMathcingMasks in header.
	angleVec.push_back(match_deg);
	frameAngle = match_deg;

	// if matching data is empty, only point cloud data will be shown.
	if (match_mat.empty())
		match_mat = binPCD;

	return make_tuple(match_deg, max_sim, match_mat);
}

void TableTracker::GenerateTableMask(Mat xy_table, int deg)
{
	// Initialize the mask data.
	mask = cv::Mat::zeros(height, width, CV_8UC1);
	for (int y = mask1_p1.y; y < mask1_p2.y; y++) {
		for (int x = mask1_p1.x; x < mask1_p2.x; x++) {
			mask_data[ y * width + x ] = 255;
		}
	}

//	Timer mask_timer;
//	mask_timer.start();
	// Rotate the mask
	maskVec.clear();
	for (int i=-maxDeg/deg; i<=maxDeg/deg; i++)
	{
		cv::Mat mask_rot, rotM;
		mask.copyTo(mask_rot);
		rotM = getRotationMatrix2D(Point(tableCenter.x, tableCenter.y), -deg*i, 1.0);
		warpAffine(mask, mask_rot, rotM, mask.size());
		maskVec.push_back(mask_rot);
	}
//	mask_timer.stop();
//	cout << "\rMask Generation Time: " << mask_timer.time() << flush;



	// View
//	cout << " Press 'c' to skip the present frame " << endl;
//	cout << " Press 'q' to quit " << endl;
//	int idx(0);
//
//	if (isCenter)
//	{
//		while(1)
//		{
//			if (idx > 180) idx = 0;
//			cv::circle(maskVec[idx], Point(tableCenter.x, tableCenter.y), 1.0, Scalar::all(0), 3, 8, 0);
//			cv::putText(maskVec[idx], "Resolution: "+ to_string(width)+ "x" + to_string(height),
//									 Point(10,20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255), 1);
//			cv::putText(maskVec[idx], "Table Degree: "+ to_string((idx-90)*deg),
//									 Point(10,40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255), 1);
//
//			imshow("mask_rot", maskVec[idx++]);
//
//
//			char key = (char)cv::waitKey(1000);
//
//			if (key == 'c') continue;
//			if (key == 'q') {
//				cv::destroyAllWindows();
//				break;
//			}
//		}
//
//	}


//	return maskVec;
}

bool TableTracker::FindTableCenter()
{
	Mat colorPCD, binPCD;
	if (isColor)
	{
		colorPCD = GenerateColorTablePointCloud(point_img, color_img);
		Mat grayPCD;
		cvtColor(colorPCD, grayPCD, COLOR_BGR2GRAY);
		threshold(grayPCD, binPCD, 10, 255, THRESH_BINARY);
	}
	else
	{
		binPCD = GenerateTablePointCloud(point_img, depth_img);
		binPCD.copyTo(colorPCD);
	}

	putText(colorPCD, "Press 'c' key to capture the mask",
			cv::Point(10,30), FONT_HERSHEY_SIMPLEX, 0.75, Scalar::all(255), 2);


	if (!isFind)
	{
		if (isCenter) {
			add(colorPCD, draw, view);
			imshow("Center", view);
		}
		else imshow("Center", colorPCD);
		char key = (char)waitKey(1);
		if (key == 'c') {
			destroyWindow("Center");
			isFind = true;
		}
	}
	else
	{
		int x_node = !isMask1 ? lat_width  * sf : long_width * sf;
		int y_node = !isMask1 ? long_width * sf : lat_width  * sf;

		// Set starting point of mask
		while (1)
		{
			Mat roi, copy;
			colorPCD.copyTo(roi);
			colorPCD.copyTo(copy);
			Rect bounds(0, 0, roi.cols, roi.rows);
			setMouseCallback("Drag", onMouseCropImage, &roi);

			if (clicked) isCrop = true;
			if (cropRect.width > 0 && clicked == false) {
				roi = colorPCD(cropRect & bounds);
				if (isCrop)
				{
					int cropBox_width = P2.x - P1.x;
					int cropBox_height = P2.y - P1.y;
					cout << "  CropBox [Width x Height]: " << cropBox_width << " x " << cropBox_height << endl;

					if (cropBox_width > x_node && cropBox_height > y_node ) {
						cout << "   Set ROI box" << flush;
						transX = P1.x, transY = P1.y;
						destroyWindow("Drag");
						isCrop = false;
						break;
					}
					else {
						cout << "   Reset ROI box" << flush;
						transX, transY = 0;
						isCrop = false;
					}

				}
			}
			else colorPCD.copyTo(roi);
			cv::rectangle(copy, P1, P2, CV_RGB(255,255,255), 2);
			cv::imshow("Drag", copy);
			waitKey(1);
		}

		isFind = false;

		// Start dragging mask
		Mat match_xor, match_and;
		while(1)
		{
			Timer proc_timer;
			proc_timer.start();
			Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
			uchar* mask_data = (uchar*)mask.data;
			Point2i p1( transX, transY );
			Point2i p2( x_node + transX, y_node + transY );

			for (int y=0; y<y_node; y++) {
				for (int x=0; x<x_node; x++) {
					mask_data[(y + transY) * width + (x + transX)] = 255;
				}
			}

			cv::bitwise_and(mask, binPCD, match_and);
			cv::bitwise_xor(mask, binPCD, match_xor);
			double similarity = cv::sum(match_and).val[0] / maskSum;

			if (!isMask1) {
				if (max_sim1 < similarity) {
					max_sim1 = similarity;
					match_and.copyTo(mask1);
					mask1_p1 = p1, mask1_p2 = p2;
				}
			}
			else {
				if (max_sim2 < similarity) {
					max_sim2 = similarity;
					match_and.copyTo(mask2);
					mask2_p1 = p1, mask2_p2 = p2;
				}
			}

			if (transX == (P2.x - x_node)) {
				transX = P1.x;
				transY++;
				if (transY == (P2.y - y_node)) {
					if (isMask1) {
						cout << "...Mask2 (90 deg) set" << endl;
						isMask2 = true;
						isCenter = true;
						break;
					}
					cout << "...Mask1 (0 deg) set" << endl;
					transX = 0, transY = 0;
					isMask1 = true;
					break;
				}
			}
			transX++;

			proc_timer.stop();
			putText(match_xor, "TransX: "  + to_string(P1.x) + " | " + to_string(transX) + " | " + to_string(P2.x - x_node),
					cv::Point(10,height-90), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
			putText(match_xor, "TransY: "  + to_string(P1.y) + " | " + to_string(transY) + " | " + to_string(P2.y - y_node),
					cv::Point(10,height-60), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
			putText(match_xor, "Similarity: " + to_string(similarity),
					cv::Point(10,height-30), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
			putText(match_xor, "Processing Time (s): " + to_string(proc_timer.time()),
					cv::Point(10,height-120), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
			imshow("CenterMask", match_xor);
			waitKey(1);
		}

		// Find rotation center of table masks
		if( isMask1 && isMask2 )
		{
			Point2i mask1A(mask1_p1.x, mask1_p1.y);
			Point2i mask1B(mask1_p1.x, mask1_p2.y);
			Point2i mask1C(mask1_p2.x, mask1_p2.y);
			Point2i mask1D(mask1_p2.x, mask1_p1.y);

			Point2i mask2A(mask2_p2.x, mask2_p1.y);
			Point2i mask2B(mask2_p1.x, mask2_p1.y);
			Point2i mask2C(mask2_p1.x, mask2_p2.y);
			Point2i mask2D(mask2_p2.x, mask2_p2.y);

			Point2i midA_((mask1A.x + mask2A.x) * 0.5, (mask1A.y + mask2A.y) * 0.5);
			Point2i midB_((mask1B.x + mask2B.x) * 0.5, (mask1B.y + mask2B.y) * 0.5);
			Point2i midC_((mask1C.x + mask2C.x) * 0.5, (mask1C.y + mask2C.y) * 0.5);
			Point2i midD_((mask1D.x + mask2D.x) * 0.5, (mask1D.y + mask2D.y) * 0.5);

			int x1 = mask2A.x-mask1A.x; int y1 = mask2A.y-mask1A.y;
			int x2 = mask2B.x-mask1B.x;	int y2 = mask2B.y-mask1B.y;
			int x3 = mask2C.x-mask1C.x;	int y3 = mask2C.y-mask1C.y;
			int x4 = mask2D.x-mask1D.x;	int y4 = mask2D.y-mask1D.y;

			Vector2f midA((mask1A.x + mask2A.x) * 0.5, (mask1A.y + mask2A.y) * 0.5);
			Vector2f midB((mask1B.x + mask2B.x) * 0.5, (mask1B.y + mask2B.y) * 0.5);
			Vector2f midC((mask1C.x + mask2C.x) * 0.5, (mask1C.y + mask2C.y) * 0.5);
			Vector2f midD((mask1D.x + mask2D.x) * 0.5, (mask1D.y + mask2D.y) * 0.5);

			Vector2f pA(midA.x() - y1, midA.y() + x1);
			Vector2f pB(midB.x() - y2, midB.y() + x2);
			Vector2f pC(midC.x() - y3, midC.y() + x3);
			Vector2f pD(midD.x() - y4, midD.y() + x4);

			Point2i pA_(midA.x() - y1, midA.y() + x1);
			Point2i pB_(midB.x() - y2, midB.y() + x2);
			Point2i pC_(midC.x() - y3, midC.y() + x3);
			Point2i pD_(midD.x() - y4, midD.y() + x4);

			using Line2 = Eigen::Hyperplane<float,2>;
			Line2 lineA = Line2::Through(midA, pA);
			Line2 lineB = Line2::Through(midB, pB);
			Line2 lineC = Line2::Through(midC, pC);
			Line2 lineD = Line2::Through(midD, pD);

			Vector2f centerF = lineA.intersection(lineB);
			tableCenter = Point2i((int)centerF.x(), (int)centerF.y());
			rectangle(draw, mask1_p1, mask1_p2, CV_RGB(255,255,255), 1);
			rectangle(draw, mask2_p1, mask2_p2, CV_RGB(255,255,255), 1);
			line(draw, mask1A, mask2A, CV_RGB(255,255,0), 1, 8, 0);
			line(draw, mask1B, mask2B, CV_RGB(255,255,0), 1, 8, 0);
			line(draw, mask1C, mask2C, CV_RGB(255,255,0), 1, 8, 0);
			line(draw, mask1D, mask2D, CV_RGB(255,255,0), 1, 8, 0);
			circle(draw, midA_, 1.0, CV_RGB(255,0,0), 3, 8, 0);
			circle(draw, midB_, 1.0, CV_RGB(255,0,0), 3, 8, 0);
			circle(draw, midC_, 1.0, CV_RGB(255,0,0), 3, 8, 0);
			circle(draw, midD_, 1.0, CV_RGB(255,0,0), 3, 8, 0);
			circle(draw, pA_, 1.0, CV_RGB(255,0,0), 3, 8, 0);
			circle(draw, pB_, 1.0, CV_RGB(255,0,0), 3, 8, 0);
			circle(draw, pC_, 1.0, CV_RGB(255,0,0), 3, 8, 0);
			circle(draw, pD_, 1.0, CV_RGB(255,0,0), 3, 8, 0);
			line(draw, midA_, pA_, CV_RGB(0,255,0), 1, 8, 0);
			line(draw, midB_, pB_, CV_RGB(0,255,0), 1, 8, 0);
			line(draw, midC_, pC_, CV_RGB(0,255,0), 1, 8, 0);
			line(draw, midD_, pD_, CV_RGB(0,255,0), 1, 8, 0);
			circle(draw, tableCenter, 2.0, CV_RGB(0,0,255), 3, 8, 0);

			destroyAllWindows();
			isCenter = true;
			cout << "TableCenter.txt is generated" << endl;
			ofstream ofs("TableCenter.txt");
			ofs << "P1: " << mask1_p1.x << " " << mask1_p1.y << endl;
			ofs << "P2: " << mask1_p2.x << " " << mask1_p2.y << endl;
			ofs << "Center: " << tableCenter.x << " " << tableCenter.y << endl;
			ofs.close();

			tableCenter_init = tableCenter;
			mask1_p1_init = mask1_p1;
			mask1_p2_init = mask1_p2;

			for (int y = mask1_p1.y; y < mask1_p2.y; y++) {
				for (int x = mask1_p1.x; x < mask1_p2.x; x++) {
					mask_data[ y * width + x ] = 255;
				}
			}
			GenerateTableMask(xy_table, deg);
		}
	}
	return isCenter;
}



void onMouseCropImage(int event, int x, int y, int f, void *param)
{
    switch (event)
    {
    case EVENT_LBUTTONDOWN:
        clicked = true;
        P1.x = x * sfInv;
        P1.y = y * sfInv;
        P2.x = x * sfInv;
        P2.y = y * sfInv;
        break;
    case EVENT_LBUTTONUP:
        P2.x = x * sfInv;
        P2.y = y * sfInv;
        clicked = false;
        break;
    case EVENT_MOUSEMOVE:
        if (clicked)
        {
            P2.x = x * sfInv;
            P2.y = y * sfInv;
        }
        break;
    case EVENT_RBUTTONUP:
        clicked = false;
        P1.x = 0;
        P1.y = 0;
        P2.x = 0;
        P2.y = 0;
        break;
    default:
        break;
    }

    if (clicked)
    {
        if (P1.x > P2.x)
        {
            cropRect.x = P2.x;
            cropRect.width = P1.x - P2.x;
        }
        else
        {
            cropRect.x = P1.x;
            cropRect.width = P2.x - P1.x;
        }

        if (P1.y > P2.y)
        {
            cropRect.y = P2.y;
            cropRect.height = P1.y = P2.y;
        }
        else
        {
            cropRect.y = P1.y;
            cropRect.height = P2.y - P1.y;
        }
    }
}
