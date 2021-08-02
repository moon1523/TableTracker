#include "TableTracker.hh"
#include <opencv2/core/eigen.hpp>

bool clicked;
Point2i P1, P2;
Rect cropRect;
float sfInv;

void onMouseCropImage(int event, int x, int y, int f, void *param);
Vector3f VectorRotatedByQuaternion(Vector3f v, Quaternionf q)
{
	Quaternionf p;
	p.w() = 0;
	p.vec() = v;
	Quaternionf rotatedP = q * p * q.inverse();
	Vector3f rotatedV = rotatedP.vec();
	return rotatedV;
}

TableTracker::TableTracker(int tableType, Mat xy_table, Quaternionf _quat, Vector3f _tvec)
: pTable(tableType), quat(_quat), tvec(_tvec),
  isCrop(false), isRot(false), isMask1(false), isMask2(false), isCenter(false), isFind(false),
  max_sim1(-FLT_MAX), max_sim2(-FLT_MAX)
{

	// color: 1280 x 720, depth: 640 x 576
	width  = xy_table.cols;
	height = xy_table.rows;

	switch((PatientTable)pTable)
	{
	case PatientTable::TestDevice:
		sf = 1.;
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
		long_width = 3200;
		lat_width = 600;
		height_width = 130;
		floor_height = 950;
		top_margin = 100;
		bot_margin = 200;
		view_calib = Point2i(1200,250);
		break;
	}
	sfInv = 1.;

	tableCenter = Point2i(width*0.5, height*0.5);
	deg = 1;
	transX = 0, transY = 0;

	Vector3f axisX = VectorRotatedByQuaternion(Vector3f(1,0,0), quat);
	Vector3f axisY = VectorRotatedByQuaternion(Vector3f(0,1,0), quat);
	Vector3f axisZ = VectorRotatedByQuaternion(Vector3f(0,0,1), quat);

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

	maskVec = GenerateTableMask(xy_table, deg);
	mask1 = Mat::zeros(height, width, CV_8UC1);
	mask2 = Mat::zeros(height, width, CV_8UC1);
	draw = Mat::zeros(height, width, CV_8UC3);

//	ofs1.open("pcd.obj");
}

TableTracker::~TableTracker()
{
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

	Mat point_cloud_2d = cv::Mat::zeros(height, width, CV_8UC3);
	uint8_t* point_cloud_2d_data = (uint8_t*)point_cloud_2d.data;

	float tf_point[3];
	float tf_worldOrigin[3]; // set to world origin
	float tf_worldNormal[3], tf_topPoint[3], tf_botPoint[3];
	transform->TransformPoint(world_origin, tf_worldOrigin);
	transform->TransformVector(world_normal, tf_worldNormal);
	transform->TransformPoint(ext_topPoint, tf_topPoint);
	transform->TransformPoint(ext_botPoint, tf_botPoint);

//	cout << point_cloud_image_data [ 3 * (int)(width * height * 0.25) + 0 ] << endl;
//	cout << point_cloud_image_data [ 3 * (int)(width * height * 0.25) + 1 ] << endl;

//	cout << "#" << endl;
//	cout << world_origin[0] << " " << world_origin[1] << " " << world_origin[2] << endl;
//	cout << world_normal[0] << " " << world_normal[1] << " " << world_normal[2] << endl;
//	cout << ext_topPoint[0] << " " << ext_topPoint[1] << " " << ext_topPoint[2] << endl;
//	cout << ext_botPoint[0] << " " << ext_botPoint[1] << " " << ext_botPoint[2] << endl;
//
//	cout << tf_worldOrigin[0] << " " << tf_worldOrigin[1] << " " << tf_worldOrigin[2] << endl;
//	cout << tf_worldNormal[0] << " " << tf_worldNormal[1] << " " << tf_worldNormal[2] << endl;
//	cout << tf_topPoint[0] << " " << tf_topPoint[1] << " " << tf_topPoint[2] << endl;
//	cout << tf_botPoint[0] << " " << tf_botPoint[1] << " " << tf_botPoint[2] << endl;

	for (int i = 0; i < width * height; i++)
	{
		float X = point_cloud_image_data[ 3 * i + 0 ];
		float Y = point_cloud_image_data[ 3 * i + 1 ];
		float Z = point_cloud_image_data[ 3 * i + 2 ];

//		ofs1 << i<< " v " << X << " " << Y << " " << Z << endl;

		if (Z == 0) continue;

		int b = color_image_data[ 4 * i + 0 ];
		int g = color_image_data[ 4 * i + 1 ];
		int r = color_image_data[ 4 * i + 2 ];
		int alpha = color_image_data[ 4 * i + 3 ];
		if (r == 0 && g == 0 && b == 0 && alpha == 0) continue;

		float point[3] = {X,Y,Z};
		transform->TransformPoint(point, tf_point);
//		ofs1 << "v " << tf_point[0] << " " << tf_point[1] << " " << tf_point[2] << endl;

		float topPlane = tf_worldNormal[0]*(tf_point[0]-tf_topPoint[0]) + tf_worldNormal[1]*(tf_point[1]-tf_topPoint[1]) + tf_worldNormal[2]*(tf_point[2]-tf_topPoint[2]);
		float botPlane = tf_worldNormal[0]*(tf_point[0]-tf_botPoint[0]) + tf_worldNormal[1]*(tf_point[1]-tf_botPoint[1]) + tf_worldNormal[2]*(tf_point[2]-tf_botPoint[2]);

		// Extract Point Cloud Data
		if (topPlane > 0) continue;
		if (botPlane < 0) continue;

		int node_x = (int)( tf_point[0] - tf_worldOrigin[0] - view_calib.x ) * sf + width  * 0.5 ;
		int node_y = (int)( tf_point[1] - tf_worldOrigin[1] + view_calib.y ) * sf + height * 0.5 ;
//		int node_x = (int)(tf_point[0] - tf_worldOrigin[0] - view_calib.x) * sf + (width  * 0.5);
//		int node_y = (int)(tf_point[1] - tf_worldOrigin[1] + view_calib.y) * sf + (height * 0.5);
//		int node_x = (int)(tf_point[0] - tf_worldOrigin[0]) * sf + (width  * 0.5);
//		int node_y = (int)(tf_point[1] - tf_worldOrigin[1]) * sf + (height * 0.5);

//		ofs1 << "v " << node_x << " " << node_y << " 0" << endl;

		if (node_x > width) continue;
		if (node_y > height) continue;
		if (node_x < 0) continue;
		if (node_y < 0) continue;

		point_cloud_2d_data[ 3*(width*node_y + node_x) + 0] = b;
		point_cloud_2d_data[ 3*(width*node_y + node_x) + 1] = g;
		point_cloud_2d_data[ 3*(width*node_y + node_x) + 2] = r;
	}
//	ofs1.close();
	return point_cloud_2d;
}

bool TableTracker::FindTableCenter()
{
	Mat colorPCD = GenerateColorTablePointCloud(point_img, color_img);
	Mat grayPCD, binPCD;
	cvtColor(colorPCD, grayPCD, COLOR_BGR2GRAY);
	threshold(grayPCD, binPCD, 10, 255, THRESH_BINARY);

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
			cv::rectangle(copy, P1, P2, CV_RGB(255,255,0), 2);
			cv::imshow("Drag", copy);
			waitKey(1);
		}

		isFind = false;

		// Start dragging mask
		Mat match_xor, match_and;
		int mask_sum = cv::sum(maskVec[0]).val[0];

		while(1)
		{
			Timer proc_timer;
			proc_timer.start();
			cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
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
			double similarity = cv::sum(match_and).val[0] / mask_sum;

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
			circle(draw, tableCenter, 1.0, CV_RGB(0,0,255), 3, 8, 0);

			destroyAllWindows();
			isCenter = true;
//			cout << "  Table Center was set: " << tableCenter << endl;

			maskVec.clear();
			maskVec = GenerateTableMask(xy_table, deg);
		}
	}
	return isCenter;
}

void TableTracker::ProcessCurrentFrame()
{
	frameTimer.start();
	Mat colorPCD = GenerateColorTablePointCloud(point_img, color_img);
	results = MatcingData(maskVec, colorPCD, deg);
	add(colorPCD, draw, view);
	frameTimer.stop();
}

void TableTracker::Render()
{

    Mat match_mat = get<2>(results);
	putText(match_mat, "Table Degree: " + to_string(get<0>(results)),
							cv::Point(10,height-60), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
//	putText(match_mat, "Similarity: " + to_string(get<1>(results)) + "%",
//										cv::Point(10,height-60), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
	putText(match_mat, "Frame Time (s): " + to_string(frameTimer.time()),
										cv::Point(10,height-30), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
	circle(match_mat, tableCenter, 1.0, Scalar::all(150), 3, 8, 0);


	imshow("TableCenter", view);
	imshow("Match Results", match_mat);
}

tuple<double, double, Mat> TableTracker::MatcingData(vector<Mat> maskVec, Mat point_cloud_2d, int deg)
{
	Mat grayPCD, binPCD;
	cvtColor(point_cloud_2d, grayPCD, COLOR_BGR2GRAY);
	threshold(grayPCD, binPCD, 10, 255, THRESH_BINARY);

	Mat match_xor, match_and, match_mat;
	int mask_sum = cv::sum(maskVec[0]).val[0];
	double max_sim(0);
	double match_deg(0);
	int idx(0);
	for (int i=-90/deg; i<=90/deg; i++)
	{
		cv::bitwise_and(maskVec[idx], binPCD, match_and);
		double similarity = cv::sum(match_and).val[0] / mask_sum;
		if (max_sim < similarity) {
			cv::bitwise_xor(maskVec[idx], binPCD, match_xor);
			max_sim = similarity;
			match_deg = (idx - 90) * deg;
			match_mat = match_xor;
		}
		idx++;
	}

	if (match_mat.empty())
		match_mat = point_cloud_2d;

	return make_tuple(match_deg, max_sim, match_mat);
}

vector<Mat> TableTracker::GenerateTableMask(Mat xy_table, int deg)
{
	int x_node = lat_width * 0.5 * sf;
	int y_nodeBot = long_width * 0.5 * sf;
	int y_nodeTop = long_width * 0.5 * sf;

	cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
	uchar* mask_data = (uchar*)mask.data;

	int yMin = isCenter? mask1_p1.y : height * 0.5 - y_nodeTop;
	int yMax = isCenter? mask1_p2.y : height * 0.5 + y_nodeBot;
	int xMin = isCenter? mask1_p1.x : width  * 0.5 - x_node;
	int xMax = isCenter? mask1_p2.x : width  * 0.5 + x_node;

	if (isCenter) {
		cout << "==========================================" << endl;
		cout << "  Table Center: " << tableCenter << endl;
		cout << "  Mask pixel X: [" << xMin << ", " << xMax << "]  " << xMax-xMin << endl;
		cout << "  Mask pixel Y: [" << yMin << ", " << yMax << "]  " << yMax-yMin << endl;
		cout << "==========================================" << endl;
	}

	vector<cv::Mat> maskVec;

	for (int y = yMin; y < yMax; y++) {
		for (int x = xMin; x < xMax; x++) {
			mask_data[ y * width + x ] = 255;
		}
	}

	// Rotate the mask
	for (int i=-90/deg; i<=90/deg; i++)
	{
		cv::Mat mask_rot, rotM;
		mask.copyTo(mask_rot);
		if(!isCenter) rotM = getRotationMatrix2D(Point(width*0.5, height*0.5), -deg*i, 1.0);
		else          rotM = getRotationMatrix2D(Point(tableCenter.x, tableCenter.y), -deg*i, 1.0);
		warpAffine(mask, mask_rot, rotM, mask.size());
		maskVec.push_back(mask_rot);
	}

	cout << " Press 'c' to skip the present frame " << endl;
	cout << " Press 'q' to quit " << endl;
	int idx(0);

//	if(isCenter)
//	{
		while(1)
		{
			if (idx > 180) idx = 0;
			cv::circle(maskVec[idx], Point(tableCenter.x, tableCenter.y), 1.0, Scalar::all(0), 3, 8, 0);
			cv::putText(maskVec[idx], "Resolution: "+ to_string(width)+ "x" + to_string(height),
									 Point(10,20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255), 1);
			cv::putText(maskVec[idx], "Table Degree: "+ to_string((idx-90)*deg),
									 Point(10,40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255), 1);

			imshow("mask_rot", maskVec[idx++]);


			char key = (char)cv::waitKey(1000);

			if (key == 'c') continue;
			if (key == 'q') {
				cv::destroyAllWindows();
				break;
			}
		}
//	}


	return maskVec;
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

//vector<Mat> TableTracker::Read_K4A_MKV_Record(string fileName, Mat xy_table)
//{
//
//
//
////	Timer rec_timer;
////	rec_timer.start();
////	VideoCapture vcap(fileName);
////	if ( !vcap.isOpened() )
////		cerr << "Fail to read video record" << endl;
////
////	Mat color_frame, depth_frame;
////	vector<Mat> colorVec, depthVec;
////
////	while(1)
////	{
////		vcap >> color_frame;
////		if (color_frame.empty()) break;
////		colorVec.push_back(color_frame);
////	}
////
////	// https://docs.microsoft.com/en-us/azure/kinect-dk/record-file-format
////	// -map 0:1 (depth)
////	// -map 0:0 (color)
////	// -vsync 0 (match frame rate)
////	system(("ffmpeg -i " + fileName + " -map 0:1 -vsync 0 ./record/depth%d.png").c_str());
////
//	vector<Mat> point_cloud_vec;
////	for (size_t i=0; i<colorVec.size(); i++) {
////		string depthFile = "./record/depth" + to_string(i+1) + ".png";
////		depth_frame = imread(depthFile, IMREAD_ANYDEPTH );
////		depthVec.push_back(depth_frame);
//////		imshow("depth", depth_frame);
//////		char key = (char)waitKey(1000/30);
//////		if (key == 'q') {
//////			break;
//////		}
////
////		Mat point_cloud = Mat::zeros(height, width, CV_32FC3);
////		float* xy_table_data = (float*)xy_table.data;
////		float* point_cloud_data = (float*)point_cloud.data;
////		uint16_t* depth_data = (uint16_t*)depthVec[i].data;
////
////		int point_count(0);
////		for (int y=0, idx=0; y<height; y++) {
////			for (int x=0; x<width; x++, idx++) {
////				int channel = y * width * 3 + x * 3;
////				if (depth_data[idx] != 0 && !isnan(xy_table_data[idx*2]) && !isnan(xy_table_data[idx*2+1]) )
////				{
////					float X = xy_table_data[idx*2]     * depth_data[idx];
////					float Y = xy_table_data[idx*2 + 1] * depth_data[idx];
////					float Z = depth_data[idx];
////					float p[3] = {X,Y,Z};
////
////						point_cloud_data[channel + 0] = xy_table_data[idx*2]     * depth_data[idx];// + calib_point[0];
////						point_cloud_data[channel + 1] = xy_table_data[idx*2 + 1] * depth_data[idx];// + calib_point[1];
////						point_cloud_data[channel + 2] = depth_data[idx];// + calib_point[2];
////						point_count++;
////				}
////				else
////				{
////					point_cloud_data[channel + 0] = nanf("");
////					point_cloud_data[channel + 1] = nanf("");
////					point_cloud_data[channel + 2] = nanf("");
////				}
////			}
////		}
////		point_cloud_vec.push_back(point_cloud);
////	}
////
////	rec_timer.stop();
////	cout << "Frame #: " << colorVec.size() << endl;
////	cout << "Reading time: " << rec_timer.time() << endl;
////
////	system("rm ./record/*.png");
////
////	exit(0);
//
//	return point_cloud_vec;
//}

//Mat TableTracker::GenerateBinaryTablePointCloud(const cv::Mat depth_mat, int *point_count)
//{
//	cv::Mat point_cloud = cv::Mat::zeros(height, width, CV_32FC3);
//	uint16_t* depth_data = (uint16_t*)depth_mat.data;
//	float* point_cloud_data = (float*)point_cloud.data;
//    *point_count = 0;
//
////    if (ofs1.is_open()) {
////    	ofs1 << depth_mat << endl;
////		for (int y=0, idx=0; y<height; y++) {
////				for (int x=0; x<width; x++, idx++) {
////					ofs1 << xy_table_data[idx*2] << " " << xy_table_data[idx*2+1] << " " << depth_data[idx] << endl;
////				}
////		}
////		ofs1.close();
////	}
//
//	// Table origin, normal
//    float p0[3], n0[3];
//    if (isCenter) {
//    	for (int i=0;i<3 ;i++) {
//    		p0[i] = plane_origin[i];
//    		n0[i] = plane_normal[i];
//    	}
//    }
//    else
//    {
//    	for (int i=0;i<3 ;i++) {
//			p0[i] = plane_origin[i];
//			n0[i] = plane_normal[i];
//		}
//    }
//	vtkMath::Normalize(n0);
//
//	float proj_point[3];
//	float tf_point[3];
//	float tf_center[3];
//
//	// Calculate center point of transformed PCD to calibration Kinect View
//	bool pass(false);
//	for (int y=0, idx=0; y<height; y++) {
//		for (int x=0; x<width; x++, idx++) {
//			float X = xy_table_data[idx*2]     * depth_data[idx];
//			float Y = xy_table_data[idx*2 + 1] * depth_data[idx];
//			float Z = depth_data[idx];
//			float p[3] = {X,Y,Z};
//			transform->TransformPoint(p, tf_point);
//
//			if ( x == width * 0.5 && y == height * 0.5 ) {
//				tf_center[0] = tf_point[0];
//				tf_center[1] = tf_point[1];
//				tf_center[2] = tf_point[2];
//				pass =true;
//				break;
//			}
//		}
//		if (pass) break;
//	}
//
//	float N0[3], P0[3], P1[3], P2[3];
//	transform->TransformPoint(plane_origin, P0);
//	transform->TransformPoint(plane_normal, N0);
//
//	float calib_point[3] = { 0 - P0[0],
//							 0 - P0[1],
//							 0 - P0[2],
//	};
//
//	for (int i=0;i<3;i++) {
//		P1[i] = P0[i] + N0[i] * top_margin;
//		P2[i] = P0[i] - N0[i] * bot_margin;
//	}
//
//	// Generate Point Cloud Data
//    for (int y=0, idx=0; y<height; y++)
//    {
//    	for (int x=0; x<width; x++, idx++)
//    	{
//    		int channel = y * width * 3 + x * 3;
//
//    		if (depth_data[idx] != 0 && !isnan(xy_table_data[idx*2]) && !isnan(xy_table_data[idx*2+1]) )
//    		{
//    			float X = xy_table_data[idx*2]     * depth_data[idx];
//				float Y = xy_table_data[idx*2 + 1] * depth_data[idx];
//				float Z = depth_data[idx];
//				float p[3] = {X,Y,Z};
//
//    			transform->TransformPoint(p, tf_point);
//
//    			float upperPlane = N0[0]*(tf_point[0]-P1[0]) + N0[1]*(tf_point[1]-P1[1]) + N0[2]*(tf_point[2]-P1[2]);
//				float lowerPlane = N0[0]*(tf_point[0]-P2[0]) + N0[1]*(tf_point[1]-P2[1]) + N0[2]*(tf_point[2]-P2[2]);
//
//    			if ( upperPlane < 0 && lowerPlane > 0) {
//    				point_cloud_data[channel + 0] = tf_point[0] + calib_point[0];
//					point_cloud_data[channel + 1] = tf_point[1] + calib_point[1];
//					point_cloud_data[channel + 2] = tf_point[2];
//					(*point_count)++;
//
////					if(!isCenter) {
////						int node_x = point_cloud_data[channel + 0] * scalingFactor + width * 0.5;
////						int node_y = point_cloud_data[channel + 1] * scalingFactor + height * 0.5;
////						screenPCD[make_pair(node_x,node_y)] = Vec3f(p[0], p[1], p[2]);
////						if (ofs2.is_open()) {
////							ofs2 << "v " << screenPCD[{node_x,node_y}](0) << " " << screenPCD[{node_x,node_y}](1) << " " << screenPCD[{node_x,node_y}](2) << endl;
////						}
////					}
//
//    			}
//    			else
//    			{
//        			point_cloud_data[channel + 0] = nanf("");
//        			point_cloud_data[channel + 1] = nanf("");
//        			point_cloud_data[channel + 2] = nanf("");
//    			}
//    		}
//    		else
//    		{
//    			point_cloud_data[channel + 0] = nanf("");
//    			point_cloud_data[channel + 1] = nanf("");
//    			point_cloud_data[channel + 2] = nanf("");
//    		}
//    	}
//    }
////    ofs2.close();
//
//    return point_cloud;
//}

//Mat TableTracker::ProjectBinaryPointCloud2Image(Mat point_cloud_3d, Mat xy_table)
//{
//	cv::Mat point_cloud_2d = cv::Mat::zeros(xy_table.rows, xy_table.cols, CV_8UC1);
//	uchar* point_cloud_2d_data = (uchar*)point_cloud_2d.data;
//	float* point_cloud_3d_data = (float*)point_cloud_3d.data;
//
//	for (int y=0, idx=0; y < xy_table.rows; y++)
//	{
//		for (int x=0; x < xy_table.cols; x++, idx++)
//		{
//			int channel_3d = y * xy_table.cols * 3 + x * 3;
//
//			if ( isnan(point_cloud_3d_data[channel_3d + 0]) ) continue;
//			int node_x = point_cloud_3d_data[channel_3d + 0] + width * 0.5;
//			if ( node_x < 0 || node_x > width) continue;
//			int node_y = point_cloud_3d_data[channel_3d + 1] + height * 0.5;
//			if ( node_y < 0 || node_y > height) continue;
//
//			point_cloud_2d_data[node_y*width + node_x] = 255;
//		}
//	}
//
//	return point_cloud_2d;
//}
