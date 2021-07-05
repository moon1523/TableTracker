#include "TableTracker.hh"
#include <opencv2/core/eigen.hpp>

bool clicked;
Point2i P1, P2;
Rect cropRect;
float sfInv;
void onMouseCropImage(int event, int x, int y, int f, void *param);

TableTracker::TableTracker(Mat _xy_table, Eigen::Vector4f _quat, Eigen::Vector3f _tvec)
: xy_table(_xy_table), quat(_quat), tvec(_tvec),
  width(_xy_table.cols), height(_xy_table.rows), cam_height(510.),
  deg(1), point_count(0),
  transX(0), transY(0),
  isCrop(false), isRot(false), isMask1(false), isMask2(false), isMove(false), isCenter(false),
  max_sim1(-FLT_MAX), max_sim2(-FLT_MAX)
{
	float* xy_table_data = (float*)xy_table.data;
	pair<float, float> bot_left, top_right;
	bot_left  = make_pair( xy_table_data[0], xy_table_data[1] );
	top_right = make_pair( xy_table_data[width*height*2-2], xy_table_data[width*height*2-1] );
	x_avg_length = (top_right.first - bot_left.first) / width;
	y_avg_length = (top_right.second - bot_left.second) / height;

	scalingFactor = 1.;
	mask1 = cv::Mat::zeros(height, width, CV_8UC1);
	mask2 = cv::Mat::zeros(height, width, CV_8UC1);

	TableInfo();
	maskVec = GenerateTableMask(xy_table, deg);
}

TableTracker::~TableTracker()
{
}

void TableTracker::TableInfo()
{
//	float p0[3] = { 14.59830, -88.69780, 330.00000 };
	float p0[3] = { 14.59830, -88.69780, 330.00000 };

	//	float n0[3] = { -0.03583, 0.82940, 0.55751 };
	margin_mm = 30;
	long_width = 225;
	lat_width = 55;
	height_width = 27;
	long_pos = 0;//25;
	Eigen::Vector3f shifting(0,0,0);
//	float p0[3] = { tvec(0) + shifting(0), tvec(1) + shifting(1), tvec(2) + shifting +(2) };
	cv::Vec3f axisZ;
	eigen2cv(Eigen::Quaternionf(quat) * Eigen::Vector3f(0.f,0.f,1.f), axisZ);
//	float n0[3] = { axisZ(0), axisZ(1), axisZ(1) };
	float n0[3] = { 0.06621, -0.58469, -0.80855 };

	originT[0] = p0[0];
	originT[1] = p0[1];
	originT[2] = p0[2];

	plane_origin[0] = p0[0]; //origin_x;
	plane_origin[1] = p0[1]; //origin_y;
	plane_origin[2] = p0[2]; //origin_z;

	plane_normal[0] = n0[0];
	plane_normal[1] = n0[1];
	plane_normal[2] = n0[2];

	floor_origin[0] = -303.49700;
	floor_origin[1] = 202.28100;
	floor_origin[2] = 606.00000;

	floor_normal[0] = n0[0];
	floor_normal[1] = n0[1];
	floor_normal[2] = n0[2];

	vtkMath::Normalize(floor_normal);
	vtkMath::Normalize(plane_normal);


	float kinectView[3] = {0,0,1};
	vtkMath::Normalize(kinectView);
	transform = vtkSmartPointer<vtkTransform>::New();
	transform = Transform_Kinect_View(floor_normal, kinectView);
	transform->TransformPoint(originT, plane_tfOrigin);
	transform->TransformPoint(originT, plane_tfOrigin2);
	transform->TransformPoint(floor_origin, floor_tfOrigin);
}

vtkSmartPointer<vtkTransform> TableTracker::Transform_Kinect_View(float v1[3], float v2[3])
{
	float axis[3];
	for (int i=0;i<3;i++) v1[i] *= -1;
	vtkMath::Cross(v1, v2, axis);
	const float cosA = vtkMath::Dot(v1, v2);
	const float k = 1.0f / (1.0f + cosA);

	const double affT[16] = { (axis[0] * axis[0] * k) + cosA,    (axis[1] * axis[0] * k) - axis[2], (axis[2] * axis[0] * k) + axis[1], 0,
	           			      (axis[0] * axis[1] * k) + axis[2], (axis[1] * axis[1] * k) + cosA,    (axis[2] * axis[1] * k) - axis[0], 0,
					          (axis[0] * axis[2] * k) - axis[1], (axis[1] * axis[2] * k) + axis[0], (axis[2] * axis[2] * k) + cosA,    0,
							  0, 0, 0, 1 };
	transform->Identity();
	transform->SetMatrix(affT);

	return transform;
}

bool TableTracker::FindTableCenter()
{
	Timer proc_timer;
	proc_timer.start();
	Mat table_pcd = GenerateTablePointCloud(depth_mat, xy_table, &point_count);
	Mat table_pcd2img = ProjectPointCloud2Image(table_pcd, xy_table);

	int x_node = !isRot ? lat_width * scalingFactor : long_width * scalingFactor;
	int y_node = !isRot ? long_width  * scalingFactor : lat_width  * scalingFactor;

	// Set starting point of mask
	Mat roi, copy;
	table_pcd2img.copyTo(roi);
	table_pcd2img.copyTo(copy);
	cv::Rect bounds(0, 0, roi.cols, roi.rows);
	cv::setMouseCallback("Drag Table", onMouseCropImage, &roi);

	if (clicked) isCrop = true;
	if (cropRect.width > 0 && clicked == false) {
		roi = table_pcd2img(cropRect & bounds);
		if (isCrop) {
			int cropBox_width = P2.x - P1.x;
			int cropBox_height = P2.y - P1.y;
			cout << "CropBox Width: " << cropBox_width << endl;
			cout << "CropBox Height: " << cropBox_height << endl;

			if (cropBox_width > x_node && cropBox_height > y_node ) {
				cout << "Set ROI box" << endl;
				transX = P1.x, transY = P1.y;
				isMove = true;
			}
			else {
				cout << "Reset ROI box" << endl;
				transX, transY = 0;
			}
			isCrop = false;
		}
	}
	else table_pcd2img.copyTo(roi);
	cv::rectangle(copy, P1, P2, CV_RGB(255,255,255), 2);
	cv::imshow("Drag Table", copy);

	// Start dragging mask
	Mat match_xor, match_and, match_mat;
	int mask_sum = cv::sum(maskVec[0]).val[0];

	cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
	uchar* mask_data = (uchar*)mask.data;
	Point2i p1( transX, transY );
	Point2i p2( x_node + transX, y_node + transY );

	if(!isMask1 || !isMask2) {
		for (int y=0; y<y_node; y++) {
			for (int x=0; x<x_node; x++) {
				mask_data[(y + transY) * width + (x + transX)] = 255;
			}
		}

		cv::bitwise_and(mask, table_pcd2img, match_and);
		cv::bitwise_xor(mask, table_pcd2img, match_xor);
		double similarity = cv::sum(match_and).val[0] / mask_sum;

		if (!isRot) {
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
			if(isMove) transY++;
			if (transY == (P2.y - y_node)) {
				isMove=false;
				transX = 0, transY = 0;
				isMask1 = true;
				if (isMask1 && isRot)
					isMask2 = true;
				cout << "Rotate the table 90 degrees" << endl;
				isRot = true;
			}
		}
		if(isMove) transX++;

		proc_timer.stop();
		putText(match_xor, "TransX: "  + to_string(P1.x) + "/" + to_string(transX) + "/" + to_string(P2.x - x_node),
				cv::Point(10,height-90), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
		putText(match_xor, "TransY: "  + to_string(P1.y) + "/" + to_string(transY) + "/" + to_string(P2.y - y_node),
				cv::Point(10,height-60), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
		putText(match_xor, "Similarity: " + to_string(similarity),
				cv::Point(10,height-30), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
		putText(match_xor, "Processing Time (s): " + to_string(proc_timer.time()),
				cv::Point(10,height-120), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
		imshow("CenterMask", match_xor);
		waitKey(1);
	}
	else
	{
		// Find rotation center of table masks
		cv::rectangle(mask1, mask1_p1, mask1_p2, CV_RGB(255,255,255), 2);
		cv::rectangle(mask2, mask2_p1, mask2_p2, CV_RGB(255,255,255), 2);
		Point2i mask1A(mask1_p1.x, mask1_p1.y);
		Point2i mask1B(mask1_p1.x, mask1_p2.y);
		Point2i mask2A(mask2_p2.x, mask2_p1.y);
		Point2i mask2B(mask2_p1.x, mask2_p1.y);
		Point2i midA_((mask1A.x + mask2A.x) * 0.5, (mask1A.y + mask2A.y) * 0.5);
		Point2i midB_((mask1B.x + mask2B.x) * 0.5, (mask1B.y + mask2B.y) * 0.5);

		Mat draw = cv::Mat::zeros(height, width, CV_8UC3);
		cv::rectangle(draw, mask1_p1, mask1_p2, CV_RGB(255,255,255), 2);
		cv::rectangle(draw, mask2_p1, mask2_p2, CV_RGB(255,255,255), 2);
		cv::line(draw, mask1A, mask2A, CV_RGB(255,255,0), 2, 8, 0);
		cv::line(draw, mask1B, mask2B, CV_RGB(255,255,0), 2, 8, 0);
		cv::circle(draw, midA_, 1.0, CV_RGB(255,0,0), 3, 8, 0);
		cv::circle(draw, midB_, 1.0, CV_RGB(255,0,0), 3, 8, 0);

		int x1 = mask2A.x-mask1A.x; int y1 = mask2A.y-mask1A.y;
		int x2 = mask2B.x-mask1B.x;	int y2 = mask2B.y-mask1B.y;

		Eigen::Vector2f midA((mask1A.x + mask2A.x) * 0.5, (mask1A.y + mask2A.y) * 0.5);
		Eigen::Vector2f midB((mask1B.x + mask2B.x) * 0.5, (mask1B.y + mask2B.y) * 0.5);
		Eigen::Vector2f pA(midA.x() - y1, midA.y() + x1);
		Eigen::Vector2f pB(midB.x() - y2, midB.y() + x2);

		Point2i pA_(midA.x() - y1, midA.y() + x1);
		Point2i pB_(midB.x() - y2, midB.y() + x2);
		cv::circle(draw, pA_, 1.0, CV_RGB(255,0,0), 3, 8, 0);
		cv::circle(draw, pB_, 1.0, CV_RGB(255,0,0), 3, 8, 0);

		using Line2 = Eigen::Hyperplane<float,2>;
		Line2 lineA = Line2::Through(midA, pA);
		Line2 lineB = Line2::Through(midB, pB);

		cv::line(draw, midA_, pA_, CV_RGB(0,255,0), 2, 8, 0);
		cv::line(draw, midB_, pB_, CV_RGB(0,255,0), 2, 8, 0);

		Eigen::Vector2f centerF = lineA.intersection(lineB);
		Point2i centerI((int)centerF.x(), (int)centerF.y());
		tableCenter = centerI;
		cv::circle(draw, centerI, 1.0, CV_RGB(0,0,255), 3, 8, 0);

		imshow("Mask1", mask1);
		imshow("Mask2", mask2);
		imshow("Drawing", draw);
		waitKey(1);

		int aimSize(10);
		int aimPP;
		for (int y=0; y<aimSize*2;y++) {
			for (int x=0; x<aimSzie*2; x++) {
				aimPP += table_pcd2img.at<uh
			}
		}

		isCenter = true;
	}
	return isCenter;
}

void TableTracker::SetTableOrigin(float x, float y, float z)
{
	// xyz => Kinect View
	// +x: right, -x: left
	// +y: down, -y: up
	// +z: push, -z: pull
	trans_table[0] = x;
	trans_table[1] = -y;
	trans_table[2] = -z;

	for (int i=0;i<3;i++) {
		plane_tfOrigin[i] += trans_table[i];
		plane_tfOrigin2[i] = plane_tfOrigin[i];
	}

	transform->Inverse();
	transform->TransformPoint(plane_tfOrigin, plane_origin);

	for (int i=0;i<3;i++) {
		plane_tfOrigin[i] -= trans_table[i];
	}
	transform->Inverse();
}

void TableTracker::ProcessCurrentFrame()
{
	procTimer.start();
	Mat table_pcd = GenerateTablePointCloud(depth_mat, xy_table, &point_count);
	Mat table_pcd2img = ProjectPointCloud2Image(table_pcd, xy_table);
	results = MatcingData(maskVec, table_pcd2img, deg);
	procTimer.stop();
}

void TableTracker::Render()
{
    Mat match_mat = get<2>(results);
	putText(match_mat, "Table Degree: " + to_string(get<0>(results)),
							cv::Point(10,depth_mat.rows-90), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
	putText(match_mat, "Similarity: " + to_string(get<1>(results)) + "%",
										cv::Point(10,depth_mat.rows-60), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
	putText(match_mat, "Processing Time (s): " + to_string(procTimer.time()),
										cv::Point(10,depth_mat.rows-30), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255), 2);
	cv::circle(match_mat, Point(depth_mat.cols*0.5, depth_mat.rows*0.5), 1.0, Scalar::all(100), 3, 8, 0);
	cv::circle(match_mat, tableCenter, 1.0, Scalar::all(255), 3, 8, 0);

	resize(color_mat, color_resize, Size(1280,720));
	imshow("Color", color_resize);
	imshow("TableTracker", match_mat);
}

vector<Mat> TableTracker::GenerateTableMask(Mat xy_table, int deg)
{
	int x_node = lat_width * 0.5 * scalingFactor;  // / cam_height / x_avg_length;
	int y_nodeBot = ( long_width * 0.5 - long_pos ) * scalingFactor;// / cam_height / y_avg_length;
	int y_nodeTop = ( long_width * 0.5 + long_pos ) * scalingFactor;// / cam_height / y_avg_length;

	cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
	uchar* mask_data = (uchar*)mask.data;

	int mask_sum(0);

	if ( y_nodeTop > height*0.5 && y_nodeBot < height*0.5 && x_node < width*0.5 )
	{
		for (int y=0; y<height*0.5+y_nodeBot; y++) {
			for (int x=width*0.5-x_node; x<width*0.5+x_node; x++) {
				mask_data[y*width+x] = 255;
				mask_sum += mask_data[y*width+x];
			}
		}
	}
	else if ( y_nodeTop > height*0.5 && y_nodeBot > height*0.5 && x_node < width*0.5 )
	{
		for (int y=0; y<height; y++) {
			for (int x=width*0.5-x_node; x<width*0.5+x_node; x++) {
				mask_data[y*width+x] = 255;
				mask_sum += mask_data[y*width+x];
			}
		}
	}
	else if ( y_nodeTop > height*0.5 && y_nodeBot > height*0.5 && x_node > width*0.5 )
	{
		for (int y=0; y<height; y++) {
			for (int x=0; x<width; x++) {
				mask_data[y*width+x] = 255;
				mask_sum += mask_data[y*width+x];
			}
		}
	}
	else
	{
		for (int y=height*0.5-y_nodeTop; y<height*0.5+y_nodeBot; y++) {
			for (int x=width*0.5-x_node; x<width*0.5+x_node; x++) {
				mask_data[y*width+x] = 255;
				mask_sum += mask_data[y*width+x];
			}
		}
	}

	vector<cv::Mat> maskVec;
	for (int i=0; i<=90/deg; i++)
	{
		cv::Mat mask_rot;
		mask.copyTo(mask_rot);
		cv::Mat rotM = getRotationMatrix2D(Point(width*0.5, height*0.5), -deg*i, 1.0);
		warpAffine(mask, mask_rot, rotM, mask.size());
		maskVec.push_back(mask_rot);
	}

//	cout << " Press 'c' to skip the present frame " << endl;
//	cout << " Press 'q' to quit " << endl;
//	int idx(0);
//	while(1)
//	{
//		if (idx > 90/deg) idx = 0;
//		cv::circle(maskVec[idx], Point(width*0.5, height*0.5), 1.0, Scalar::all(100), 3, 8, 0);
//		cv::putText(maskVec[idx], "Resolution: "+ to_string(width)+ "x" + to_string(height),
//								 Point(10,20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255), 1);
//		cv::putText(maskVec[idx], "Table Degree: "+ to_string(idx*deg),
//								 Point(10,40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255), 1);
//
//		imshow("mask_rot", maskVec[idx++]);
//
//		char key = (char)cv::waitKey(1000);
//
//		if (key == 'c') continue;
//		if (key == 'q') {
//			cv::destroyAllWindows();
//			break;
//		}
//	}

	return maskVec;
}


Mat TableTracker::GenerateTablePointCloud(const Mat depth_mat, const Mat xy_table, int *point_count)
{
	cv::Mat point_cloud = cv::Mat::zeros(height, width, CV_32FC3);
	uint16_t* depth_data = (uint16_t*)depth_mat.data;
	float* xy_table_data = (float*)xy_table.data;
	float* point_cloud_data = (float*)point_cloud.data;
    *point_count = 0;

	// Table origin, normal
    float p0[3], n0[3];
    if (isCenter) {
    	for (int i=0;i<3 ;i++) {
    		p0[i] = plane_origin[i];
    		n0[i] = plane_normal[i];
    	}
    }
    else
    {
    	for (int i=0;i<3 ;i++) {
			p0[i] = plane_origin[i];
			n0[i] = plane_normal[i];
		}
    }
	vtkMath::Normalize(n0);

	float proj_point[3];
	float tf_point[3];
	float tf_center[3];

	// Calculate center point of transformed PCD to calibration Kinect View
	bool pass(false);
	for (int y=0, idx=0; y<height; y++) {
		for (int x=0; x<width; x++, idx++) {
			float X = xy_table_data[idx*2]     * depth_data[idx];
			float Y = xy_table_data[idx*2 + 1] * depth_data[idx];
			float Z = depth_data[idx];
			float p[3] = {X,Y,Z};
			transform->TransformPoint(p, tf_point);

			if ( x == width * 0.5 && y == height * 0.5 ) {
				tf_center[0] = tf_point[0];
				tf_center[1] = tf_point[1];
				tf_center[2] = tf_point[2];
				pass =true;
				break;
			}
		}
		if (pass) break;
	}

	float N0[3], P0[3], P1[3], P2[3];
	transform->TransformPoint(plane_origin, P0);
	transform->TransformPoint(plane_normal, N0);

	float calib_point[3] = { 0 - P0[0],
							 0 - P0[1],
							 0 - P0[2],
	};

	for (int i=0;i<3;i++) {
		P1[i] = P0[i] + N0[i] * margin_mm;
		P2[i] = P0[i] - N0[i] * margin_mm;
	}

	// Generate Point Cloud Data
    for (int y=0, idx=0; y<height; y++)
    {
    	for (int x=0; x<width; x++, idx++)
    	{
    		int channel = y * width * 3 + x * 3;

    		if (depth_data[idx] != 0 && !isnan(xy_table_data[idx*2]) && !isnan(xy_table_data[idx*2+1]) )
    		{
    			float X = xy_table_data[idx*2]     * depth_data[idx];
				float Y = xy_table_data[idx*2 + 1] * depth_data[idx];
				float Z = depth_data[idx];
				float p[3] = {X,Y,Z};

    			transform->TransformPoint(p, tf_point);

    			float upperPlane = N0[0]*(tf_point[0]-P1[0]) + N0[1]*(tf_point[1]-P1[1]) + N0[2]*(tf_point[2]-P1[2]);
				float lowerPlane = N0[0]*(tf_point[0]-P2[0]) + N0[1]*(tf_point[1]-P2[1]) + N0[2]*(tf_point[2]-P2[2]);

    			if ( upperPlane < 0 && lowerPlane > 0) {
    				point_cloud_data[channel + 0] = tf_point[0] + calib_point[0];
					point_cloud_data[channel + 1] = tf_point[1] + calib_point[1];
					point_cloud_data[channel + 2] = tf_point[2];
					(*point_count)++;
    			}
    			else
    			{
        			point_cloud_data[channel + 0] = nanf("");
        			point_cloud_data[channel + 1] = nanf("");
        			point_cloud_data[channel + 2] = nanf("");
    			}
    		}
    		else
    		{
    			point_cloud_data[channel + 0] = nanf("");
    			point_cloud_data[channel + 1] = nanf("");
    			point_cloud_data[channel + 2] = nanf("");
    		}
    	}
    }

    return point_cloud;
}

Mat TableTracker::ProjectPointCloud2Image(Mat point_cloud_3d, Mat xy_table)
{
	cv::Mat point_cloud_2d = cv::Mat::zeros(xy_table.rows, xy_table.cols, CV_8UC1);
	uchar* point_cloud_2d_data = (uchar*)point_cloud_2d.data;
	float* point_cloud_3d_data = (float*)point_cloud_3d.data;

	for (int y=0, idx=0; y < xy_table.rows; y++)
	{
		for (int x=0; x < xy_table.cols; x++, idx++)
		{
			int channel_3d = y * xy_table.cols * 3 + x * 3;

			if ( isnan(point_cloud_3d_data[channel_3d + 0]) ) continue;
			int node_x = point_cloud_3d_data[channel_3d + 0] * scalingFactor + width * 0.5;
			if ( node_x < 0 || node_x > width) continue;
			int node_y = point_cloud_3d_data[channel_3d + 1] * scalingFactor + height * 0.5;
			if ( node_y < 0 || node_y > height) continue;

			point_cloud_2d_data[node_y*width + node_x] = 255;
		}
	}

	return point_cloud_2d;
}

tuple<double, double, Mat> TableTracker::MatcingData(vector<Mat> maskVec, Mat point_cloud_2d, int deg)
{
	Mat match_xor, match_and, match_mat;
	int mask_sum = cv::sum(maskVec[0]).val[0];

	double max_sim(0);
	double match_deg(0);
	int idx(0);
	for (int i=0; i<=90/deg; i++)
	{
		cv::bitwise_and(maskVec[idx], point_cloud_2d, match_and);
		double similarity = cv::sum(match_and).val[0] / mask_sum;

		if (max_sim < similarity) {
			cv::bitwise_xor(maskVec[idx], point_cloud_2d, match_xor);
			max_sim = similarity;
			match_deg = idx * deg;
			match_mat = match_xor;
		}
		idx++;
	}

	if (match_mat.empty())
		match_mat = point_cloud_2d;

	return make_tuple(match_deg, max_sim, match_mat);
}

Mat TableTracker::GeneratePointCloud(const cv::Mat depth_mat, const cv::Mat xy_table, int *point_count)
{
	cv::Mat point_cloud = cv::Mat::zeros(height, width, CV_32FC3);
	uint16_t* depth_data = (uint16_t*)depth_mat.data;
	float* xy_table_data = (float*)xy_table.data;
	float* point_cloud_data = (float*)point_cloud.data;

	// Calculate center point of transformed PCD to calibration Kinect View
	float tf_center[3];
	bool pass(false);
	for (int y=0, idx=0; y<height; y++) {
		for (int x=0; x<width; x++, idx++) {
			float X = xy_table_data[idx*2]     * depth_data[idx];
			float Y = xy_table_data[idx*2 + 1] * depth_data[idx];
			float Z = depth_data[idx];
			float p[3] = {X,Y,Z};

			if ( x == width * 0.5 && y == height * 0.5 ) {
				tf_center[0] = X;
				tf_center[1] = Y;
				tf_center[2] = Z;
				pass =true;
				break;
			}
		}
		if (pass) break;
	}

	float calib_point[3] = { tf_center[0] - plane_origin[0], tf_center[1] - plane_origin[1], tf_center[2] - plane_origin[2] };
	float p0[3] = { plane_origin[0], plane_origin[1], plane_origin[2] };
	float n0[3] = { plane_normal[0], plane_normal[1], plane_normal[2] };
	vtkMath::Normalize(n0);

	float p1[3], p2[3];

	for (int i=0;i<3;i++) {
		p1[i] = p0[i] + n0[i] * margin_mm;
		p2[i] = p0[i] - n0[i] * margin_mm;
	}

	*point_count = 0;
	for (int y=0, idx=0; y<height; y++) {
		for (int x=0; x<width; x++, idx++) {
			int channel = y * width * 3 + x * 3;
			if (depth_data[idx] != 0 && !isnan(xy_table_data[idx*2]) && !isnan(xy_table_data[idx*2+1]) )
			{
				float X = xy_table_data[idx*2]     * depth_data[idx];
				float Y = xy_table_data[idx*2 + 1] * depth_data[idx];
				float Z = depth_data[idx];
				float p[3] = {X,Y,Z};

				float upperPlane = n0[0]*(X-p1[0]) + n0[1]*(Y-p1[1]) + n0[2]*(Z-p1[2]);
				float lowerPlane = n0[0]*(X-p2[0]) + n0[1]*(Y-p2[1]) + n0[2]*(Z-p2[2]);

//				if ( upperPlane < 0 && lowerPlane > 0) {
					point_cloud_data[channel + 0] = xy_table_data[idx*2]     * depth_data[idx];// + calib_point[0];
					point_cloud_data[channel + 1] = xy_table_data[idx*2 + 1] * depth_data[idx];// + calib_point[1];
					point_cloud_data[channel + 2] = depth_data[idx];// + calib_point[2];
					(*point_count)++;
//				}
//				else
//				{
//					point_cloud_data[channel + 0] = nanf("");
//					point_cloud_data[channel + 1] = nanf("");
//					point_cloud_data[channel + 2] = nanf("");
//				}

			}
			else
			{
				point_cloud_data[channel + 0] = nanf("");
				point_cloud_data[channel + 1] = nanf("");
				point_cloud_data[channel + 2] = nanf("");
			}
		}
	}

	return point_cloud;
}

void TableTracker::WritePointCloud(string file_name, const cv::Mat point_cloud, int point_count)
{
	float* point_cloud_data = (float*)point_cloud.data;

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
}

vector<Mat> TableTracker::Read_K4A_MKV_Record(string fileName, Mat xy_table)
{
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

	// https://docs.microsoft.com/en-us/azure/kinect-dk/record-file-format
	// -map 0:1 (depth)
	// -map 0:0 (color)
	// -vsync 0 (match frame rate)
	system(("ffmpeg -i " + fileName + " -map 0:1 -vsync 0 ./record/depth%d.png").c_str());

	vector<Mat> point_cloud_vec;
	for (size_t i=0; i<colorVec.size(); i++) {
		string depthFile = "./record/depth" + to_string(i+1) + ".png";
		depth_frame = imread(depthFile, IMREAD_ANYDEPTH );
		depthVec.push_back(depth_frame);
//		imshow("depth", depth_frame);
//		char key = (char)waitKey(1000/30);
//		if (key == 'q') {
//			break;
//		}

		Mat point_cloud = Mat::zeros(height, width, CV_32FC3);
		float* xy_table_data = (float*)xy_table.data;
		float* point_cloud_data = (float*)point_cloud.data;
		uint16_t* depth_data = (uint16_t*)depthVec[i].data;

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
		point_cloud_vec.push_back(point_cloud);
	}

	rec_timer.stop();
	cout << "Frame #: " << colorVec.size() << endl;
	cout << "Reading time: " << rec_timer.time() << endl;

	system("rm ./record/*.png");

	exit(0);

	return point_cloud_vec;
}

void onMouseCropImage(int event, int x, int y, int f, void *param)
{
    switch (event)
    {
    case EVENT_LBUTTONDOWN:
        clicked = true;
        P1.x = x;
        P1.y = y;
        P2.x = x;
        P2.y = y;
        break;
    case EVENT_LBUTTONUP:
        P2.x = x;
        P2.y = y;
        clicked = false;
        break;
    case EVENT_MOUSEMOVE:
        if (clicked)
        {
            P2.x = x;
            P2.y = y;
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











