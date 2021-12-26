#include "TableTracker.hh"

bool clicked;
cv::Point2i P1, P2;
cv::Rect cropRect;
float sfInv;

void onMouseCropImage(int event, int x, int y, int f, void *param);


TableTracker::TableTracker(string _config_file, int _image_width, int _image_height)
: image_width(_image_width), image_height(_image_height),
  isMove(false), isRot(false), isCheck(false), isFirst(true), ocr(0,0,0), frameNo(0), mean_angle(0), mean_similarity(0)
{
	ReadConfigData(_config_file);
	ConfigVirtualCamera();
	ConfigTableData();
	ConfigMaskData();
	PrintConfigData();
	ofs.open("result.out");
	ofs << setw(10) << "Time (sec)" << setw(10) << "Angle (deg)" << setw(10) << "Similarity" << endl;
}

TableTracker::~TableTracker() { ofs.close(); }

cv::Mat TableTracker::GenerateTablePointCloud(const k4a_image_t point_cloud_image, const k4a_image_t depth_image)
{
	int16_t *point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(point_cloud_image);
	uint8_t *depth_image_data = k4a_image_get_buffer(depth_image);

	cv::Mat point_cloud_2d = cv::Mat::zeros(mask_height, mask_width, CV_8UC1);
	uint8_t* point_cloud_2d_data = (uint8_t*)point_cloud_2d.data;

	float tf_point[3];
	for (int i=0; i<image_width*image_height; i++)
	{
		float Z = point_cloud_image_data[ 3 * i + 2 ];
			if (Z == 0) continue;
		float X = point_cloud_image_data[ 3 * i + 0 ];
		float Y = point_cloud_image_data[ 3 * i + 1 ];

		float point[3] = {X,Y,Z};
		transform->TransformPoint(point, tf_point);

		float topPlane = tf_world_normal[0] * (tf_point[0] - tf_table_topPoint[0]) +
						 tf_world_normal[1] * (tf_point[1] - tf_table_topPoint[1]) +
						 tf_world_normal[2] * (tf_point[2] - tf_table_topPoint[2]);
		if (topPlane > 0) continue;
		float botPlane = tf_world_normal[0] * (tf_point[0] - tf_table_botPoint[0]) +
						 tf_world_normal[1] * (tf_point[1] - tf_table_botPoint[1]) +
						 tf_world_normal[2] * (tf_point[2] - tf_table_botPoint[2]);
		if (botPlane < 0) continue;

		int pixel_x = tf_point[0] * pixelPerLength + mask_width  * 0.5 ;
		if (pixel_x > mask_width) continue;
		if (pixel_x < 0) continue;

		int pixel_y = tf_point[1] * pixelPerLength + mask_height * 0.5 ;
		if (pixel_y > mask_height) continue;
		if (pixel_y < 0) continue;

		point_cloud_2d_data[ mask_width*pixel_y + pixel_x ] = 255;
	}

	return point_cloud_2d;
}

void TableTracker::ProcessCurrentFrame()
{
//	GenerateColorTablePointCloud(point_img, color_img);
	binPCD = GenerateTablePointCloud(point_img, depth_img);

//	match_results_prev = match_results;
	match_results = MatchingData();

	//	if(isFirst) {
//		match_results_prev = match_results;
//		isFirst = false;
//	}

//	// Threshold
//	if (fabs(get<0>(match_results_prev) - get<0>(match_results)) > 10) {
//		cout << "frame #: " << frameNo << endl;
//		cout << "angle spikes! " << endl;
//		get<0>(match_results) = get<0>(match_results_prev);
//		get<1>(match_results) = get<1>(match_results_prev);
//		cv::Mat match_xor;
//		cv::bitwise_xor(mask_vec[(get<0>(match_results)-mask_minRotDeg)/mask_deg], binPCD, match_xor);
//		get<2>(match_results) = match_xor;
//
//		get<0>(match_results_filter) = get<0>(match_results);
//	}







//	else if (fabs(get<1>(match_results_prev) - get<1>(match_results)) > 0.2) {
//		cout << "frame #: " << frameNo << endl;
//		cout << "similarity spikes! " << endl;
//		get<0>(match_results) = get<0>(match_results_prev);
//		get<1>(match_results) = get<1>(match_results_prev);
//		cv::Mat match_xor;
//		cv::bitwise_xor(mask_vec[(get<0>(match_results)-mask_minRotDeg)/mask_deg], binPCD, match_xor);
//		get<2>(match_results) = match_xor;
//	}

//	// hampel filter
//	match_angleList.push_back(get<0>(match_results));
//	if (match_angleList.size() == 5) {
//		VectorXi v, vv;
//		int idx(0);
//		double x;
//		v.resize(match_angleList.size());
//
//		for (auto itr: match_angleList) {
//			v(idx++) = itr;
//			if (idx == 2)
//				x = itr;
//		}
//		sort(v.data(),v.data()+v.size());
//		double median = v(2);
//		vv = VectorXi::Constant(match_angleList.size(), median);
//		double sdev = 1.4826*(v - vv).cwiseAbs()(2);
//
//		if ( fabs(x - median) > 3*sdev  ) {
//			cout << "hampel filter detected spike: " << x << " -> " << median << endl;
//			x = median;
//		}
//
//		get<0>(match_results_filter) = x;
//		cv::Mat match_xor;
//		cv::bitwise_xor(mask_vec[((int)x-mask_minRotDeg)/mask_deg], binPCD, match_xor);
//		get<2>(match_results_filter) = match_xor;
//
//		match_angleList.pop_front();
//	}



//		// 1-D median filter
//		match_angleList.push_back(get<0>(match_results_prev));
//		if (match_angleList.size() == 5) {
//			VectorXi v;
//			int idx(0);
//			v.resize(match_angleList.size());
//
//			for (auto itr:match_angleList) {
//				v(idx++) = itr;
//			}
//			double median = v(2);
//
//			cout << v << endl;
//			for (int i=0; i<v.size(); i++) {
//				if (v(i) > median) {
//					cout << "frame #:" << frameNo << endl;
//					cout << "spikes" << endl;
//					v(i) = median;
//				}
//			}
//
//			get<0>(match_results_filter) = v(4);
//
//			cv::Mat match_xor;
//			cv::bitwise_xor(mask_vec[((int)median-mask_minRotDeg)/mask_deg], binPCD, match_xor);
//			get<2>(match_results_filter) = match_xor;
//
//
//			match_angleList.pop_front();
//		}




//	// IQR method
//	match_angleList.push_back(get<0>(match_results));
//	if (match_angleList.size() == 21) {
//		VectorXi v;
//		int idx(0);
//		int angle;
//		v.resize(match_angleList.size());
//
//		for (auto itr:match_angleList) {
//			v(idx++) = itr;
//			if (idx == 11)
//				angle = itr;
//		}
//		sort(v.data(),v.data()+v.size());
//
//		double median = v(11);
//		double q1 = (v(4) + v(5)) * 0.5;
//		double q3 = (v(15) + v(16)) * 0.5;
//		double iqr = q3 - q1;
//		double lower_bound = q1 - 1.5 * iqr;
//		double upper_bound = q3 + 1.5 * iqr;
//		double mean = v.mean();
//
//		if (angle < lower_bound || angle > upper_bound) {
//			cout << "frame #: " << frameNo << endl;
//			cout << "present matching angle: " << angle << endl;
//			cout << "window:" << lower_bound << "~" << upper_bound << endl;
//			cout << "median data: " << median << endl;
//
//			get<0>(match_results_filter) = median;
//			cv::Mat match_xor;
//			cv::bitwise_xor(mask_vec[((int)median-mask_minRotDeg)/mask_deg], binPCD, match_xor);
//			get<2>(match_results_filter) = match_xor;
//			cout << "spikes! :" << get<0>(match_results) << endl;
//			cout << endl;
//		}
//		match_angleList.pop_front();
//	}

}


void TableTracker::Render(double time)
{
	cv::Mat match_mat;


//	if (!isCheck) {
		match_mat = get<2>(match_results);
		cv::putText(match_mat, "Similarity: " + to_string(get<1>(match_results)),
				cv::Point(10,mask_height-60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 1.5);
		cv::putText(match_mat, "Table Degree (Raw): " + to_string(get<0>(match_results)),
				cv::Point(10,mask_height-40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 1.5);
		ofs << setw(10) << time << setw(10) << get<0>(match_results) << setw(10) << get<1>(match_results) << endl;
//	}
//	else {
//		cout << "!!!" << endl;
//		match_mat = get<2>(match_results_filter);
//		cv::putText(match_mat, "Similarity (Raw/Filtered): " + to_string(get<1>(match_results))+"/"+to_string(get<1>(match_results_filter)),
//				cv::Point(10,mask_height-60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 1.5);
//		cv::putText(match_mat, "Table Degree (Raw/Filtered): " + to_string(get<0>(match_results)) +"/"+to_string(get<0>(match_results_filter)),
//				cv::Point(10,mask_height-40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 1.5);
//		isCheck = false;
//	}

	cv::putText(match_mat, "Frame #: " + to_string(frameNo++),
				cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 1.5);
	cv::putText(match_mat, "Frame Time (s): " + to_string(time),
			cv::Point(10,mask_height-20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 1.5);
	cv::circle(match_mat, cv::Point(table_rotCenter_pixel_x,table_rotCenter_pixel_y), 2.0,cv::Scalar(100), 3, 8, 0);
	cv::Mat match_color;
	cv::cvtColor(match_mat, match_color, cv::COLOR_GRAY2BGR);
	cv::imshow("bin", match_color);

	for (int i=0; i<3; i++) {
		tf_table_topPoint[i] -= tf_world_axisX[i] * ocr[i] + tf_world_axisY[i] * ocr[i] + tf_world_axisZ[i] * ocr[i];
		tf_table_botPoint[i] -= tf_world_axisX[i] * ocr[i] + tf_world_axisY[i] * ocr[i] + tf_world_axisZ[i] * ocr[i];
		tf_table_position[i] -= tf_world_axisX[i] * ocr[i] + tf_world_axisY[i] * ocr[i] + tf_world_axisZ[i] * ocr[i];
	}
}

tuple<double, double, cv::Mat> TableTracker::MatchingData()
{
	cv::Mat match_xor, match_and, match_mat;
	double max_sim(0);
	double match_deg(0);
	int idx(0);
	for (int i=mask_minRotDeg; i<=mask_maxRotDeg; i+=mask_deg)
	{
		// Matching masks and point cloud data
		cv::bitwise_and(mask_vec[idx], binPCD, match_and);
		double similarity = cv::sum(match_and).val[0] / mask_sum;

		// Find the most similar mask
		if (max_sim < similarity) {
			// bitwise xor is used to view, not affect the mathcing results
			cv::bitwise_xor(mask_vec[idx], binPCD, match_xor);
			match_mat = match_xor;
			max_sim = similarity;
			match_deg = mask_minRotDeg + idx * mask_deg;
		}
		idx++;
	}
	if (match_mat.empty())
		match_mat = binPCD;

	return make_tuple(match_deg, max_sim, match_mat);
}



void TableTracker::ReadConfigData(string configData)
{
	cv::FileStorage fs(configData, cv::FileStorage::READ);
	if (!fs.isOpened()) {
		cerr << "Fail to read config data" << endl; exit(1);
	}
	cv::Mat qMat, tMat, vMat, rMat, pMat;
	MatrixXd wqMat, wtMat, vtMat, rtMat, ptMat;

	fs["World_Quaternion(x,y,z,w)"] >> qMat;
	fs["World_Translation(cm)"] >> tMat;
	fs["VirtualCamera_Position(cm)"] >> vMat;
	fs["Table_Position(mm)"] >> pMat;
	fs["Table_RotationCenter(cm)"] >> rMat;
	fs["Table_Width(cm)"] >> table_width;
	fs["Table_Length(cm)"] >> table_length;
	fs["Table_Height(cm)"] >> table_height;
	fs["Table_TopMargin(cm)"] >> table_topMargin;
	fs["Table_BotMargin(cm)"] >> table_botMargin;
	fs["Mask_CropWidth(px)"] >> mask_width;
	fs["Mask_CropHeight(px)"] >> mask_height;
	fs["Mask_Rot(deg)"] >> mask_deg;
	fs["Mask_MaxRot(deg)"] >> mask_maxRotDeg;
	fs["Mask_MinRot(deg)"] >> mask_minRotDeg;
	fs["PixelPerLength(px/mm)"] >> pixelPerLength;
	fs.release();

	cv::cv2eigen(qMat,wqMat);
	world_quat.x() = wqMat(0,0);
	world_quat.y() = wqMat(0,1);
	world_quat.z() = wqMat(0,2);
	world_quat.w() = wqMat(0,3);
	cv::cv2eigen(tMat, wtMat);
	world_trans = Vector3d(wtMat(0)*10, wtMat(1)*10, wtMat(2)*10);
	cv::cv2eigen(vMat, vtMat);
	vcam_position = world_trans + world_quat.matrix()*Vector3d(vtMat(0,0)*10, vtMat(0,1)*10, vtMat(0,2)*10);

	cv::cv2eigen(pMat, ptMat);
	table_position[0] = ptMat(0); table_position[1] = ptMat(1); table_position[2] = ptMat(2);

	cv::cv2eigen(rMat, rtMat);
	table_rotCenter = world_trans + world_quat.matrix()*Vector3d(rtMat(0)*10, rtMat(1)*10, rtMat(2)*10);

	table_width *= 10; table_length *= 10; table_height *= 10;
	table_topMargin *= 10; table_botMargin *= 10;
}

void TableTracker::ConfigVirtualCamera()
{
	transform = vtkSmartPointer<vtkTransform>::New();
	transform->Identity();
	transform->Translate(-vcam_position(0), -vcam_position(1), -vcam_position(2));

	double theta;
	double axis_z[3], axis_x[3];

	// Align Z
	double v1[3]; // world down vector;
	for (int i=0;i<3;i++) v1[i] = -(world_quat.matrix() * Vector3d::UnitZ())(i);
	double v2[3] = {0,0,1}; // camera z
	theta = acos(vtkMath::Dot(v1,v2)) * 180 / M_PI;
	vtkMath::Cross(v1, v2, axis_z);
	vtkMath::Normalize(axis_z);
	transform->RotateWXYZ(theta, axis_z);

	// Align X
	double xd[3];
	for (int i=0;i<3;i++) xd[i] = (world_quat.matrix() * Vector3d::UnitX())(i);
	double v3[3], v3_proj[3];
	transform->TransformVector(xd,v3);
	vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();
	plane->SetOrigin(0.,0.,0.);
	plane->SetNormal(0.,0.,1.);
	double origin[3] = {0.,0.,0.};
	double normal[3] = {0.,0.,1.};
	plane->ProjectVector(v3, origin, normal, v3_proj);

	double v4[3] = {1.,0.,0.};
	theta = acos(vtkMath::Dot(v3_proj,v4)) * 180 / M_PI;
	vtkMath::Cross(v3_proj, v4, axis_x);
	vtkMath::Normalize(axis_x);
	transform->RotateWXYZ(theta, axis_x);

	double vcam_Position[3];
	for (int i=0; i<3; i++) {
		world_origin[i] = world_trans(i);
		world_normal[i] = (world_quat.matrix() * Vector3d::UnitZ())(i);
		world_axisX[i] = (world_quat.matrix() * Vector3d::UnitX())(i);
		world_axisY[i] = (world_quat.matrix() * Vector3d::UnitY())(i);
		world_axisZ[i] = (world_quat.matrix() * Vector3d::UnitZ())(i);
		vcam_Position[i] = vcam_position(i);
	}

	transform->TransformPoint(world_origin, tf_world_origin);
	transform->TransformVector(world_normal, tf_world_normal);
	transform->TransformVector(world_axisX, tf_world_axisX);
	transform->TransformVector(world_axisY, tf_world_axisY);
	transform->TransformVector(world_axisZ, tf_world_axisZ);
	transform->TransformPoint(vcam_Position, tf_vcam_position);

	vcam_pixel_x = tf_vcam_position[0] + image_width  * 0.5;
	vcam_pixel_y = tf_vcam_position[1] + image_height * 0.5;

//	cout << tf_vcam_position[0] << " " << tf_vcam_position[1] << " " << tf_vcam_position[2] << endl;
//	cout << vcam_pixel_x << " " << vcam_pixel_y << endl;
}

void TableTracker::ConfigTableData()
{
	double table_RotCenter[3];
	for (int i=0;i<3;i++) {

		table_topPoint[i] = world_origin[i] + world_normal[i] * (table_height + table_topMargin);
		table_botPoint[i] = world_origin[i] + world_normal[i] * (table_height - table_botMargin);
//		table_centerPoint[i] = world_origin[i] + world_normal[i] * table_height;
		table_RotCenter[i] = table_rotCenter(i);
	}

	transform->TransformPoint(table_topPoint, tf_table_topPoint);
	transform->TransformPoint(table_botPoint, tf_table_botPoint);
//	transform->TransformPoint(table_centerPoint, tf_table_centerPoint);
	transform->TransformPoint(table_RotCenter, tf_table_rotCenter);
	transform->TransformPoint(table_position, tf_table_position);

	table_position_pixel_x  = tf_table_position[0]  * pixelPerLength + mask_width  * 0.5;
	table_position_pixel_y  = tf_table_position[1]  * pixelPerLength + mask_height * 0.5;
	table_rotCenter_pixel_x = tf_table_rotCenter[0] * pixelPerLength + mask_width  * 0.5;
	table_rotCenter_pixel_y = tf_table_rotCenter[1] * pixelPerLength + mask_height * 0.5;

	assert(0 < table_position_pixel_x && table_position_pixel_x < mask_width);
	assert(0 < table_position_pixel_y && table_position_pixel_y < mask_height);
	assert(0 < table_rotCenter_pixel_x && table_rotCenter_pixel_x < mask_width);
	assert(0 < table_rotCenter_pixel_y && table_rotCenter_pixel_y < mask_height);
}

void TableTracker::ConfigMaskData()
{
	mask = cv::Mat::zeros(mask_height, mask_width, CV_8UC1);
	mask_data = (uchar*)mask.data;
	mask_sum = table_width * pixelPerLength * table_length * pixelPerLength * 255;
	GenerateRectangleTableMask();
}

void TableTracker::SetOCR(Vector3d _ocr) {
	if (ocr != _ocr)
		isMove = true;

	ocr = _ocr;
	for (int i=0; i<3; i++) {
		tf_table_topPoint[i] += tf_world_axisX[i] * ocr[i] + tf_world_axisY[i] * ocr[i] + tf_world_axisZ[i] * ocr[i];
		tf_table_botPoint[i] += tf_world_axisX[i] * ocr[i] + tf_world_axisY[i] * ocr[i] + tf_world_axisZ[i] * ocr[i];
		tf_table_position[i] += tf_world_axisX[i] * ocr[i] + tf_world_axisY[i] * ocr[i] + tf_world_axisZ[i] * ocr[i];
	}

	if (isMove) {
		table_position_pixel_x += ocr[0] * pixelPerLength;
		table_position_pixel_y -= ocr[1] * pixelPerLength;
		GenerateRectangleTableMask();
		table_position_pixel_x -= ocr[0] * pixelPerLength;
		table_position_pixel_y += ocr[1] * pixelPerLength;
		isMove = false;
	}
}


void TableTracker::GenerateRectangleTableMask()
{
	int table_pixelWidth = table_width  * pixelPerLength;
	int table_pixelLength = table_length * pixelPerLength;

	mask_minX = table_position_pixel_x;
	mask_maxX = mask_minX + table_pixelWidth;
	mask_minY = table_position_pixel_y;
	mask_maxY = mask_minY + table_pixelLength;

	if (mask_minX < 0) mask_minX = 0;
	if (mask_minY < 0) mask_minY = 0;
	if (mask_maxX > mask_width)  mask_maxX = mask_width;
	if (mask_maxY > mask_height) mask_maxY = mask_height;

	mask = cv::Mat::zeros(mask_height, mask_width, CV_8UC1);
	for (int y = mask_minY; y < mask_maxY; y++) {
		for (int x = mask_minX; x < mask_maxX; x++) {
			mask_data[ y * mask_width + x ] = 255;
		}
	}

	mask_vec.clear();
	for (int i=mask_minRotDeg; i<=mask_maxRotDeg; i+=mask_deg) {
		cv::Mat mask_rot, rotM;
		mask.copyTo(mask_rot);
		rotM = cv::getRotationMatrix2D(cv::Point(table_rotCenter_pixel_x, table_rotCenter_pixel_y), i, 1.0);
		cv::warpAffine(mask, mask_rot, rotM, mask_rot.size());
		mask_vec.push_back(mask_rot);
	}

//	// View
//	cout << " Press 'c' to skip the present frame " << endl;
//	cout << " Press 'q' to quit " << endl;
//	int idx(0);
//	while(true)
//	{
//		if ( idx > (mask_maxRotDeg - mask_minRotDeg)/mask_deg ) idx = 0;
//		cv::circle(mask_vec[idx], cv::Point(table_rotCenter_pixel_x, table_rotCenter_pixel_y), 1.0, cv::Scalar::all(100), 3, 8, 0);
//		cv::putText(mask_vec[idx], "Resolution: "+ to_string(mask_width)+ "x" + to_string(mask_height),
//								 cv::Point(10,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
//		cv::putText(mask_vec[idx], "Table Degree: "+ to_string(mask_minRotDeg + idx * mask_deg),
//								 cv::Point(10,40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
//
//		imshow("mask_rot", mask_vec[idx++]);
//
//
//		char key = (char)cv::waitKey(1000);
//
//		if (key == 'c') continue;
//		if (key == 'q') {
//			cv::destroyAllWindows();
//			break;
//		}
//	}

}

void TableTracker::GenerateCustomTableMask()
{

}

void TableTracker::PrintConfigData()
{
	cout << "  >> Configuration data" << endl;
	cout << "     1) World Coordinate (ChArUco Board)" << endl;
	cout << "        - Origin (mm)   : " << world_trans.transpose() << endl;
	cout << "        - axis-X        : " << (world_quat.matrix() * Vector3d::UnitX()).transpose() << endl;
	cout << "        - axis-Y        : " << (world_quat.matrix() * Vector3d::UnitY()).transpose() << endl;
	cout << "        - axis-Z        : " << (world_quat.matrix() * Vector3d::UnitZ()).transpose() << endl;
	cout << endl;
	cout << "     2) Mask Data" << endl;
	cout << "        - Matching size : " << mask_width << " x " << mask_height << endl;
	cout << "        - rotation deg  : " << mask_deg << endl;
	cout << "        - rotation range: " << mask_minRotDeg << " ~ " << mask_maxRotDeg << endl;
	cout << "        - data size     : " << (mask_maxRotDeg - mask_minRotDeg + 1)/mask_deg << endl;
	cout << endl;
	cout << "     3) Virtual Camera" << endl;
	cout << "        - position (mm): " << vcam_position.transpose() << endl;
	cout << endl;
	cout << "     4) Table Data (Unit: mm)" << endl;
	cout << "        - width, length, height: " << table_width << ", " << table_length << ", " << table_height << endl;
	cout << "        - rotation center      : " << table_rotCenter.transpose() << endl;
	cout << "        - tf_rotation center   : " << tf_table_rotCenter[0] << " " << tf_table_rotCenter[1] << " " << tf_table_rotCenter[2] << endl;
//	cout << "        - layer_center         : " << table_centerPoint[0] << " " << table_centerPoint[1] << " " << table_centerPoint[2] << endl;
	cout << "        - layer_top            : " << table_topPoint[0] << " " << table_topPoint[1] << " " << table_topPoint[2] << endl;
	cout << "        - layer_bot            : " << table_botPoint[0] << " " << table_botPoint[1] << " " << table_botPoint[2] << endl;
//	cout << "        - tf_layer_center      : " << tf_table_centerPoint[0] << " " << tf_table_centerPoint[1] << " " << tf_table_centerPoint[2] << endl;
	cout << "        - tf_layer_top:        : " << tf_table_topPoint[0] << " " << tf_table_topPoint[1] << " " << tf_table_topPoint[2] << endl;
	cout << "        - tf_layer_bot:        : " << tf_table_botPoint[0] << " " << tf_table_botPoint[1] << " " << tf_table_botPoint[2] << endl;
	cout << endl;
}

// ====================


void TableTracker::FindTableCenter()
{
	colorPCD = GenerateColorTablePointCloud(point_img, color_img);
	cv::putText(colorPCD, "Press 'c' key to capture the mask",
			cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar::all(255), 2);


}


cv::Mat TableTracker::GenerateColorTablePointCloud(const k4a_image_t point_cloud_image, const k4a_image_t color_image)
{
	int16_t *point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(point_cloud_image);
	uint8_t *color_image_data = k4a_image_get_buffer(color_image);

	cv::Mat point_cloud_2d = cv::Mat::zeros(mask_height, mask_width, CV_8UC3);
	uint8_t* point_cloud_2d_data = (uint8_t*)point_cloud_2d.data;

	vector<RowVector3d> xyz;
	vector<RowVector3i> rgb;

	float tf_point[3];
	for (int i=0; i<image_width*image_height; i++)
	{
		float Z = point_cloud_image_data[ 3 * i + 2 ];
			if (Z == 0) continue;
		float X = point_cloud_image_data[ 3 * i + 0 ];
		float Y = point_cloud_image_data[ 3 * i + 1 ];

		int b = color_image_data[ 4 * i + 0 ];
		int g = color_image_data[ 4 * i + 1 ];
		int r = color_image_data[ 4 * i + 2 ];
		int alpha = color_image_data[ 4 * i + 3 ];
		if (r == 0 && g == 0 && b == 0 && alpha == 0) continue;

		float point[3] = {X,Y,Z};
		transform->TransformPoint(point, tf_point);

		xyz.push_back(RowVector3d(point[0],point[1],point[2]));
		rgb.push_back(RowVector3i(r,g,b));

		float topPlane = tf_world_normal[0] * (tf_point[0] - tf_table_topPoint[0]) +
				         tf_world_normal[1] * (tf_point[1] - tf_table_topPoint[1]) +
						 tf_world_normal[2] * (tf_point[2] - tf_table_topPoint[2]);
		if (topPlane > 0) continue;
		float botPlane = tf_world_normal[0] * (tf_point[0] - tf_table_botPoint[0]) +
						 tf_world_normal[1] * (tf_point[1] - tf_table_botPoint[1]) +
						 tf_world_normal[2] * (tf_point[2] - tf_table_botPoint[2]);
		if (botPlane < 0) continue;

		int pixel_x = tf_point[0] * pixelPerLength + mask_width  * 0.5 ;
		if (pixel_x > mask_width) continue;
		if (pixel_x < 0) continue;

		int pixel_y = tf_point[1] * pixelPerLength + mask_height * 0.5 ;
		if (pixel_y > mask_height) continue;
		if (pixel_y < 0) continue;

		point_cloud_2d_data[ 3*(mask_width*pixel_y + pixel_x) + 0] = b;
		point_cloud_2d_data[ 3*(mask_width*pixel_y + pixel_x) + 1] = g;
		point_cloud_2d_data[ 3*(mask_width*pixel_y + pixel_x) + 2] = r;
	}
	WritePointCloud(xyz, rgb);
	exit(1);


	return point_cloud_2d;
}

void TableTracker::WritePointCloud(vector<RowVector3d> xyz, vector<RowVector3i> rgb)
{
    std::ofstream ofs("tf_pcd_vcam2.ply"); // text mode first
    ofs << "ply" << std::endl;
    ofs << "format ascii 1.0" << std::endl;
    ofs << "element vertex "  << xyz.size() << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "property uchar red" << std::endl;
    ofs << "property uchar green" << std::endl;
    ofs << "property uchar blue" << std::endl;
    ofs << "end_header" << std::endl;

    for (size_t i=0;i<xyz.size();i++) {
    	ofs << xyz[i] << " " << rgb[i] << endl;
//    	ofs << vec[i] << endl;
//    	ofs << vec[i](0) << " " << vec[i](1) << " " <<vec[i](2) << " "
//    			<< vec[i](3) << " " <<  vec[i](4) << " " << vec[i](5) << endl;
    }


}

void onMouseCropImage(int event, int x, int y, int f, void *param)
{
    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN:
        clicked = true;
        P1.x = x * sfInv;
        P1.y = y * sfInv;
        P2.x = x * sfInv;
        P2.y = y * sfInv;
        break;
    case cv::EVENT_LBUTTONUP:
        P2.x = x * sfInv;
        P2.y = y * sfInv;
        clicked = false;
        break;
    case cv::EVENT_MOUSEMOVE:
        if (clicked)
        {
            P2.x = x * sfInv;
            P2.y = y * sfInv;
        }
        break;
    case cv::EVENT_RBUTTONUP:
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



