#include "TableTracker.hh"

TableTracker::TableTracker(string _charuco_file, string _config_file, string _output_file, int _image_width, int _image_height)
: image_width(_image_width), image_height(_image_height),
  isMove(false), isSpike(false), isFirst(true), ocr(0,0,0), frameNo(0), ctime(0),
  isColorView(false), isMaskView(false), isPCDFile(false)
{
	ReadCharucoData(_charuco_file);
	ReadConfigData(_config_file);
	ConfigTransform();
	ConfigTableData();
	ConfigMaskData();
	PrintConfigData();
	if(_output_file.find(".mkv") != string::npos) {
		cout << "Don't use mkv filename extension" << endl; exit(1);
	}
	ofs.open(_output_file);
	ofs << "Time(sec)" << "\t" << "Raw_angle(deg)" << "\t" << "Filtered_angle(deg)" << endl;
}

TableTracker::~TableTracker()
{
	ofs.close();
}

// 1. Configuration
//
void TableTracker::ReadCharucoData(string charucoData)
{
	ifstream ifs(charucoData);
	if (!ifs.is_open()) {
		cerr << "Fail to read the " << charucoData << endl; exit(1);
	}
	string dump;

	ifs >> dump >> world_quat.x() >> world_quat.y() >> world_quat.z() >> world_quat.w();
	ifs >> dump >> world_trans.x() >> world_trans.y() >> world_trans.z();
	world_trans *= 10;
	ifs.close();
}

void TableTracker::ReadConfigData(string configData)
{
	cv::FileStorage fs(configData, cv::FileStorage::READ);
	if (!fs.isOpened()) {
		cerr << "Fail to read config data" << endl; exit(1);
	}
	cv::Mat cvMat_ref, cvMat_rot, cvMat_r2e;
	MatrixXd eMat_ref, eMat_rot, eMat_r2e;
	fs["Table_Reference(cm)"] >> cvMat_ref;
	fs["Table_Reference_to_Edge(cm)"] >> cvMat_r2e;
	fs["Table_RotationCenter(cm)"] >> cvMat_rot;
	fs["Table_Width(cm)"] >> table_width;
	fs["Table_Length(cm)"] >> table_length;
	fs["Table_Height_at_0(cm)"] >> table_height;
	fs["Table_TopMargin(cm)"] >> table_topMargin;
	fs["Table_BotMargin(cm)"] >> table_botMargin;
	fs["Mask_Rot(deg)"] >> mask_deg;
	fs["Mask_MaxRot(deg)"] >> mask_maxRotDeg;
	fs["Mask_MinRot(deg)"] >> mask_minRotDeg;
	fs["Crop_Width(mm)"] >> crop_width;
	fs["Crop_Height(mm)"] >> crop_height;
	fs["PixelPerLength(px/mm)"] >> pixelPerLength;
	fs.release();

	// Table edge (upper left point)
	cv::cv2eigen(cvMat_ref, eMat_ref);
	cv::cv2eigen(cvMat_r2e, eMat_r2e);
	Vector3d table_ref = world_trans +
			world_quat.matrix() * Vector3d(eMat_ref(0,0)*10, eMat_ref(0,1)*10, eMat_ref(0,2)*10);

	table_edge[0] = table_ref(0) + ( world_quat.matrix() * Vector3d(eMat_r2e(0,0)*10, eMat_r2e(0,1)*10, eMat_r2e(0,2)*10) )(0);
	table_edge[1] = table_ref(1) + ( world_quat.matrix() * Vector3d(eMat_r2e(0,0)*10, eMat_r2e(0,1)*10, eMat_r2e(0,2)*10) )(1);
	table_edge[2] = table_ref(2) + ( world_quat.matrix() * Vector3d(eMat_r2e(0,0)*10, eMat_r2e(0,1)*10, eMat_r2e(0,2)*10) )(2);

	// Table rotation center
	cv::cv2eigen(cvMat_rot, eMat_rot);
	table_rotCenter = world_trans +
			world_quat.matrix() * Vector3d(eMat_rot(0)*10, eMat_rot(1)*10, eMat_rot(2)*10);

	// Table size
	table_width *= 10; table_length *= 10; table_height *= 10;
	table_topMargin *= 10; table_botMargin *= 10;

	// Matching matrix size
	mask_width = crop_width * pixelPerLength;
	mask_height = crop_height * pixelPerLength;
}

void TableTracker::ConfigTransform()
{
	transform = vtkSmartPointer<vtkTransform>::New();
	Vector3d vcam_axisX =  world_quat.matrix() * Vector3d::UnitX();
	Vector3d vcam_axisY = -world_quat.matrix() * Vector3d::UnitY();
	Vector3d vcam_axisZ = -world_quat.matrix() * Vector3d::UnitZ();
	MatrixXd rot(3,3);

	rot << vcam_axisX(0), vcam_axisY(0), vcam_axisZ(0),
	       vcam_axisX(1), vcam_axisY(1), vcam_axisZ(1),
	       vcam_axisX(2), vcam_axisY(2), vcam_axisZ(2);
	rot = rot.inverse();
	Vector3d trans = rot * table_rotCenter;

	const double mat[16] = { rot(0,0), rot(0,1), rot(0,2), -trans(0),
							 rot(1,0), rot(1,1), rot(1,2), -trans(1),
							 rot(2,0), rot(2,1), rot(2,2), -trans(2),
							 0, 0, 0, 1};

	transform->SetMatrix(mat); // Kinect Camera View -> Virtual Camera View

	for (int i=0; i<3; i++) {
		world_origin[i] = world_trans(i);
		world_normal[i] = (world_quat.matrix() * Vector3d::UnitZ())(i);
		world_axisX[i] = (world_quat.matrix() * Vector3d::UnitX())(i);
		world_axisY[i] = (world_quat.matrix() * Vector3d::UnitY())(i);
		world_axisZ[i] = (world_quat.matrix() * Vector3d::UnitZ())(i);
	}
	transform->TransformVector(world_normal, tf_world_normal);
	transform->TransformVector(world_axisX, tf_world_axisX);
	transform->TransformVector(world_axisY, tf_world_axisY);
	transform->TransformVector(world_axisZ, tf_world_axisZ);
}

void TableTracker::ConfigTableData()
{
	double table_RotCenter[3];
	for (int i=0;i<3;i++) {

		table_topPoint[i] = world_origin[i] + world_normal[i] * (table_height + table_topMargin);
		table_botPoint[i] = world_origin[i] + world_normal[i] * (table_height - table_botMargin);
		table_RotCenter[i] = table_rotCenter(i);
	}

	transform->TransformPoint(table_topPoint, tf_table_topPoint);
	transform->TransformPoint(table_botPoint, tf_table_botPoint);
	transform->TransformPoint(table_RotCenter, tf_table_rotCenter);
	transform->TransformPoint(table_edge, tf_table_edge);

	table_edge_pixel_x  = tf_table_edge[0]  * pixelPerLength + mask_width  * 0.5;
	table_edge_pixel_y  = tf_table_edge[1]  * pixelPerLength + mask_height * 0.5;
	table_rotCenter_pixel_x = tf_table_rotCenter[0] * pixelPerLength + mask_width  * 0.5;
	table_rotCenter_pixel_y = tf_table_rotCenter[1] * pixelPerLength + mask_height * 0.5;

	assert(0 < table_edge_pixel_x && table_edge_pixel_x < mask_width);
	assert(0 < table_edge_pixel_y && table_edge_pixel_y < mask_height);
	assert(0 < table_rotCenter_pixel_x && table_rotCenter_pixel_x < mask_width);
	assert(0 < table_rotCenter_pixel_y && table_rotCenter_pixel_y < mask_height);
}

void TableTracker::ConfigMaskData()
{
	mask = cv::Mat::zeros(mask_height, mask_width, CV_8UC1);
	mask_color = cv::Mat::zeros(mask_height, mask_width, CV_8UC3);
	mask_data = (uchar*)mask.data;
	mask_color_data = (uchar*)mask_color.data;
	mask_sum = table_width * pixelPerLength * table_length * pixelPerLength * 255;
}


// 2. Tracking
//
void TableTracker::SetOCR(Vector3d _ocr) {
	if (ocr != _ocr)
		isMove = true;

	ocr = _ocr;
	for (int i=0; i<3; i++) {
		tf_table_topPoint[i] += tf_world_axisX[i] * ocr[i] + tf_world_axisY[i] * ocr[i] + tf_world_axisZ[i] * ocr[i];
		tf_table_botPoint[i] += tf_world_axisX[i] * ocr[i] + tf_world_axisY[i] * ocr[i] + tf_world_axisZ[i] * ocr[i];
		tf_table_edge[i] += tf_world_axisX[i] * ocr[i] + tf_world_axisY[i] * ocr[i] + tf_world_axisZ[i] * ocr[i];
	}

	if (isMove || isFirst) {
		table_edge_pixel_x += ocr[0] * pixelPerLength;
		table_edge_pixel_y -= ocr[1] * pixelPerLength;
		GenerateRectangleTableMask();
		table_edge_pixel_x -= ocr[0] * pixelPerLength;
		table_edge_pixel_y += ocr[1] * pixelPerLength;
	}
}

void TableTracker::GenerateRectangleTableMask()
{
	int table_pixelWidth = table_width  * pixelPerLength;
	int table_pixelLength = table_length * pixelPerLength;

	mask_minX = table_edge_pixel_x;
	mask_maxX = mask_minX + table_pixelWidth;
	mask_minY = table_edge_pixel_y;
	mask_maxY = mask_minY + table_pixelLength;

	if (mask_minX < 0) mask_minX = 0;
	if (mask_minY < 0) mask_minY = 0;
	if (mask_maxX > mask_width)  mask_maxX = mask_width;
	if (mask_maxY > mask_height) mask_maxY = mask_height;

	mask = cv::Mat::zeros(mask_height, mask_width, CV_8UC1);
	if(isColorView && isMaskView) mask_color = cv::Mat::zeros(mask_height, mask_width, CV_8UC3);
	for (int y = mask_minY; y < mask_maxY; y++) {
		for (int x = mask_minX; x < mask_maxX; x++) {

//			mask_data[ y * mask_width + x ] = 255;

			// quadratic weight
			mask_data[ y * mask_width + x ] = 255
					//* ( (mask_maxX-x)/(double)table_pixelWidth  )// * ( (mask_maxX-x)/(double)table_pixelWidth  );
					* ( (y-mask_minY)/(double)table_pixelLength ) * ( (y-mask_minY)/(double)table_pixelLength );

			if(isColorView && isMaskView) {
				mask_color_data[ (y * mask_width + x) * 3 + 1 ] = 255;
				mask_color_data[ (y * mask_width + x) * 3 + 2 ] = 255;
			}

		}
	}

	mask_vec.clear();
	if(isColorView && isMaskView) {
		mask_color_vec.clear();
	}
	for (int i=mask_minRotDeg; i<=mask_maxRotDeg; i+=mask_deg) {
		cv::Mat mask_rot, rotM, mask_color_rot;
		mask.copyTo(mask_rot);
		rotM = cv::getRotationMatrix2D(cv::Point(table_rotCenter_pixel_x, table_rotCenter_pixel_y), i, 1.0);
		cv::warpAffine(mask, mask_rot, rotM, mask_rot.size());
		cv::warpAffine(mask_color, mask_color_rot, rotM, mask_color_rot.size());
		mask_vec.push_back(mask_rot);
		if(isColorView && isMaskView) {
			mask_color_vec.push_back(mask_color_rot);
		}
	}

	if (isMaskView && isFirst) {
		cout << " Press 'c' to skip the present frame " << endl;
		cout << " Press 'q' to quit " << endl;
		int idx(0);
		while(true)
		{
			if ( idx > (mask_maxRotDeg - mask_minRotDeg)/mask_deg ) idx = 0;
			cv::circle(mask_vec[idx], cv::Point(table_rotCenter_pixel_x, table_rotCenter_pixel_y), 1.0, cv::Scalar::all(100), 3, 8, 0);
			cv::putText(mask_vec[idx], "Resolution: "+ to_string(mask_width)+ "x" + to_string(mask_height),
									 cv::Point(10,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
			cv::putText(mask_vec[idx], "Table Degree: "+ to_string(mask_minRotDeg + idx * mask_deg),
									 cv::Point(10,40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);

			imshow("Matching Masks", mask_vec[idx++]);


			char key = (char)cv::waitKey(1000);

			if (key == 'c') continue;
			if (key == 'q') {
				cv::destroyAllWindows();
				break;
			}
		}
	}

}

void TableTracker::PrintConfigData()
{
	cout << "  >> Configuration data" << endl;
	cout << "     1) World Coordinate (ChArUco Board)" << endl;
	cout << "        - Origin (mm)   : " << world_trans.transpose() << endl;
	cout << "        - axis-X        : " << world_axisX[0] << " " << world_axisX[1] << " " << world_axisX[2] << endl;
	cout << "        - axis-Y        : " << world_axisY[0] << " " << world_axisY[1] << " " << world_axisY[2] << endl;
	cout << "        - axis-Z        : " << world_axisZ[0] << " " << world_axisZ[1] << " " << world_axisZ[2] << endl;
	cout << endl;
	cout << "     2) Mask Data" << endl;
	cout << "        - Matching size : " << mask_width << " x " << mask_height << endl;
	cout << "        - rotation deg  : " << mask_deg << endl;
	cout << "        - rotation range: " << mask_minRotDeg << " ~ " << mask_maxRotDeg << endl;
	cout << "        - data size     : " << (mask_maxRotDeg - mask_minRotDeg + 1)/mask_deg << endl;
	cout << endl;
	cout << "     3) Table Data (Unit: mm)" << endl;
	cout << "        - left upper edge point: " << table_edge[0] << " " << table_edge[1] << " " << table_edge[2] << endl;
	cout << "        - width/length/height0 : " << table_width << "/" << table_length << "/" << table_height << endl;
	cout << "        - rotation center      : " << table_rotCenter.transpose() << endl;
	cout << "        - tf_rotation center   : " << tf_table_rotCenter[0] << " " << tf_table_rotCenter[1] << " " << tf_table_rotCenter[2] << endl;
	cout << "        - layer_top            : " << table_topPoint[0] << " " << table_topPoint[1] << " " << table_topPoint[2] << endl;
	cout << "        - layer_bot            : " << table_botPoint[0] << " " << table_botPoint[1] << " " << table_botPoint[2] << endl;
	cout << "        - tf_layer_top:        : " << tf_table_topPoint[0] << " " << tf_table_topPoint[1] << " " << tf_table_topPoint[2] << endl;
	cout << "        - tf_layer_bot:        : " << tf_table_botPoint[0] << " " << tf_table_botPoint[1] << " " << tf_table_botPoint[2] << endl;
	cout << endl;
}


cv::Mat TableTracker::GenerateTablePointCloud(const k4a_image_t point_cloud_image, const k4a_image_t depth_image)
{
	int16_t *point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(point_cloud_image);
	uint8_t *depth_image_data = k4a_image_get_buffer(depth_image);

	cv::Mat point_cloud_2d = cv::Mat::zeros(mask_height, mask_width, CV_8UC1);
	uint8_t* point_cloud_2d_data = (uint8_t*)point_cloud_2d.data;

	vector<RowVector3d> xyz, tf_xyz;
	vector<RowVector3i> rgb;

	float tf_point[3];
	for (int i=0; i<image_width*image_height; i++)
	{
		float Z = point_cloud_image_data[ 3 * i + 2 ];
			if (Z == 0) continue;
		float X = point_cloud_image_data[ 3 * i + 0 ];
		float Y = point_cloud_image_data[ 3 * i + 1 ];

		float point[3] = {X,Y,Z};
		transform->TransformPoint(point, tf_point);

		if (isPCDFile)
		{
			xyz.push_back(RowVector3d(point[0],point[1],point[2]));
			tf_xyz.push_back(RowVector3d(tf_point[0],tf_point[1],tf_point[2]));
		}

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

	if (isPCDFile) WritePointCloud(xyz, tf_xyz, rgb);

	return point_cloud_2d;
}

void TableTracker::ProcessCurrentFrame()
{
	if(isColorView) colorPCD = GenerateColorTablePointCloud(point_img, color_img);
	binPCD = GenerateTablePointCloud(point_img, depth_img);

	match_results_raw = MatchingData();
	match_results_filtered = match_results_raw;
	match_results = match_results_raw;

	if (isFirst) {
		match_results_prev = match_results;
		isFirst = false;
	}

	// Spikes Detection: Threshold
	//
	if ((fabs(get<0>(match_results_prev) - get<0>(match_results)) > 10.)) {
		isSpike = true;
		cout << "[Warning] Spike is detected (frame " << frameNo << ")" << endl;
		if (frameNo - frameNo_prev == 1) {
			spike_vec.push_back(get<0>(match_results_raw));
		}
		frameNo_prev = frameNo;

		get<0>(match_results) = get<0>(match_results_prev);
		get<1>(match_results) = get<1>(match_results_prev);
		cv::Mat match_xor;
		cv::bitwise_xor(mask_vec[(get<0>(match_results) - mask_minRotDeg)/mask_deg], binPCD, match_xor);
		get<2>(match_results) = match_xor;

		if (spike_vec.size() == 10) {
			isSpike = false;
			spike_vec.clear();
			cout << "[ Alert ] Spikes persists. Reset data" << endl;
			get<0>(match_results) = get<0>(match_results_raw);
			get<1>(match_results) = get<1>(match_results_raw);
		}
	}

	// Jittering Correction: Moving Average
	//
	match_angleList.push_back(get<0>(match_results_raw));
	if (match_angleList.size() == 30)
	{
		if (isSpike) {
			match_angleList.pop_back();
			match_angleList.push_back(get<0>(match_results));
		}

		double mean = accumulate(match_angleList.begin(), match_angleList.end(), 0LL) / (double)match_angleList.size();

		get<0>(match_results_filtered) = mean;
		if (isMaskView) {
			cv::Mat match_xor, rotM, mask_rot;
			rotM = cv::getRotationMatrix2D(cv::Point(table_rotCenter_pixel_x, table_rotCenter_pixel_y), mean, 1.0);
			cv::warpAffine(mask, mask_rot, rotM, mask_rot.size());
			cv::bitwise_xor(mask_rot, binPCD, match_xor);
			get<2>(match_results_filtered) = match_xor;
		}
		match_angleList.pop_front();
	}
	match_results_prev = match_results;

	// Post-processing
	for (int i=0; i<3; i++) {
		tf_table_topPoint[i] -= tf_world_axisX[i] * ocr[i] + tf_world_axisY[i] * ocr[i] + tf_world_axisZ[i] * ocr[i];
		tf_table_botPoint[i] -= tf_world_axisX[i] * ocr[i] + tf_world_axisY[i] * ocr[i] + tf_world_axisZ[i] * ocr[i];
		tf_table_edge[i] -= tf_world_axisX[i] * ocr[i] + tf_world_axisY[i] * ocr[i] + tf_world_axisZ[i] * ocr[i];
	}
}



void TableTracker::Render(double time)
{
	cv::Mat match_mat, match_mat_color;
	if (isMaskView) {
		match_mat = get<2>(match_results_filtered);
		match_mat_color = match_xor_color;
	}
	else {
		match_mat = binPCD;
		match_mat_color = colorPCD;
	}

	cv::putText(match_mat, "Frame #: " + to_string(frameNo),
			cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 1.5);
	cv::putText(match_mat, "Similarity: " + to_string(get<1>(match_results_raw)),
			cv::Point(10,mask_height-80), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 1.5);
	cv::putText(match_mat, "Table Degree (Filtered): " + to_string(get<0>(match_results_filtered)),
			cv::Point(10,mask_height-60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 1.5);
	cv::putText(match_mat, "Table Degree (Raw): " + to_string(get<0>(match_results_raw)),
			cv::Point(10,mask_height-40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 1.5);
	cv::putText(match_mat, "Frame Time (s): " + to_string(time),
			cv::Point(10,mask_height-20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 1.5);


	cv::Mat match_mat_cvt;
	cv::cvtColor(match_mat, match_mat_cvt, cv::COLOR_GRAY2BGR);
	cv::circle(match_mat_cvt, cv::Point(table_rotCenter_pixel_x,table_rotCenter_pixel_y), 2.0,cv::Scalar(0,0,255), 3, 8, 0);
	cv::imshow("TableTracker", match_mat_cvt);

	if(isColorView) {
		cv::Mat color(image_height, image_width, CV_8UC3, cv::Scalar(0,0,0));
		uchar* color_data = color.data;
		uint8_t *color_image_data = k4a_image_get_buffer(color_img);

		for (int i=0;i<image_width*image_height; i++) {
			color_data[ 3 * i + 0 ] = color_image_data[ 4 * i + 0 ];
			color_data[ 3 * i + 1 ] = color_image_data[ 4 * i + 1 ];
			color_data[ 3 * i + 2 ] = color_image_data[ 4 * i + 2 ];
		}
		cv::putText(match_mat_color, "Matching mask (raw)",
				cv::Point(10,mask_height-20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar::all(255), 1.5);
		cv::circle(match_mat_color, cv::Point(table_rotCenter_pixel_x,table_rotCenter_pixel_y), 2.0,cv::Scalar(0,0,255), 3, 8, 0);

		cv::imshow("Kinect_Color", color);
		cv::imshow("TableTracker_Color", match_mat_color);
	}


	// Print Results
	ctime += time;
	ofs << ctime << "\t" << get<0>(match_results_raw) << "\t" << get<0>(match_results_filtered) << endl;

	isMove = false;
	isSpike = false;
	frameNo++;
}

tuple<double, double, cv::Mat> TableTracker::MatchingData()
{
	cv::Mat match_xor, match_and, match_mat;
	double max_sim(0);
	double match_deg(0);
	int idx(0);

	cv::Mat pcd_color;
	cv::Mat mask_color, mask_hsv;

	for (int i=mask_minRotDeg; i<=mask_maxRotDeg; i+=mask_deg)
	{
		// Matching masks and point cloud data
		cv::bitwise_and(mask_vec[idx], binPCD, match_and);
		double similarity = cv::sum(match_and).val[0] / mask_sum;

		// Find the most similar mask
		if (max_sim < similarity) {
			// bitwise xor is used to view, not affect the mathcing results
			cv::bitwise_xor(mask_vec[idx], binPCD, match_xor);
			if(isColorView && isMaskView){
				cv::bitwise_xor(mask_color_vec[idx], colorPCD, match_xor_color);
			}

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



// 3. Option
//

cv::Mat TableTracker::GenerateColorTablePointCloud(const k4a_image_t point_cloud_image, const k4a_image_t color_image)
{
	int16_t *point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(point_cloud_image);
	uint8_t *color_image_data = k4a_image_get_buffer(color_image);

	cv::Mat point_cloud_2d = cv::Mat::zeros(mask_height, mask_width, CV_8UC3);
	uint8_t* point_cloud_2d_data = (uint8_t*)point_cloud_2d.data;

	vector<RowVector3d> xyz, tf_xyz;
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

		if (isPCDFile)
		{
			xyz.push_back(RowVector3d(point[0],point[1],point[2]));
			tf_xyz.push_back(RowVector3d(tf_point[0],tf_point[1],tf_point[2]));
			rgb.push_back(RowVector3i(r,g,b));
		}


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

	if (isPCDFile) WritePointCloud(xyz, tf_xyz, rgb);

	return point_cloud_2d;
}

void TableTracker::WritePointCloud(vector<RowVector3d> xyz, vector<RowVector3d> tf_xyz, vector<RowVector3i> rgb)
{
    ofstream ofs( to_string(frameNo) + ".ply" );
    ofs << "ply" << endl;
    ofs << "format ascii 1.0" << endl;
    ofs << "element vertex "  << xyz.size() << endl;
    ofs << "property float x" << endl;
    ofs << "property float y" << endl;
    ofs << "property float z" << endl;
    if (isColorView) {
    	ofs << "property uchar red" << endl;
		ofs << "property uchar green" << endl;
		ofs << "property uchar blue" << endl;
		ofs << "end_header" << endl;
		for (size_t i=0;i<xyz.size();i++) {
			ofs << xyz[i] << " " << rgb[i] << endl;
		}
    }
    else {
    	ofs << "end_header" << endl;
    	for (size_t i=0;i<xyz.size();i++) {
			ofs << xyz[i] << endl;
		}
    } ofs.close();

    ofstream ofs_tf( to_string(frameNo) + "_tf.ply" );
	ofs_tf << "ply" << endl;
	ofs_tf << "format ascii 1.0" << endl;
	ofs_tf << "element vertex "  << tf_xyz.size() << endl;
	ofs_tf << "property float x" << endl;
	ofs_tf << "property float y" << endl;
	ofs_tf << "property float z" << endl;
	if (isColorView) {
		ofs_tf << "property uchar red" << endl;
		ofs_tf << "property uchar green" << endl;
		ofs_tf << "property uchar blue" << endl;
		ofs_tf << "end_header" << endl;

		for (size_t i=0;i<tf_xyz.size();i++) {
			ofs_tf << tf_xyz[i] << " " << rgb[i] << endl;
		}
	}
	else {
		ofs_tf << "end_header" << endl;
		for (size_t i=0;i<tf_xyz.size();i++) {
			ofs_tf << tf_xyz[i] << endl;
		}
	}
	ofs_tf.close();
	isPCDFile = false;
}

void TableTracker::IncreaseTopMargin() {
	table_topMargin += 10;
	cout << "top margin (cm): " << table_topMargin*0.1 << endl;
	for (int i=0;i<3;i++)
		table_topPoint[i] = world_origin[i] + world_normal[i] * (table_height + table_topMargin);
	transform->TransformPoint(table_topPoint, tf_table_topPoint);
}
void TableTracker::DecreaseTopMargin() {
	table_topMargin -= 10;
	if (table_topMargin < 0) table_topMargin = 0;
	cout << "top margin (cm): " << table_topMargin*0.1 << endl;
	for (int i=0;i<3;i++)
		table_topPoint[i] = world_origin[i] + world_normal[i] * (table_height + table_topMargin);
	transform->TransformPoint(table_topPoint, tf_table_topPoint);
}
void TableTracker::IncreaseBotMargin() {
	table_botMargin += 10;
	cout << "bot margin (cm): " << table_botMargin*0.1 << endl;
	for (int i=0;i<3;i++)
		table_botPoint[i] = world_origin[i] + world_normal[i] * (table_height - table_botMargin);
	transform->TransformPoint(table_botPoint, tf_table_botPoint);
}
void TableTracker::DecreaseBotMargin() {
	table_botMargin -= 10;
	if (table_botMargin < 0) table_botMargin = 0;
	cout << "bot margin (cm): " << table_botMargin*0.1 << endl;
	for (int i=0;i<3;i++)
		table_botPoint[i] = world_origin[i] + world_normal[i] * (table_height - table_botMargin);
	transform->TransformPoint(table_botPoint, tf_table_botPoint);
}


