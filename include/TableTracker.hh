#ifndef TABLETRACKER_HH_
#define TABLETRACKER_HH_

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <list>
#include <numeric>
#include <algorithm>

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <vtkTransform.h>
#include <vtkSmartPointer.h>
#include <vtkPlane.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>



using namespace std;
using namespace Eigen;

class Timer
{
public:
    Timer() : start_(0), time_(0) {}

    void start() {
    	start_ = cv::getTickCount();
    }
    void stop() {
        CV_Assert(start_ != 0);
        int64 end = cv::getTickCount();
        time_ += end - start_;
        start_ = 0;
    }

    double time() {
        double ret = time_ / cv::getTickFrequency();
        time_ = 0;
        return ret;
    }

private:
    int64 start_, time_;
};

enum class PatientTable
{
	TestDevice,
	AlluraXper,
	ArtisZeego,
	ArtisQ,
	Azurion7,
	InnovaIGS630
};

class TableTracker {
public:
	TableTracker(string config_file, int image_width, int image_height);
	~TableTracker();

	void ColorView(bool _isColorView) { isColorView = _isColorView; }
	void MaskView(bool _isMaskView) { isMaskView = _isMaskView; }

	void SetNewImage(k4a_image_t depth_image, k4a_image_t point_image) {
		depth_img = depth_image;
		point_img = point_image;
	}
	void SetNewImage(k4a_image_t color_image, k4a_image_t depth_image, k4a_image_t point_image) {
		color_img = color_image;
		depth_img = depth_image;
		point_img = point_image;
	}
	void SetOCR(Vector3d _ocr);
	void IncreaseTopMargin() {
		table_topMargin += 10;
		cout << "top margin (cm): " << table_topMargin*0.1 << endl;
		for (int i=0;i<3;i++)
			table_topPoint[i] = world_origin[i] + world_normal[i] * (table_height + table_topMargin);
		transform->TransformPoint(table_topPoint, tf_table_topPoint);
	}
	void DecreaseTopMargin() {
		table_topMargin -= 10;
		if (table_topMargin < 0) table_topMargin = 0;
		cout << "top margin (cm): " << table_topMargin*0.1 << endl;
		for (int i=0;i<3;i++)
			table_topPoint[i] = world_origin[i] + world_normal[i] * (table_height + table_topMargin);
		transform->TransformPoint(table_topPoint, tf_table_topPoint);
	}
	void IncreaseBotMargin() {
		table_botMargin += 10;
		cout << "bot margin (cm): " << table_botMargin*0.1 << endl;
		for (int i=0;i<3;i++)
			table_botPoint[i] = world_origin[i] + world_normal[i] * (table_height - table_botMargin);
		transform->TransformPoint(table_botPoint, tf_table_botPoint);
	}
	void DecreaseBotMargin() {
		table_botMargin -= 10;
		if (table_botMargin < 0) table_botMargin = 0;
		cout << "bot margin (cm): " << table_botMargin*0.1 << endl;
		for (int i=0;i<3;i++)
			table_botPoint[i] = world_origin[i] + world_normal[i] * (table_height - table_botMargin);
		transform->TransformPoint(table_botPoint, tf_table_botPoint);
	}

	// Configuration
	void ReadConfigData(string fileName);
	void ConfigVirtualCamera();
	void ConfigTableData();
	void ConfigMaskData();
	void PrintConfigData();

	// Mask
	void GenerateRectangleTableMask();

	// Tracking
	cv::Mat GenerateTablePointCloud(const k4a_image_t point_cloud_image, const k4a_image_t depth_image);
	void ProcessCurrentFrame();
	void Render(double time);
	tuple<double, double, cv::Mat> MatchingData();


	// Auxiliary Functions
	void GeneratePointCloudFile() {
		cout << "Generate Point Cloud PLY file: " << frameNo << ".ply/tf_ply" << endl;
		isPCDFile = true;
	}
	cv::Mat GenerateColorTablePointCloud(const k4a_image_t point_cloud_image, const k4a_image_t color_image);
	void WritePointCloud(vector<RowVector3d> xyz, vector<RowVector3d> tf_xyz, vector<RowVector3i> rgb);


private:
	// View
	bool isColorView, isMaskView, isPCDFile;

	// Image data
	k4a_image_t color_img, depth_img, point_img;
	double pixelPerLength;
	int frameNo;
	cv::Mat binPCD, colorPCD;
	int image_width, image_height;

	// Mask
	int mask_width, mask_height;
	int mask_deg, mask_maxRotDeg, mask_minRotDeg;
	cv::Mat mask, mask_color;
	uchar* mask_data;
	uchar* mask_color_data;
	int mask_sum;
	vector<cv::Mat> mask_vec, mask_color_vec;
	int mask_minX, mask_maxX, mask_minY, mask_maxY;

	// Virtual Camera
	vtkSmartPointer<vtkTransform> transform;
	Vector3d vcam_position;
	int vcam_pixel_x, vcam_pixel_y;

	// World
	Quaterniond world_quat;
	Vector3d world_trans;
	double world_origin[3], world_normal[3];
	double world_axisX[3], world_axisY[3], world_axisZ[3];

	// Table
	Vector3d table_rotCenter;
	double table_position[3];
	double table_width, table_length, table_height;
	double table_topPoint[3], table_botPoint[3];
	double table_topMargin, table_botMargin;

	int table_position_pixel_x, table_position_pixel_y;
	int table_rotCenter_pixel_x, table_rotCenter_pixel_y;

	// Transformation
	double tf_vcam_position[3];
	double tf_world_origin[3], tf_world_normal[3];
	double tf_world_axisX[3], tf_world_axisY[3], tf_world_axisZ[3];
	double tf_table_topPoint[3], tf_table_botPoint[3];
	double tf_table_rotCenter[3];
	double tf_table_position[3];

	// Results
	tuple<double, double, cv::Mat> match_results_raw, match_results_raw_prev;
	tuple<double, double, cv::Mat> match_results, match_results_prev;
	tuple<double, double, cv::Mat> match_results_filtered;


	list<double> match_angleList;
	ofstream ofs;

	// OCR
	Vector3d ocr;
	bool isMove, isRot, isSpike, isFirst, isFix;
	int spike_chk;
	vector<double> spike_vec;
	int frameNo_prev;
	double mean;
	cv::Mat match_xor_color;


};


#endif
