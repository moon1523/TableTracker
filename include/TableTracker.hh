#ifndef INCLUDE_TABLETRACKER_HH_
#define INCLUDE_TABLETRACKER_HH_

#include <iostream>
#include <fstream>
#include <vector>
#include <map>

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include <vtkTransform.h>
#include <vtkSmartPointer.h>
#include <vtkPlane.h>
#include <vtkMath.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

using namespace std;
using namespace cv;
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

class TableTracker {
public:
	TableTracker(Mat xy_table, Quaternionf quat, Vector3f tvec);
	virtual ~TableTracker();

	void ProcessCurrentFrame();
	void Render();

	// Processing Functions
	bool FindTableCenter();
	vtkSmartPointer<vtkTransform> Transform_KinectView(float v1[3], float v2[3]);
	Mat GenerateColorTablePointCloud(const k4a_image_t point_cloud_image, const k4a_image_t color_image);

	vector<Mat> GenerateTableMask(Mat xy_table, int deg);
	Mat GenerateBinaryTablePointCloud(const Mat depth_mat, int *point_count);
	Mat ProjectBinaryPointCloud2Image(Mat point_cloud_3d, Mat xy_table);

	tuple<double, double, Mat> MatcingData(vector<Mat> maskVec, Mat point_cloud_2d, int deg);

	// Access Functions
	void SetNewFrame(Mat& _color, Mat& _depth) {
		_color.copyTo(color_mat);
		_depth.copyTo(depth_mat);
	}
	void TransTableOrigin(float x, float y, float z) {
		// +x: right, -x: left, +y: down, -y: up, +z: push, -z: pull
		trans_table[0] = x;
		trans_table[1] = -y;
		trans_table[2] = -z;
		for (int i=0; i<3; i++) {
			world_origin[i] += trans_table[i];
		}
	}
	void SetNewImage(k4a_image_t color_image, k4a_image_t point_image) {
		color_img = color_image;
		point_img = point_image;
	}

	// Auxiliary Functions
	vector<Mat> Read_K4A_MKV_Record(string fileName, Mat xy_table);

private:
	// Mat, Image
	Mat xy_table;
	int width, height;
	Mat color_mat, depth_mat;
	k4a_image_t color_img, depth_img, point_img;

	// Table
	float lat_width, long_width, height_width, floor_height, long_pos;
	float top_margin, bot_margin;
	float ext_topPoint[3], ext_botPoint[3];
	float trans_table[3];

	// World
	Quaternionf quat;
	Vector3f tvec;
	float world_origin[3], world_normal[3];
	vtkSmartPointer<vtkTransform> transform;

	// Masks
	int deg;
	vector<Mat> maskVec;
	tuple<double,double,Mat> results;

	// Center
	bool isCrop, isRot, isMask1, isMask2, isCenter, isFind;
	Point2i mask1_p1, mask1_p2, mask2_p1, mask2_p2;
	int transX, transY;
	Point2i tableCenter;
	Mat mask1, mask2;
	float max_sim1, max_sim2;
	Mat draw, view;

	// Timer
	Timer frameTimer;
};




#endif /* INCLUDE_TABLETRACKER_HH_ */
