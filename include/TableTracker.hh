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
	TableTracker(Mat xy_table, Eigen::Vector4f quat, Eigen::Vector3f tvec);
	virtual ~TableTracker();

	void ProcessCurrentFrame();
	void Render();

	// Processing Functions
	void TableInfo();
	bool FindTableCenter();
	vtkSmartPointer<vtkTransform> Transform_Kinect_View(float v1[3], float v2[3]);
	vector<Mat> GenerateTableMask(Mat xy_table, int deg);
	Mat GenerateTablePointCloud(const Mat depth_mat, const Mat xy_table, int *point_count);
	Mat ProjectPointCloud2Image(Mat point_cloud_3d, Mat xy_table);
	tuple<double, double, Mat> MatcingData(vector<Mat> maskVec, Mat point_cloud_2d, int deg);

	// Access Functions
	void SetNewFrame(Mat& _color, Mat& _depth) {
		_color.copyTo(color_mat);
		_depth.copyTo(depth_mat);
	}
	void SetTableOrigin(float x, float y, float z);

	// Auxiliary Functions
	Mat GeneratePointCloud(const Mat depth_mat, const Mat xy_table, int *point_count);
	void WritePointCloud(string file_name, const Mat point_cloud, int point_count);
	vector<Mat> Read_K4A_MKV_Record(string fileName, Mat xy_table);

private:
	Mat xy_table;
	Eigen::Vector4f quat;
	Eigen::Vector3f tvec;
	int width, height;

	Mat color_mat, depth_mat, color_resize;
	int deg;
	vector<Mat> maskVec;
	int point_count;

	bool isCrop, isRot, isMask1, isMask2, isMove, isCenter;
	Point2i mask1_p1, mask1_p2, mask2_p1, mask2_p2;
	int transX, transY;
	Point2i tableCenter;

	tuple<double,double,Mat> results;
	float x_avg_length, y_avg_length;
	float scalingFactor;

	// Table
	float lat_width, long_width, height_width, long_pos;
	float margin_mm;
	float plane_normal[3], plane_origin[3], originT[3], plane_tfOrigin[3], plane_tfOrigin2[3];
	float floor_origin[3], floor_normal[3], trans_table[3], floor_tfOrigin[3];
	float cam_height;
	Mat mask1, mask2;
	float max_sim1, max_sim2;

	vtkSmartPointer<vtkTransform> transform;
	Timer procTimer;
};




#endif /* INCLUDE_TABLETRACKER_HH_ */
