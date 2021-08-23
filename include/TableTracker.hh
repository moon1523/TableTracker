#ifndef INCLUDE_TABLETRACKER_HH_
#define INCLUDE_TABLETRACKER_HH_

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <numeric>

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
#include <vtkMatrix4x4.h>

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
	TableTracker(int tableType, Mat xy_table, Quaterniond quat, Vector3d tvec);
	virtual ~TableTracker();

	void ProcessCurrentFrame();
	void Render(double time);

	// Processing Functions
	bool FindTableCenter();
	vtkSmartPointer<vtkTransform> Transform_KinectView(float v1[3], float v2[3]);
	Mat GenerateColorTablePointCloud(const k4a_image_t point_cloud_image, const k4a_image_t color_image);
	Mat GenerateTablePointCloud(const k4a_image_t point_cloud_image, const k4a_image_t depth_image);
	void GenerateTableMask(Mat xy_table, int deg);

	tuple<double, double, Mat> MatchingData(Mat point_cloud_2d, int deg);

	// Access Functions
	void SetIsColor(bool _isColor) { isColor = _isColor; }
	void SetTableCenter(tuple<Point2i, Point2i, Point2i> data) {
		isCenter = true;
		mask1_p1 = get<0>(data);
		mask1_p2 = get<1>(data);
		tableCenter = get<2>(data);
		tableCenter_init = tableCenter;
		mask1_p1_init = mask1_p1;
		mask1_p2_init = mask1_p2;
	}
	void SetNewImage(k4a_image_t depth_image, k4a_image_t point_image) {
		depth_img = depth_image;
		point_img = point_image;
	}
	void SetColorImage(k4a_image_t color_image, k4a_image_t point_image) {
		color_img = color_image;
		point_img = point_image;
	}
	void TransMatchingMasks(Vector3d ocrDat) {
		// +x: right, -x: left, +y: up, -y: down, +z: pull, -z: push
		if (ocrData != ocrDat) isMove = true;
		ocrData = ocrDat;
		for (int i=0; i<3; i++) {
//			world_origin[i] += axisX(i) * (x-init_position(0)) + axisY(i) * (y-init_position(1)) + axisZ(i) * (z-init_position(2));
			ext_topPoint[i] += axisX(i) * (ocrData(0)-init_position(0)) + axisY(i) * (ocrData(1)-init_position(1)) + axisZ(i) * (ocrDat(2)-init_position(2));
			ext_botPoint[i] += axisX(i) * (ocrData(0)-init_position(0)) + axisY(i) * (ocrData(1)-init_position(1)) + axisZ(i) * (ocrDat(2)-init_position(2));
		}
//		tableCenter += Point2i((int)((x-init_position(0))*sf), (int)(-(y-init_position(1))*sf));
		mask1_p1 += Point2i((int)((ocrData(0)-init_position(0))*sf), (int)(-(ocrData(1)-init_position(1))*sf));
		mask1_p2 += Point2i((int)((ocrData(0)-init_position(0))*sf), (int)(-(ocrData(1)-init_position(1))*sf));

		if (isMove)
		{
			angleVec.clear();
			GenerateTableMask(xy_table, deg);
			isMove = false;
		}
	}


private:
	vector<Point2i> pixelData;
	// Mat, Image
	Mat xy_table;
	int width, height;
	k4a_image_t color_img, depth_img, point_img;
	bool isColor;

	// Table
	int pTable;
	Vector3d init_position;
	double lat_width, long_width, height_width, floor_height;
	double top_margin, bot_margin;
	float ext_topPoint[3], ext_botPoint[3];
	Point2i view_calib;
	double sf;
	vtkSmartPointer<vtkTransform> transform_mask;

	// World
	Quaterniond quat;
	Vector3d tvec;
	float world_origin[3], world_normal[3];
	vtkSmartPointer<vtkTransform> transform;

	// Matching Masks
	int deg, maxDeg;
	Mat mask;
	uchar* mask_data;
	vector<Mat> maskVec;
	tuple<double,double,Mat> results;
	int maskSum;
	bool isMove;
	Vector3d ocrData;
	vector<Point2i> maskPoints;

	// Center
	bool isCrop, isRot, isMask1, isMask2, isCenter, isFind;
	Point2i mask1_p1, mask1_p2, mask2_p1, mask2_p2;
	int transX, transY;
	Point2i tableCenter;
	Mat mask1, mask2;
	double max_sim1, max_sim2;
	Mat draw, view;
	Vector3d axisX, axisY, axisZ;
	Point2i tableCenter_init, mask1_p1_init, mask1_p2_init;

	int frameNo;
	// Accumulate angle
	vector<double> angleVec;

};




#endif /* INCLUDE_TABLETRACKER_HH_ */
