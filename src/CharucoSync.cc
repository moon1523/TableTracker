#include "CharucoSync.hh"

Rect cropRect;
bool clicked;
Point2i P1, P2;
float sfInv;
void onMouseCropImage(int event, int x, int y, int f, void *param);

Eigen::Vector4d quaternionAverage(std::vector<Eigen::Vector4d> quaternions);

CharucoSync::CharucoSync(int tableType)
    :pTable(tableType), getPose(false), sf(1), frameNo(0), isStack(false)
{
}

CharucoSync::~CharucoSync()
{
}

bool CharucoSync::SetParameters(string camParm, string detParam)
{
    cv::FileStorage fs(camParm, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    switch((PatientTable)pTable)
	{
	case PatientTable::TestDevice:
//		board = cv::aruco::CharucoBoard::create(5, 7, 0.0300f, 0.0235f, dictionary);
		board = cv::aruco::CharucoBoard::create(5, 7, 0.0290f, 0.0230f, dictionary);
		break;
	case PatientTable::AlluraXper:
		board = cv::aruco::CharucoBoard::create(5, 7, 0.0765f, 0.0535f, dictionary);
		break;
	}

    params = cv::aruco::DetectorParameters::create();

    FileStorage fs2(detParam, FileStorage::READ);
    if (!fs2.isOpened())
        return false;
    fs2["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs2["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs2["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs2["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs2["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs2["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs2["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs2["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs2["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs2["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs2["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs2["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs2["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs2["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs2["markerBorderBits"] >> params->markerBorderBits;
    fs2["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs2["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs2["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs2["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs2["errorCorrectionRate"] >> params->errorCorrectionRate;

    return true;
}

void CharucoSync::EstimatePose(const Mat &color, Vec3d &rvec, Vec3d &tvec)
{
    color.copyTo(display);

    Mat cropImg;
    if (cropRect.width > 0 && cropRect.height > 0)
        cropImg = color(cropRect).clone();
    else
        color.copyTo(cropImg);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::aruco::detectMarkers(cropImg, board->dictionary, markerCorners, markerIds, params);
    // if at least one marker detected
    if (markerIds.size() > 0)
    {
        std::for_each(markerCorners.begin(), markerCorners.end(), [](vector<Point2f> &vec)
                      {
                          for (Point2f &point : vec)
                              point += Point2f(cropRect.tl());
                      });
        cv::aruco::drawDetectedMarkers(display, markerCorners, markerIds);
        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;
        cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, color, board, charucoCorners, charucoIds, camMatrix, distCoeffs);
        // if at least one charuco corner detected
        if (charucoIds.size() > 0)
        {
            cv::aruco::drawDetectedCornersCharuco(display, charucoCorners, charucoIds);
            bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, camMatrix, distCoeffs, rvec, tvec);
            // if charuco pose is valid
            if (valid) {
                cv::aruco::drawAxis(display, camMatrix, distCoeffs, rvec, tvec, 0.1f);
//				cout << rvec << " " << tvec << endl;
            }

            if (getPose)
            {
                double angle = norm(rvec);
                Vector3d axis(rvec(0) / angle, rvec(1) / angle, rvec(2) / angle);
                Quaterniond q(AngleAxisd(angle, axis)); q.normalize();

                if (isnan(q.x()))
				{
					cout << "Filtering the frame: Coordinate is not correctly estimated" << endl;
				}
                else if (frameNo < 10)
                {
                	tenQuats.push_back(Vector4d(q.x(), q.y(), q.z(), q.w()));
                	cout << frameNo++ << ": " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
                }
                else if (frameNo == 10)
                {
                	avg10_quat = quaternionAverage(tenQuats);
                	frameNo++;
                }
                else
                {
                	Quaterniond quat0 = Quaterniond(avg10_quat.w(), avg10_quat.x(), avg10_quat.y(), avg10_quat.z());
                	Quaterniond quat1 = Quaterniond(q.w(), q.x(), q.y(), q.z());

                	Vector3d axisX0 = quat0.matrix() * Vector3d::UnitX();
					Vector3d axisY0 = quat0.matrix() * Vector3d::UnitY();
					Vector3d axisZ0 = quat0.matrix() * Vector3d::UnitZ();
					Vector3d axisX1 = quat1.matrix() * Vector3d::UnitX();
                    Vector3d axisY1 = quat1.matrix() * Vector3d::UnitY();
                    Vector3d axisZ1 = quat1.matrix() * Vector3d::UnitZ();

                    double dotX = axisX0.dot(axisX1);
					double dotY = axisY0.dot(axisY1);
					double dotZ = axisZ0.dot(axisZ1);

                    if (dotX > 0 && dotY > 0 && dotZ > 0) {
                    	frameNo++;
						isStack = true;
                    }
                }

                if (isStack)
				{
					quaternions.push_back(Vector4d(q.x(), q.y(), q.z(), q.w()));
					tvec_sum += tvec;
					isStack = false;
				}


            } // getPose
        }
    }
}

void CharucoSync::Render()
{
    setMouseCallback("Synchronization", onMouseCropImage);
    if (clicked)
    	cv::rectangle(display, P1, P2, CV_RGB(255, 255, 0), 3);
    else if (cropRect.width > 0){
        imshow("crop img.", display(cropRect));
        cv::rectangle(display, cropRect, CV_RGB(255, 255, 0), 3);
    }

    resize(display, display, Size(display.cols * sf, display.rows * sf));

    if (frameNo>10)
    	putText(display, "number of data: " + to_string(frameNo-10) , Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255.f,0.f,0.f), 1.2);
    if(getPose) 
        putText(display, "obtaining pose data..", Point(10, 40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255.f,0.f,0.f), 1);
    imshow("Synchronization", display);
}

#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
void CharucoSync::ShowAvgValue(const Mat &color)
{
    AngleAxisd avg(Quaterniond((quaternionAverage(quaternions)))); 

    Vec3d rvec;
    eigen2cv(avg.axis(), rvec);
    rvec *= avg.angle();
    Vec3d tvec_avg = tvec_sum / (double)quaternions.size();

    color.copyTo(display);
    cv::aruco::drawAxis(display, camMatrix, distCoeffs, rvec, tvec_avg, 0.1f);
    resize(display, display, Size(display.cols * sf, display.rows * sf));
    imshow("Synchronization", display);
}

void CharucoSync::WriteTransformationData(string fileName){
	cout << fileName << " is generated." << endl;
    ofstream ofs(fileName);
    Vector4d q = quaternionAverage(quaternions);
    Vec3d t = tvec_sum / (double)quaternions.size();
    ofs<<"q "<< q(0) <<", "<<q(1)<<", "<<q(2)<<", "<<q(3)<<endl;
    ofs<<"t "<<t(0)*100<<", "<<t(1)*100<<", "<<t(2)*100<<endl;
    time_t now = chrono::system_clock::to_time_t(chrono::system_clock::now());
    ofs<<ctime(&now)<<endl;
    ofs<<endl;
    ofs.close();
}

void CharucoSync::SetScalingFactor(float s)
{
    sf = s;
    sfInv = 1 / s;
}

Eigen::Vector4d quaternionAverage(std::vector<Eigen::Vector4d> quaternions)
{
    if (quaternions.size() == 0)
    {
        std::cerr << "Error trying to calculate the average quaternion of an empty set!\n";
        return Eigen::Vector4d::Zero();
    }

    // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

    for (int q = 0; q < quaternions.size(); ++q)
        A += quaternions[q] * quaternions[q].transpose();

    // normalise with the number of quaternions
    A /= quaternions.size();

    // Compute the SVD of this 4x4 matrix
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::MatrixXd U = svd.matrixU();

    // find the eigen vector corresponding to the largest eigen value
    int largestEigenValueIndex;
    float largestEigenValue;
    bool first = true;

    for (int i = 0; i < singularValues.rows(); ++i)
    {
        if (first)
        {
            largestEigenValue = singularValues(i);
            largestEigenValueIndex = i;
            first = false;
        }
        else if (singularValues(i) > largestEigenValue)
        {
            largestEigenValue = singularValues(i);
            largestEigenValueIndex = i;
        }
    }

    Eigen::Vector4d average;
    average(0) = -U(0, largestEigenValueIndex);
    average(1) = -U(1, largestEigenValueIndex);
    average(2) = -U(2, largestEigenValueIndex);
    average(3) = -U(3, largestEigenValueIndex);

    return average;
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



