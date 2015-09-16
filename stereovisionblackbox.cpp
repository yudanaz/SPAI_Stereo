#include "stereovisionblackbox.h"

StereoVisionBlackBox::StereoVisionBlackBox(int rows, int cols)
{
	this->cols = cols;
	this->rows = rows;
	this->imgSz.width = cols;
	this->imgSz.height = rows;
}

bool StereoVisionBlackBox::addCalibrationImages(vector<Mat> leftImgs, vector<Mat> rightImgs, Size chessboardSize)
{
	//check if same amount of left and right images, and if correct image size
	if(leftImgs.size() != rightImgs.size()){ return false; }
	for (uint i = 0; i < leftImgs.size(); ++i)
	{
		if( ((Mat)leftImgs[i]).cols != this->cols  || ((Mat)leftImgs[i]).rows != this->rows ||
			((Mat)rightImgs[i]).cols != this->cols || ((Mat)rightImgs[i]).rows != this->rows    )
		{ return false; }
	}

	//if ok, add to calib vectors
	this->calibImgsLeft.insert(calibImgsLeft.end(), leftImgs.begin(), leftImgs.end());
	this->calibImgsRight.insert(calibImgsRight.end(), rightImgs.begin(), rightImgs.end());
	this->chessboardSz = chessboardSize;
	return true;
}

bool StereoVisionBlackBox::addCalibrationImagePair(Mat leftImg, Mat rightImg, Size chessboardSize)
{
	if(leftImg.cols != this->cols || leftImg.rows != this->rows || rightImg.cols != this->cols || rightImg.rows != this->rows)
	{ return false; }
	this->calibImgsLeft.push_back(leftImg);
	this->calibImgsRight.push_back(rightImg);
	this->chessboardSz = chessboardSize;
	return true;
}

void StereoVisionBlackBox::resetCalibrationImages()
{
	this->calibImgsLeft.clear(); this->calibImgsRight.clear();
}

bool StereoVisionBlackBox::calibrateCameras(bool crop2commonArea)
{
	if(this->calibImgsLeft.size() < 8){ return false; }

	/**************************************************************************
	 * get chessboard corners for all calibration images
	 *************************************************************************/
	vector<vector<Point3f> > objPoints;
	vector<vector<Point2f> > imgPoints1, imgPoints2;

	//object points should contain physical location of each corners but
	//since we don’t know these locations, we assign constant positions to all the corners
	vector<Point3f> obj;
	for (int i = 0; i < chessboardSz.width*chessboardSz.height; i++)
	{
		obj.push_back(Point3f(i / chessboardSz.width, i % chessboardSz.width, 0.0f));
	}

	//get image and objects points
	for (uint i = 0; i < calibImgsLeft.size(); ++i)
	{
		vector<Point2f> corners1, corners2;
		bool success1 = findChessboardCorners((Mat)calibImgsLeft[i], chessboardSz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		bool success2 = findChessboardCorners((Mat)calibImgsRight[i], chessboardSz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if(success1 && success2)
		{
			imgPoints1.push_back(corners1);
			imgPoints2.push_back(corners2);
			objPoints.push_back(obj);
		}
	}
	//check if pattern was detected in enough calibration images
	if(imgPoints1.size() < 8){ return false; }

	/**************************************************************************
	 * calibrate camera intrinsics individually for the cameras
	 *************************************************************************/
	vector<Mat> rvecs1, rvecs2, tvecs1, tvecs2; //dummy vectors, we won't use these
	Mat camMat1, camMat2, distCoeff1, distCoeff2; //these are the important matrices!
	calibrateCamera(objPoints, imgPoints1, imgSz, camMat1, distCoeff1, rvecs1, tvecs1);
	calibrateCamera(objPoints, imgPoints2, imgSz, camMat2, distCoeff2, rvecs2, tvecs2);

	/**************************************************************************
	 * calibrate stereo camera pair with intrinsics from previous step
	 * in order to get rotation and translation vectors between cameras
	 *************************************************************************/
	Mat rot, transl, E, F;
	stereoCalibrate(objPoints, imgPoints1, imgPoints2, camMat1, distCoeff1, camMat2, distCoeff2, imgSz, rot, transl, E, F,
					TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6), CALIB_FIX_INTRINSIC);

	/**************************************************************************
	 * compute the rectification transforms for each camera
	 *************************************************************************/
	Mat rect1, rect2, proj1, proj2, Q;
	stereoRectify(camMat1, distCoeff1, camMat2, distCoeff2, imgSz, rot, transl, rect1, rect2, proj1, proj2, Q,
				  CALIB_ZERO_DISPARITY, (crop2commonArea ? 0 : 1) );

	/**************************************************************************
	 * finally, get the rectification maps for each camera
	 *************************************************************************/

	return true;
}
