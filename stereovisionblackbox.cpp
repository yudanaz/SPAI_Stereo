#include "stereovisionblackbox.h"

StereoVisionBlackBox::StereoVisionBlackBox(int rows, int cols)
	: singleCamsCalibrated(false), stereoCamsCalibrated(false), metricCalibrated(false), channels(3), algorithm(SEMI_GLOBAL_BLOCK_MATCHING)
{
	this->cols = cols;
	this->rows = rows;
	this->imgSz.width = cols;
	this->imgSz.height = rows;
	setStereoMatchingParameters(64,5,0,0,0,0,5,0,0,1);
}

bool StereoVisionBlackBox::addCalibrationImages(vector<Mat> leftImgs, vector<Mat> rightImgs, Size chessboardSize,
												CalibrationImageType calibType)
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
	for (uint i = 0; i < leftImgs.size(); ++i)
	{
		if(calibType == CALIB_STEREO_CAMERAS)
		{
			this->calibStereo_ImgsLeft.push_back( checkForGrayScale(leftImgs[i]) );
			this->calibStereo_ImgsRight.push_back( checkForGrayScale(rightImgs[i]) );
		}
		else
		{
			this->calibSingle_ImgsLeft.push_back( checkForGrayScale(leftImgs[i]) );
			this->calibSingle_ImgsRight.push_back( checkForGrayScale(rightImgs[i]) );
		}
	}

	this->chessboardSz = chessboardSize;
	return true;
}

bool StereoVisionBlackBox::addCalibrationImagePair(Mat leftImg, Mat rightImg, Size chessboardSize,
												   CalibrationImageType calibType)
{
	if(leftImg.cols != this->cols || leftImg.rows != this->rows || rightImg.cols != this->cols || rightImg.rows != this->rows)
	{ return false; }

	if(calibType == CALIB_STEREO_CAMERAS)
	{
		this->calibStereo_ImgsLeft.push_back( checkForGrayScale(leftImg) );
		this->calibStereo_ImgsRight.push_back( checkForGrayScale(rightImg) );
	}
	else
	{
		this->calibSingle_ImgsLeft.push_back( checkForGrayScale(leftImg) );
		this->calibSingle_ImgsRight.push_back( checkForGrayScale(rightImg) );
	}
	this->chessboardSz = chessboardSize;
	return true;
}

void StereoVisionBlackBox::resetCalibrationImages(CalibrationImageType calibType)
{
	if(calibType == CALIB_STEREO_CAMERAS)
	{
		calibStereo_ImgsLeft.clear();
		calibStereo_ImgsRight.clear();
	}
	else
	{
		calibSingle_ImgsLeft.clear();
		calibSingle_ImgsRight.clear();
	}
}

bool StereoVisionBlackBox::calibrateCameras(bool crop2commonArea)
{
	if(this->calibStereo_ImgsLeft.size() < 8){ return false; }

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
	for (uint i = 0; i < calibStereo_ImgsLeft.size(); ++i)
	{
		vector<Point2f> corners1, corners2;
		bool success1 = findChessboardCorners((Mat)calibStereo_ImgsLeft[i], chessboardSz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		bool success2 = findChessboardCorners((Mat)calibStereo_ImgsRight[i], chessboardSz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
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
	Mat camMat1, camMat2, distCoeff1, distCoeff2; //these are the important matrices
	if(calibSingle_ImgsLeft.size() >= 8)
	{
		calibrateCamera(objPoints, imgPoints1, imgSz, camMat1, distCoeff1, rvecs1, tvecs1);
		calibrateCamera(objPoints, imgPoints2, imgSz, camMat2, distCoeff2, rvecs2, tvecs2);
		singleCamsCalibrated = true;
	}

	/**************************************************************************
	 * calibrate stereo camera pair with intrinsics from previous step
	 * (if those have been computed, else compute intrinsic in this step)
	 * in order to get rotation and translation vectors between cameras
	 *************************************************************************/
	int calibFlag = singleCamsCalibrated ? CALIB_FIX_INTRINSIC : (CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);
	Mat rot, transl, E, F;
	stereoCalibrate(objPoints, imgPoints1, imgPoints2, camMat1, distCoeff1, camMat2, distCoeff2, imgSz, rot, transl, E, F,
					TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6), calibFlag);

	/**************************************************************************
	 * compute the rectification transforms for each camera
	 *************************************************************************/
	Mat rect1, rect2, proj1, proj2, Q;
	stereoRectify(camMat1, distCoeff1, camMat2, distCoeff2, imgSz, rot, transl, rect1, rect2, proj1, proj2, Q,
				  CALIB_ZERO_DISPARITY, crop2commonArea ? 0 : 1);

	/**************************************************************************
	 * finally, get the rectification maps for each camera
	 *************************************************************************/
	Mat rectif1x, rectif1y, rectif2x, rectif2y;
	initUndistortRectifyMap(camMat1, distCoeff1, rect1, proj1, imgSz, CV_32FC1, rectif1x, rectif1y);
	initUndistortRectifyMap(camMat2, distCoeff2, rect2, proj2, imgSz, CV_32FC1, rectif2x, rectif2y);
	updateRectifyMaps(rectif1x, rectif1y, rectif2x, rectif2y);

	stereoCamsCalibrated = true;
	return true;
}

bool StereoVisionBlackBox::saveCalibration(QString filename)
{
	FileStorage fs(filename.toStdString(), FileStorage::WRITE);
	if(!fs.isOpened()){ return false; }
	fs << "rectifyMap1x" << rectifyMaps1[0];
	fs << "rectifyMap1y" << rectifyMaps1[1];
	fs << "rectifyMap2x" << rectifyMaps2[0];
	fs << "rectifyMap2y" << rectifyMaps2[1];
	fs.release();
	return true;
}

bool StereoVisionBlackBox::loadCalibration(QString filename)
{
	Mat rectif1x, rectif1y, rectif2x, rectif2y;
	FileStorage fs(filename.toStdString(), FileStorage::READ);
	if(!fs.isOpened()){ return false; }
	fs["rectifyMap1x"] >> rectif1x;
	fs["rectifyMap1y"] >> rectif1y;
	fs["rectifyMap2x"] >> rectif2x;
	fs["rectifyMap2y"] >> rectif2y;
	fs.release();
	updateRectifyMaps(rectif1x, rectif1y, rectif2x, rectif2y);
	stereoCamsCalibrated = true;
	return true;
}

Mat StereoVisionBlackBox::getDisparityMap(Mat left, Mat right, Mat *out_left_rectified, Mat *out_right_rectified)
{
	if(!stereoCamsCalibrated){ return Mat(); }

	//rectify
	Mat left_, right_, disp;
	remap(left, left_, rectifyMaps1[0], rectifyMaps1[1], INTER_LINEAR, BORDER_CONSTANT, Scalar());
	remap(right, right_, rectifyMaps2[0], rectifyMaps2[1], INTER_LINEAR, BORDER_CONSTANT, Scalar());
//	imshow("orig left", left);
//	imshow("rectif left", left_);
//	imshow("orig right", right);
//	imshow("rectif right", right_);cvWaitKey();
	*out_left_rectified = left_.clone();
	*out_right_rectified = right_.clone();

	//compute disparity map
	if(algorithm == SEMI_GLOBAL_BLOCK_MATCHING)
	{
		if(left.channels() != channels)
		{
			channels = left.channels();
			sgbm.P1 = 8 * channels * sgbm.SADWindowSize * sgbm.SADWindowSize;
			sgbm.P2 = 32 * channels * sgbm.SADWindowSize * sgbm.SADWindowSize;
		}
		sgbm(left_, right_, disp);
	}
	else
	{
		sbm(left_, right_, disp);
	}
	return disp / 16; //return real values (disp values are multiple of 16, see http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereosgbm-operator)
}

void StereoVisionBlackBox::setStereoMatchingParameters(int nrOfDisparities, int SADWindowSize, int minDisparity, int prefilterSize,
													   int preFilterCap, int textureThresh, float uniquenessRatio, int speckleWindowSize,
													   int speckleRange, int channels)
{
	if(minDisparity >= 0)
	{
		sbm.state->minDisparity = minDisparity;
		sgbm.minDisparity = minDisparity;
	}
	if(nrOfDisparities > 0)
	{
		//number of disparities must be divisble by 16 because of the openCV implementation
		while(nrOfDisparities % 16 != 0){ ++nrOfDisparities; }
		sbm.state->numberOfDisparities = nrOfDisparities;
		sgbm.numberOfDisparities = nrOfDisparities;
	}
	if(SADWindowSize > 0)
	{
		if(SADWindowSize % 2 == 0){ ++SADWindowSize; }//window size (pixel neighborhood) must be odd
		sbm.state->SADWindowSize = SADWindowSize;
		sgbm.SADWindowSize = SADWindowSize;
	}
	if(prefilterSize > 0)
	{
		sbm.state->preFilterSize = prefilterSize;
	}
	if(preFilterCap > 0)
	{
		sbm.state->preFilterCap = preFilterCap;
		sgbm.preFilterCap = preFilterCap;
	}
	if (textureThresh >= 0)
	{
		sbm.state->textureThreshold = textureThresh;
	}
	if(uniquenessRatio >= 0)
	{
		sbm.state->uniquenessRatio = uniquenessRatio;
		sgbm.uniquenessRatio = uniquenessRatio;
	}
	if(speckleWindowSize > 0)
	{
		sbm.state->speckleWindowSize = speckleWindowSize;
		sgbm.speckleWindowSize = speckleWindowSize;
	}
	if(speckleRange > 0)
	{
		sbm.state->speckleRange = speckleRange;
		sgbm.speckleRange = speckleRange;
	}

	if(channels > 0)
	{
		this->channels = channels;
	}
	sgbm.P1 = 8 * this->channels * SADWindowSize * SADWindowSize;
	sgbm.P2 = 32 * this->channels * SADWindowSize * SADWindowSize;
}

///////////////////////////////////////////////////////////////////////////////
/// PRIVATE:                                                                 //
///////////////////////////////////////////////////////////////////////////////
void StereoVisionBlackBox::updateRectifyMaps(Mat rectif1x, Mat rectif1y, Mat rectif2x, Mat rectif2y)
{
	rectifyMaps1.clear();
	rectifyMaps1.push_back(rectif1x);
	rectifyMaps1.push_back(rectif1y);
	rectifyMaps2.clear();
	rectifyMaps2.push_back(rectif2x);
	rectifyMaps2.push_back(rectif2y);
}

Mat StereoVisionBlackBox::checkForGrayScale(Mat img)
{
	//if necessary, convert to gray-scale
	if(img.channels() > 1 )
	{
		Mat img_;
		cvtColor(img, img_, CV_BGR2GRAY);
		return img_;
	}
	return img;
}
