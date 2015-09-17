#ifndef STEREOVISIONBLACKBOX_H
#define STEREOVISIONBLACKBOX_H

#include<opencv2/opencv.hpp>
#include<QString>

using namespace cv;
using namespace std;

enum StereoMatchingAlgorithm{ BLOCK_MATCHING, SEMI_GLOBAL_BLOCK_MATCHING };
enum CalibrationImageType{ CALIB_IMGS_SINGLE_CAMS, CALIB_IMGS_STEREO_CAMS };

class StereoVisionBlackBox
{
public:
	///
	/// \brief Inits the black box with the size of the camera images given in rows and columns.
	/// \param rows
	/// \param cols
	///
	StereoVisionBlackBox(int rows, int cols);

	/// \brief Allows to add calibration images for left and right camera as vector lists.
	/// You should preferably provide separate calibration images for single camera and stereo camera rig calibration.
	/// For single camera: The chessboard pattern is recorded for each camera separately, allowing to cover more image area
	/// and thus improving the quality for the intrinsic parameter estimation.
	/// For stereo camera calbration: The images should be pairs showing exactly the same chessboard calibration pattern,
	/// altering the pose from pair to pair. Thus, the vector lists must have the same lenght.
	/// \param leftImgs: A vector containing the calibration images for the left camera.
	/// \param rightImgs: A vector containing the calibration images for the right camera.
	/// \param chessboardSize: The size of the used chessboard calibration pattern.
	/// \param calibType: Defines if images for single camera calibration or stereo camera rig calibration are passed to this method.
	/// \return True if successfull.
	///
	bool addCalibrationImages(vector<Mat> leftImgs, vector<Mat> rightImgs, CalibrationImageType calibType);

	///
	/// \brief Overloaded method that allows to add calibration images by providing a containing folder.
	/// \param leftImageFolder
	/// \param rightImageFolder
	/// \return
	///
	bool addCalibrationImages(QString leftImageFolder, QString rightImageFolder, CalibrationImageType calibType);

	///
	/// \brief Allows to add a single pair of calibration images for left and right camera.
	/// For single camera: The chessboard pattern is recorded for each camera separately, allowing to cover more image area
	/// and thus improving the quality for the intrinsic parameter estimation.
	/// For stereo camera calbration: The pair should show exactly the same chessboard calibration pattern,
	/// altering the pose from pair to pair.
	/// \param leftImg: A calibration image for the left camera.
	/// \param rightImg: A calibration image for the right camera.
	/// \param chessboardSize: The size of the used chessboard calibration pattern.
	/// \param calibType: Defines if images for single camera calibration or stereo camera rig calibration are passed to this method.
	/// \return True if successfull.
	///
	bool addCalibrationImagePair(Mat leftImg, Mat rightImg, CalibrationImageType calibType);

	///
	/// \brief Empties the vector list containing calibration images.
	/// \param calibType: Defines which list should be reset (calibration images for single camera or stereo camera rig)
	///
	void resetCalibrationImages(CalibrationImageType calibType);

	///
	/// \brief Calibrates the cameras based on the calibration images that have been added.
	/// The results are undistort and rectification maps for each camera, which can be used in future to undistort and rectify camera
	/// images in order to perform stereo matching.
	/// \param crop2commonArea: Flag to enable/disable cropping of the resulting images to the common image area, i.e. no black areas
	/// (that is, the images resulting from applying the rectification maps to any input images). Default is true.
	/// The idea behind this is that for areas that don't have corresponding image information in both image, depth cannot be detected.
	/// \return False if: 1) less thann 8 calibration images have been supplied or 2) the chessboard could be detected in less then 8 images.
	///
	bool calibrateCameras(Size chessboardSize, bool crop2commonArea = true);

	///
	/// \brief Saves the stereo camera calibration parameters (i.e. undistort and rectify maps) to a file
	/// \param filename: The name of the calibration file.
	/// \return True if successfull;
	///
	bool saveCalibration(QString filename);

	///
	/// \brief Loads the stereo camera calibration parameters (i.e. undistort and rectify maps) from a file
	/// \param filename: The name of the calibration file.
	/// \return True if successfull;
	///
	bool loadCalibration(QString filename);

	///
	/// \brief Computes a disparity map based on previous camera calibration and the supplied images.
	/// The semi-global block matching algorithm is used. If you want to use the simpler BM algorithm, you'll have to write your own code.
	/// \param left: Image from left camera.
	/// \param right: Image from right camera.
	/// \param out_left_rectified: Out parameter in which the rectified left images is stored.
	/// \param out_right_rectified: Out parameter in which the rectified right images is stored.
	/// \return The disparity map as 16 bit (ushort) image.
	///
	Mat getDisparityMap(Mat left, Mat right, Mat *out_left_rectified, Mat *out_right_rectified);

	///
	/// \brief Allows to set the stereo matching parameters. If a negative number is supplied, the parameter is set to the default value.
	/// \param minDisparity: Minimum possible disparity value. Normally, it is zero, but can be set higher if zero disparity
	/// doesn't occur, e.g. when cameras are placed far apart.
	/// \param nrOfDisparities Maximum disparity minus minimum disparity. The value is always greater than zero.
	///	Must be divisible by 16. Non-matching values are incremented until divisible by 16.
	/// \param SADWindowSize: the linear size of the blocks compared by the algorithm.
	/// Larger block size implies smoother, though less accurate disparity map.
	/// Smaller block size gives more detailed disparity map, but there is higher chance for algorithm to find a wrong correspondence.
	/// \param prefilterSize: Averaging window size: ~5x5..21x21
	/// \param prefilterCap: The output of pre-filtering is clipped by [-preFilterCap,preFilterCap]
	/// \param textureThresh: The disparity is only computed for pixels with textured enough neighborhood
	/// \param uniquenessRatio: Margin in percentage by which the best (minimum) computed cost function value should “win” the second best value to consider the found match correct.
	/// Normally, a value within the 5-15 range is good enough.
	/// \param specklewindowSize: Disparity variation window
	/// \param speckleRange: Acceptable range of variation in window
	/// \param channels: Number of channels in the images on which disparity is going to be computed.
	///
	void setStereoMatchingParameters(int nrOfDisparities = -1, int SADWindowSize = -1, int minDisparity = -1,
									 int prefilterSize = -1, int prefilterCap = -1,
									 int textureThresh = -1, float uniquenessRatio = -1,
									 int speckleWindowSize = -1, int speckleRange = -1, int channels = 3);

	void setStereoMatchingAlgorithm(StereoMatchingAlgorithm type = SEMI_GLOBAL_BLOCK_MATCHING){ algorithm = type; }

	///
	/// \brief Computes the rectified image from the original image, for left or right camera.
	/// \param img: The original image.
	/// \param leftOrRight01: Set 0 for left and else for right image.
	/// \return
	///
	Mat getRectifiedImage(Mat img, int leftOrRight01);

	Mat getDepthInCentimenters();

	///
	/// \brief Calibrates the depth metric, i.e. informs the algorithm which disparity value correspondends to
	/// which real-world depth in centimeters, i.e. distance from the camera.
	/// At least two images for different depths have to be supplied. A linear mapping is computed.
	/// \param image: An image in which the central pixel's depth is mapped to the supplied value.
	/// \param centimeters2CentralPixel: The value to which the central pixel's depth is mapped.
	///
	void calibrateDepthMetric(Mat image, int centimeters2CentralPixel);

	///
	/// \brief Returns true if cameras have already been successfully calibrated as a stereo camera rig.
	///
	bool isCalibrated(){ return this->stereoCamsCalibrated; }

	///
	/// \brief Returns true if the depth metric has been calibrated
	///
	bool depthMetricCalibrated(){ return this->metricCalibrated; }

private:
	void updateRectifyMaps(Mat rectif1x, Mat rectif1y, Mat rectif2x, Mat rectif2y);
	Mat checkForGrayScale(Mat img);

	int cols;
	int rows;
	Size imgSz;
	Size chessboardSz;
	vector<Mat> calibStereo_ImgsLeft;
	vector<Mat> calibStereo_ImgsRight;
	vector<Mat> calibSingle_ImgsLeft;
	vector<Mat> calibSingle_ImgsRight;
	vector<Mat> rectifyMaps1;
	vector<Mat> rectifyMaps2;
	bool singleCamsCalibrated;
	bool stereoCamsCalibrated;
	bool metricCalibrated;
	StereoBM sbm;
	StereoSGBM sgbm;
	int channels;
	StereoMatchingAlgorithm algorithm;
};

#endif // STEREOVISIONBLACKBOX_H
