#ifndef STEREOVISIONBLACKBOX_H
#define STEREOVISIONBLACKBOX_H

#include<opencv2/opencv.hpp>
#include<QString>

using namespace cv;
using namespace std;

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
    /// The images should be pairs showing exactly the same chessboard calibration pattern, altering the pose from pair to pair.
    /// Thus, the vector lists must have the same lenght.
    /// \param leftImgs: A vector containing the calibration images for the left camera.
    /// \param rightImgs: A vector containing the calibration images for the right camera.
    /// \param chessboardSize: The size of the used chessboard calibration pattern.
    /// \return True if successfull.
    ///
    bool addCalibrationImages(vector<Mat> leftImgs, vector<Mat> rightImgs, Size chessboardSize);

    ///
    /// \brief Allows to add a single pair of calibration images for left and right camera.
    /// The pair should show exactly the same chessboard calibration pattern, altering the pose from pair to pair.
    /// \param leftImg: A calibration image for the left camera.
    /// \param rightImg: A calibration image for the right camera.
    /// \param chessboardSize: The size of the used chessboard calibration pattern.
    /// \return True if successfull.
    ///
    bool addCalibrationImagePair(Mat leftImg, Mat rightImg, Size chessboardSize);

    ///
    /// \brief Empties the vector list containing calibration images.
    ///
    void resetCalibrationImages();

    ///
    /// \brief Calibrates the cameras based on the calibration images that have been added.
    /// The results are undistort and rectification maps for each camera, which can be used in future to undistort and rectify camera
    /// images in order to perform stereo matching.
    /// \return False if: 1) less thann 8 calibration images have been supplied or 2) the chessboard could be detected in less then 8 images.
    ///
    bool calibrateCameras();

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

private:
    int cols;
    int rows;
    Size imgSz;
    Size chessboardSz;
    vector<Mat> calibImgsLeft;
    vector<Mat> calibImgsRight;

};

#endif // STEREOVISIONBLACKBOX_H
