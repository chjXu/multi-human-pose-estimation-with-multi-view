#include <multi_human_estimation/kinect.h>

bool Kinect::readParameters(string param_path, double &scale)
{
    cv::FileStorage fs;
    cv::Mat cameraMatrix_origin;

    if(fs.open(param_path, cv::FileStorage::READ)){
        fs["cameraMatrix"] >> cameraMatrix_origin;
        this->cameraMatrix = cameraMatrix_origin.clone();
        this->cameraMatrix.at<double>(0, 0) *= scale;
        this->cameraMatrix.at<double>(1, 1) *= scale;
        this->cameraMatrix.at<double>(0, 2) *= scale;
        this->cameraMatrix.at<double>(1, 2) *= scale;

        this->distortion = cv::Mat::zeros(1, 5, CV_64F);

        cout << "CameraMatrix load successfully!" << endl;

        fs.release();
        return true;
    }
    else{
        cout << "No calibration file: calib_color.yalm, using default calibration setting" << endl;
        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        distortion = cv::Mat::zeros(1, 5, CV_64F);
    }
    return false;
}