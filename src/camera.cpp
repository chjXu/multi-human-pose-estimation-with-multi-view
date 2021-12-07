#include <multi_human_estimation/camera.h>

void Camera::setParameters(double &fx, double &fy, double &cx, double &cy){
    this->fx = fx;
    this->fy = fy;
    this->cx = cx;
    this->cy = cy;
}

void Camera::readParameters(string param_path, double scale)
{
    cv::FileStorage fs;
    cv::Mat cameraMatrix_origin;

    if(fs.open(param_path + "/calib_color_" + to_string(this->id) + ".yaml", cv::FileStorage::READ)){
        fs["cameraMatrix"] >> cameraMatrix_origin;
        this->cameraMatrix = cameraMatrix_origin.clone();
        this->cameraMatrix.at<double>(0, 0) *= scale;
        this->cameraMatrix.at<double>(1, 1) *= scale;
        this->cameraMatrix.at<double>(0, 2) *= scale;
        this->cameraMatrix.at<double>(1, 2) *= scale;

        this->distortion = cv::Mat::zeros(1, 5, CV_64F);

        cout << "CameraMatrix_" + to_string(this->id) + " load successfully!" << endl;

        fs.release();
        //return true;
    }
    else{
        cout << "No calibration file: calib_color.yalm, using default calibration setting" << endl;
        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        distortion = cv::Mat::zeros(1, 5, CV_64F);
    }
    //return false;
}

void Camera::setParameters(){
    if(!this->cameraMatrix.empty()){
        fx = cameraMatrix.at<double>(0, 0);
        fy = cameraMatrix.at<double>(1, 1);
        cx = cameraMatrix.at<double>(0, 2);
        cy = cameraMatrix.at<double>(1, 2);
    }else{
        throw "Parameters set failed.";
    }
}

void Camera::readImageFromVideo(string video_path, string winName){
    cap >> this->image;
    showImage(winName, image);
}

void Camera::readImageFromFile(string winName, string data_type){
    char image_name[100];
    if(data_type == "campus"){
        static int frame_index = 0;
        //if(++frame_index > Dataset::_campus_max_frames) frame_index = 0;
        sprintf(image_name, "Camera%d/campus4-c%d-%05d.png", this->id, this->id, frame_index);
    }else if(data_type == "shelf"){
        int frame_index = 0;
        sprintf(image_name, "Camera%d/campus4-c%d-%5d.png", this->id, this->id, frame_index);
    }else if(data_type == "cmu"){
        int frame_index = 0;
        sprintf(image_name, "Camera%d/campus4-c%d-%5d.png", this->id, this->id, frame_index);
    }else{
        cout << "Please enter right dataset name!" << endl;
    }
    // cout << this->image_path + image_name << endl;
    image = cv::imread(this->image_path + image_name);
    showImage(winName, image);
}

void Camera::readImageFromVideo(string winName){
    cap >> this->image;
    showImage(winName, image);
}

cv::Mat Camera::readImageFromVideo(){
    cap >> this->image;
    return this->image;
}


void Camera::showImage(string& winName, cv::Mat& image, bool vis){
    if(!image.empty() && vis){
        cv::resize(image, image, cv::Size(960, 540));
        cv::imshow(winName, image);
        cv::waitKey(3);
    }
}