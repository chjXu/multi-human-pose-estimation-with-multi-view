#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <multi_human_estimation/camera.h>

using namespace std;

class Kinect : public Camera
{
public:
    Kinect():id(-1), fx(0.0), fy(0.0), cx(0.0), cy(0.0) {}
    virtual ~Kinect(){}

    /**
     * function: 读取相机内参
     * param_path：文件路径
     * scale : 内参尺度大小
    */
    virtual bool readParameters(string param_path, double& scale);
    virtual void setParameters();
    virtual void readImageFromVideo(string video_path);
    virtual void showImage();

private:
    cv::Mat image;
    cv::Mat cameraMatrix, distortion;
    int id;
    double fx, fy, cx, cy;
};