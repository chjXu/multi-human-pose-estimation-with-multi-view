#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

using namespace std;

class HD
{
public:
    HD(){}
    virtual ~HD(){}

    /**
     * function: 读取相机内参
     * param_path：文件路径
     * id : 相机索引
     * scale : 内参尺度大小
    */
    virtual void readParameters(string param_path, double scale =1.0);
    
    /**
     * 设置相机内参，fx,fy,cx,cy
    */
    virtual void setParameters();


private:
    string video_path, image_path;
    bool is_video;
    cv::Mat cameraMatrix, distortion;
    int id;
    double fx, fy, cx, cy;
};