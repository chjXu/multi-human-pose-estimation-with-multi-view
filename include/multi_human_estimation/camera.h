#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <dirent.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


#include "multi_human_estimation/hd.h"
#include "multi_human_estimation/dataset.h"

using namespace std;

class Camera
{
public:
    Camera(string video_path, bool is_Video, int id):id(id),
    fx(0.0), fy(0.0), cx(0.0), cy(0.0), video_path(video_path), 
    image_path(video_path), is_video(is_video)
    {
        if(is_Video){
            if((dir = opendir(video_path.c_str())) == nullptr){
                cout << "Video path isn't exist." << endl;
                return;
            }else
            {
                cout << video_path << endl;
                cap.open(video_path);
                if(!cap.isOpened()) 
                    throw "Video file is not open!";
            }
        } 
    }
 
    ~Camera(){
        free(dir);
    }

    void readParameters(string param_path, double scale =1.0);
    void setParameters();

    /**
     * @brief Set the Parameters object
     * 设置内参，fx,fy,cx,cy
     */
    void setParameters(double &fx, double &fy, double &cx, double &cy);

    /**
     * function : 读取图像文件
     * image_file_path：文件路径
     * winName：窗口名称
     * data_type：数据集类型
    */
    void readImageFromFile(string winName, string data_type);

    /**
     * function : 读取视频
     * video_path：视频路径
     * winName：窗口名称
    */
    void readImageFromVideo(string video_path, string winName);
    /**
     * 函数重载
    */
    void readImageFromVideo(string winName);
    cv::Mat readImageFromVideo();

    /**
     * function:显示图片
     * winName：窗口名称
     * image：图像
    */
    void showImage(string& winName, cv::Mat& image, bool vis = true);

private:
    string video_path, image_path;
    bool is_video;
    cv::Mat cameraMatrix, distortion; // 相机内参矩阵和畸变参数
    Eigen::Matrix3d R;  //相机旋转
    Eigen::Matrix<double, 3, 1> t;  //相机平移

    int id;
    double fx, fy, cx, cy;

    cv::VideoCapture cap;
    DIR* dir;

    cv::Mat image;
};