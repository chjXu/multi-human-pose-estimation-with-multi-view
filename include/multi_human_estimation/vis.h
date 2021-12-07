/*
 * @Description:  
 * @Author: chengjun_xu
 * @Data: Do not edit
 * @LastAuthor: Do not edit
 * @LastEditTime: 2021-12-02 20:22:42
 */
#pragma once
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "multi_human_estimation/pose.h"
#include "multi_human_estimation/datasetCameras.h"
#include "multi_human_estimation/gl.h"

using namespace std;

struct pixel_location{
    int x;
    int y;
    pixel_location():x(0),y(0){}
    pixel_location(int _x, int _y):x(_x),y(_y){}
};

class Vis
{
public:
    Vis(ros::Publisher &pub);
    ~Vis();


    /**
     * 功能：转换数据集根关节点到像素坐标
     * 参数：root_joint_3d 一帧图像中，所有人的根关节；DC 数据集相机对象
     * 返回：void
    */
    void transRootJoint(const vector<Pose> &root_joint_3d, const DataSetCamera& DC);

    /**
     * 功能：可视化数据集根关节点
     * 参数：
     * 返回：void
    */
    void showRootJoint();

    /**
     * @brief add image
     * img: 传入的图像
     */
    void addImage(const cv::Mat &img);

    /**
     * @brief 连接骨骼
     * 输入：2个CV格式的点
     * 输出：在图像中画线
     */
    void drawLine(const pixel_location &, const pixel_location &, const int person_index);

    /**
     * @description: 
     * @param {Marker&} line_list
     * @param {int} a
     * @param {int} b
     * @return {*}
     */    
    void drawLine(visualization_msgs::Marker& line_list, const vector<Joint_3d>& pose_3d, int a, int b);

    /**
     * @brief 在图像中画出骨骼点
     * 输出：骨骼点
     * 输出：
     */
    void drawPoint(const pixel_location &, const int person_index);

    /**
     * @brief 重载，在图像中画出骨骼点
     * 输入：骨骼点的容器
     * 输出：
     */
    void drawPoint(const vector<Pose> &);

    /**
     * @brief 显示图像
     * 输入：
     * 输出：
     */
    void showImage(int camera_index);

    /**
     * @description: 
     * @param {vector<Pose>} &skeletons
     * @param {DataSetCamera&} DC
     * @return {*}
     */    
    void transToWorld(const vector<Pose> &skeletons, const DataSetCamera& DC);

    /**
     * @brief 将三维骨架可视化
     * 输入：三维骨骼数据
     * 输出：可视化界面
     */
    void drawSkeletons(const vector<Pose>& , const DataSetCamera& );

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */    
    void printInfo(const vector<Pose>& pose);
private:
    int person_nums;

    vector<vector<pixel_location>> rootPixelLocation;
    cv::Mat image;
    vector<cv::Scalar> colors;

    static vector<vector<int>> body_edges;

    ros::Publisher *pose_pub;

    vector<Pose> new_poses;

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */    
    Eigen::Matrix<double, 3, 1> cameraToWorld(const Eigen::Matrix<double, 3, 1>&, const DataSetCamera&);
};