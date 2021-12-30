/*
 * @Author: your name
 * @Date: 2021-07-14 12:43:08
 * @LastEditTime: 2021-12-06 19:53:23
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /multi_human_estimation/include/multi_human_estimation/pose.h
 */
#pragma once
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#include "multi_human_estimation/datasetCameras.h"

using namespace std;

const float alpha = 1920.0 / 832.0;
const float belta = 1080.0 / 512.0;

struct Joint_2d
{
    float x;
    float y;
    float p;

    Joint_2d():x(0.0), y(0.0), p(0.5){}
};

struct Root_3d
{
    float x;
    float y;
    float z;
    float p;
    bool available;

    Root_3d():x(0.0), y(0.0), z(0.0), available(false){}
};

struct Joint_3d
{
    float x;
    float y;
    float z;
    bool available;

    Joint_3d():x(0.0), y(0.0), z(0.0), available(false) {}
};



class Pose
{
public:
    Pose();
    ~Pose();

    // set2Dpose from netowrk
    void set2DPose(vector<double>& joints);

    // setRoot from network
    void setRootPose(vector<double> &root);

    // setCamera id
    void setCameraID(int _camera_id);

    // setCamera parameters
    // void setCameraParameters(Camera &cam);

    // getCamera parameters
    // vector<double> getCameraParameters() const{
    //     return {};
    // }

    /**
     * @brief 设置标签ID
     * 输入：label
     * 输出：
     */
    void setLabel(const int _label);

    /**
     * @brief 获取标签ID
     * 输入：
     * 输出：label
     */
    int getLabel() const{
        return this->label;
    }

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    // get2Dpose
    vector<Joint_2d> get2DPose() const{
        return this->pose_2d;
    }

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    vector<Root_3d> getRootPose() const{
        return this->root_3d;
    }

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    int getCameraID() const {
        return this->camera_id;
    }

    /**
     * @description:
     * @param {vector<double>} &depth
     * @param {DataSetCamera} &DC
     * @param {bool} true means world and false means camera
     * @return {*}
     */
    void update3DPose(vector<double> &depth, DataSetCamera &DC, bool );

    /**
     * @description:
     * @param {vector<cv::Point3d>} &points3d
     * @param {DataSetCamera} &DC
     * @return {*}
     */
    void update3DPose(vector<cv::Point3d> &points3d, DataSetCamera &DC, bool );

    /**
     * @description:
     * @param {vector<Joint_3d>} &points3d
     * @param {DataSetCamera} &DC
     * @return {*}
     */
    void update3DPose(vector<Joint_3d> &joint3d, DataSetCamera &DC, bool );

    /**
     * @description:
     * @param {DataSetCamera} &DC
     * @return {*}
     */
    vector<Joint_2d> pixel2cam(DataSetCamera &DC);

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    vector<Joint_3d> get3DPose() const{
        return this->pose_3d;
    }

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    void set3DPose(vector<Eigen::Matrix<double, 3, 1> >& pose_3d);

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    void set3DPose(const vector<cv::Point3d>& pose_3d);

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    void set3DPose(const vector<Joint_3d>& pose_3d);

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    bool getUpdated() const{
        return this->updated;
    }

    /**
     * @description: 判断姿态是否有效
     * @param {*}
     * @return {*}
     */
    bool empty() const{
        return (pose_3d.empty() || pose_2d.empty());
    }
private:
    int camera_id;
    int root_id;
    int label;
    bool updated;
    int num_kpt;

    vector<Root_3d> root_3d;
    vector<Joint_2d> pose_2d;
    vector<Joint_3d> pose_3d;
};
