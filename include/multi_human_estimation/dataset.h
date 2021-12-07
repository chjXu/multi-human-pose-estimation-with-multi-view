#pragma once

// This is a script to deal dataset
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <jsoncpp/json/json.h>

// 读取文件夹中的文件头文件
#include <sys/types.h>
#include <dirent.h>

#include "multi_human_estimation/pose.h"
#include "multi_human_estimation/vis.h"
#include "multi_human_estimation/datasetCameras.h"
#include "multi_human_estimation/associate.h"
#include "multi_human_estimation/gl.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

using namespace std;

/**
数据集格式：00000001.json
{
    "img_path:" xxx
    "camera":[
        {
            "id":0
            "K":[]
            "D":[]
            "R":[]
            "t":[]
        },{

        }
    ]
    "bodies:"[
        "id":0,
        "joint_2d:" [],
        "root_3d:"[]
    ]

    "gt_3d:"[
        {
            "id:"0,
            "joint_3d:"[]
        }
    ]
}

*/


class ImageProcess{
public:
    ImageProcess(){}
    virtual ~ImageProcess(){}
    vector<DataSetCamera> cameras;  //相机参数，与姿态顺序对应

// protected:
    virtual void readImagePath(Json::Value& root);

    /**
     * @brief 读取相机参数函数
     *
     * @param root
     * @return ** void
     */
    virtual void readCameraParametersFromFile(Json::Value &root);

    /**
     * @brief 重载读取相机参数函数
     *
     * @param root
     * @return ** DataSetCamera
    */
    virtual void readCameraParametersFromFile(Json::Value &root, DataSetCamera& DC);
    virtual void readBodies(Json::Value& root);
    virtual void readImage();
    virtual void showImage();
    virtual cv::Mat getImage() const{
        return image;
    }
    virtual void projection(Pose &pose);

protected:
    string image_path;
    cv::Mat image;
};


class Dataset : public ImageProcess //, public GLshow
{
private:
    std::ifstream fin;
    vector<Pose> poses;
    vector<int> frames;

    int frame_num;

    Json::Reader reader;
    Json::Value root;
public:
    string _campus_dataset_path;
    string _shelf_dataset_path;
    string _cmu_dataset_path;

    // int _campus_max_frames;

    enum DataName{
        cmu = 0,
        shelf,
        campus
    }dataname;

public:
    /**
     * 数据集构造函数
     * data_path:数据路径
     * data_name：数据集名称
     * frame_num：融合数据的帧数
    */
    Dataset(string data_path, string data_name, int _frame_num);
    ~Dataset();


    /**
     * @brief 添加要使用的视频帧索引
     * 输出：视频帧索引
     * 输出：
     */
    void addFrames(vector<int> &);

    /**
     * @brief 从JSON文件中读取相关信息
     * 仅包含图像路径信息和关节信息
     * @param frame_index JSON文件索引
     * 输出：
     */
    void readJSONFile(int frame_index);

    /**
     * @brief 从JSON文件中读取相机信息，因为相机信息仅需要读取一遍即可
     *
     * @param frame_index
     * @return ** DataSetCamera
     */
    DataSetCamera readCameraParameterFromJSONFile(int frame_index);

    /**
     * @brief 从JSON文件中读取关节信息
     *
     * @param root
     * @return ** void
     */
    void readBodies(Json::Value& root);

    /**
     * @brief 清除临时数据
     *
     * @return ** void
     */
    void clear();

    /**
     * @brief Get the Poses object
     *
     * @return ** vector<Pose>
     */
    vector<Pose> getPoses() const{
        return poses;
    }

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    vector<Pose> loadData();
};
