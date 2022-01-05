/*
 * @Author: your name
 * @Date: 2021-07-14 12:52:53
 * @LastEditTime: 2022-01-05 21:44:21
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /multi_human_estimation/include/multi_human_estimation/associate.h
 */
#pragma once
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <cmath>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "multi_human_estimation/pose.h"
#include "multi_human_estimation/posePair.h"
#include "multi_human_estimation/datasetCameras.h" // tf, eigen
#include "multi_human_estimation/optimizer.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// #include <human_pose_msgs/PointWithProb.h>
// #include <human_pose_msgs/HumanList.h>
// #include <human_pose_msgs/Human.h>

using namespace std;

enum Mode{
    triangulation,
    OpenCV,
    Ceres
};

struct AssPair{
    vector<int> cam_ids;
    vector<int> pose_indexs;
    int pose_label;
    
    AssPair() : pose_label(-1) {}
    AssPair(int _pose_label) : pose_label(_pose_label) {}
};

class Associate
{
public:
    Associate();
    ~Associate();
    /**
     * @brief 执行程序运行
     * @param single
     * @return ** void
     */
    void run(const Mode&& mode = Mode::triangulation);

    /**
     * @brief 添加每一帧不同视角捕获到的姿态信息
     * 输入：每一个视角下的所有人的人体姿态信息
     * 输出：
     */
    void addPoseInfo(const vector<vector<Pose>> &framePoses);

    /**
     * @brief 添加每一帧不同视角下的相机姿态信息。
     * 注意，该函数可执行一次
     * 输入：相机与世界坐标系的变换关系
     * 输出：
     */
    void addCameraInfo(const DataSetCamera &DC, int camera_id);

    /**
     * @brief 返回更新后的坐标
     * 输入：
     * 输出：一个姿态的结果
     */
    vector<Joint_3d> get3DPose() const{
        return this->pose_3d;
    }

    /**
     * @description:返回更新后的坐标
     * @param {*}
     * @return {*}:所有姿态结果
     */
    vector<Pose> getResult() const{
        return this->poses_3d;
    }

    vector<pair<int, int> > getPair() const{
        return ass_pairs;
    }

    vector<AssPair> getPairsRes() const{
        return this->ass_pairs_res;
    }

protected:
    /**
     * @brief 生成所有视角下不同姿态可配对的所有可能性
     * 输入：两帧图像数据
     * 输出：修改pose_pairs
     */
    void generatePair(vector<Pose>& pose_1, vector<Pose>& pose_2);


    /**
     * @brief 数据融合
     * 输入：两帧图像数据
     * 输出：修改一个公共3维姿态
     */
    void fusionOfEachFrame(vector<Pose>& pose_1, vector<Pose>& pose_2, bool inc = false);

    /**
     * @description: 
     * @param {bool} inc
     * @return {*}
     */    
    void fusionOfEachPose(list<PosePair> &pairs, Pose& pose, vector<Pose>& pose_set, const int target, int label);

    /**
     * @brief 数据融合
     * 输入：3D关节和其它2D信息
     * 输出：修改一个公共3维姿态
     */
    // void fusionOfIncremental(int , int);

    /**
     * @brief 计算置信度
     * 输入：配对关系
     * 输出：
     */
    bool calculateCorrespondence(PosePair &, const double & threshold = 0.4, bool inc = false);

    /**
     * @description: translate the camera point to world
     * @param {Pose} &
     * @return {void}
     */
    void transToWorldOfRoot(Pose &, const DataSetCamera& DC);

    /**
     * @description: translate the camera point to world
     * @param {Pose} &
     * @return {void}
     */
    vector<Root_3d> transToWorldOfRoot(const vector<Root_3d> &, const DataSetCamera& DC);

    /**
     * @description:
     * @param {vector<Root_3d> &, vector<Root_3d>} &
     * @return {*}
     */
    double lineToline(const vector<Root_3d> &, const vector<Root_3d> &);

    /**
     * @description:
     * @param {Root_3d} &
     * @param {vector<Root_3d>} &
     * @return {*}
     */
    double pointToline(const Root_3d &, const vector<Root_3d> &);

    /**
     * @description:
     * @param {Root_3d} &
     * @param {Root_3d} &
     * @return {*}
     */
    double pointTopoint(const Root_3d &, const Root_3d &);

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    vector<pair<int, int> > extract2DAssociation();

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */    
    vector<vector<int>> extractAssociationResults();

    /**
     * @description: 
     * @param {vector<Pose>} &
     * @param {int} target
     * @return {*}
     */    
    vector<AssPair> extract2DAssociation(const vector<Pose> &, const int target);

    /**
     * @description:计算正确配对姿态的3D姿态
     * @param {vector<vector<int> > &}
     * @return {*}
     */
    void calcualte3DPose(vector<pair<int, int> > &, const Mode& );

    /**
     * @description:
     * @param {vector<pair<int, int> >} &
     * @param {int} reference
     * @param {int} target
     * @param {int} method. 0 means triangulation by ourself. 1 means OpenCV, 2 means Ceres
     * @return {*}
     */
    void triangularization(const vector<pair<int, int> > &, const int reference, const int target, const Mode& method, bool inc = false);

    /**
     * @description: 
     * @param {pair<int, int>} &
     * @param {int} reference
     * @param {int} target
     * @return {*}
     */    
    void triangulatePose(const pair<int, int> &, const int reference, const int target);
    
    /**
     * @description: 
     * @param {pair<int, int>} &
     * @param {int} reference
     * @param {int} target
     * @return {*}
     */    
    void triangulatePose(const pair<int, int> &, const int target);
    
    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    vector<double> triangularPoints(const Joint_2d&, const Joint_2d&, const int reference, const int target);

    /**
     * @description: 
     * @param {int} ref
     * @return {*}
     */    
    Eigen::Matrix<double, 3, 1> normalization(const Joint_2d&, const int ref);
    
    /**
     * @description:
     * @param {int} reference
     * @param {int} target
     * @return {*}
     */
    void triangularCamera(const int reference, const int target, const Mode& mode);

    // void triangularCamera(const Pose &, const int target, const Mode& mode);

    /**
     * @description: 
     * @param {Pose} &
     * @param {int} reference
     * @param {int} target
     * @return {*}
     */    
    void optimizer3DLoc(Pose &, const int reference, const int target);

    /**
     * @description:
     * @param {Joint_2d&} point_1
     * @param {Joint_2d&} point_2
     * @return {*}
     */
    vector<double> OptimizerWithCereSolver(const Joint_2d& point_1, const Joint_2d& point_2, const int reference, const int target, ceres::Problem& problem);

    /**
     * @description:
     * @param {vector<Pose>} &
     * @param {int} reference
     * @return {*}
     */
    void transToReference(vector<Pose> &, int reference = 0);

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    void averageProcess(vector<Pose>& set_1, vector<Pose>& set_2, vector<pair<int, int> >&);

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    Pose average(const Pose&, const Pose&);

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    void triangulatePointsWithOpenCV(vector<Joint_2d>& set_1, vector<Joint_2d>& set_2, DataSetCamera& , DataSetCamera& , vector<cv::Point3d>& points);


    /**
     * @description:
     * @param {DataSetCamera} &DC
     * @return {*}
     */
    cv::Point2d pixel2cam(const Joint_2d& p, const DataSetCamera& DC);

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    void getPoseResult(int reference);

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    vector<Pose> poseFusion(int reference, int target, const Mode& mode);

    /**
     * @description: 
     * @param {vector<Pose>} &
     * @param {int} target
     * @param {Mode&} mode
     * @return {*}
     */    
    vector<Pose> IncrementalPoseFusion(vector<Pose> &, int target, const Mode& mode);

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    double getDistance(const Root_3d &, const Root_3d &);

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    double getScore(const Root_3d &, const Root_3d &);

private:
    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    template <typename T>
    T computeModel(const vector<T>& );

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    template <typename T>
    T dot(const vector<T>& , const vector<T>& );

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    template <typename T>
    vector<T> crossMulti(const vector<T>& , const vector<T>& );

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */    
    template <typename T>
    T det(const vector<T>& , const vector<T>& );

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */    
    template <typename T>
    bool isParallel(const vector<T>& v1, const vector<T>& v2);
    
    /**
     * @description:打印输出一帧图像中的所有姿态信息
     * @param {vector<Pose>} &
     * @return {*}
     */
    void printInfo(const vector<Pose> &);

    /**
     * @description: 打印每一个姿态信息
     * @param {Pose} &
     * @return {*}
     */
    void printPoseInfo(const Pose&);

    vector<vector<Pose>> inputs;

    vector<DataSetCamera> cameras;  //自定义相机的数量

    list<PosePair> pose_pairs;

    vector<Joint_3d> pose_3d;

    vector<Pose> poses_3d;

    vector<Pose> pose_tmp;

    vector<pair<int, int> > ass_pairs;

    vector<AssPair> ass_pairs_res;

    static int index;

    // 置信度权重系数
    double miu;
    double namada;

    // 三角化之旋转和平移
    Eigen::Matrix3d R;
    Eigen::Matrix<double, 3, 1> t;

    // 优化器
    ceres::Solver::Options options;


    // Another method of tf listener
    tf::TransformListener *tf_listener;

    int reference, target;
};
