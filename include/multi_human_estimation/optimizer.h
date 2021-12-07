/*
 * @Description:  cere_optimization
 * @Author: chengjun_xu
 * @Data: Do not edit
 * @LastAuthor: Do not edit
 * @LastEditTime: 2021-11-17 15:12:33
 */

#include <iostream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "multi_human_estimation/pose.h"
#include "multi_human_estimation/datasetCameras.h"


class CostFunction_cam_2d_2d{//仿函数
public:
   /**
    * @description: 
    * @param {Joint_2d} _observe_point1
    * @param {Joint_2d} _observe_point2
    * @param {DataSetCamera} _DC1
    * @param {DataSetCamera} _DC2
    * @return {*}
    */
  	CostFunction_cam_2d_2d(Joint_2d _observe_point1, Joint_2d _observe_point2, DataSetCamera _DC1, DataSetCamera _DC2);

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */    
    ~CostFunction_cam_2d_2d();

    // 残差的计算
    /**
     * @description: 
     * @param {*}
     * @return {*}
     */    
    template <typename T>
    bool operator()(const T *const depths, T *residual) const{
        Eigen::Matrix<T,3,1> Cam_point1, Cam_point2;
        Eigen::Matrix<T,3,1> World_point1, World_point2;

        Cam_point1(2,0) = depths[0];
        Cam_point2(2,0) = depths[1];
        Cam_point1(0,0) = (T(observe_point1.x) - DC1.cx) * Cam_point1(2,0) / DC1.fx;
        Cam_point1(1,0) = (T(observe_point1.y) - DC1.cy) * Cam_point1(2,0) / DC1.fy;
        Cam_point2(0,0) = (T(observe_point2.x) - DC2.cx) * Cam_point2(2,0) / DC2.fx;
        Cam_point2(1,0) = (T(observe_point2.y) - DC2.cy) * Cam_point2(2,0) / DC2.fy;

        World_point1 = DC1.R.cast<T>() * Cam_point1 + DC1.t.cast<T>();
        World_point2 = DC2.R.cast<T>() * Cam_point2 + DC2.t.cast<T>();

        residual[0] = ((World_point1(0,0) - World_point2(0,0)) * (World_point1(0,0) - World_point2(0,0))
                    +(World_point1(1,0) - World_point2(1,0)) * (World_point1(1,0) - World_point2(1,0))
                    +(World_point1(2,0) - World_point2(2,0)) * (World_point1(2,0) - World_point2(2,0)));


        return true;
    }

private:
	Joint_2d observe_point1;
	Joint_2d observe_point2;

    DataSetCamera DC1;
    DataSetCamera DC2;
};