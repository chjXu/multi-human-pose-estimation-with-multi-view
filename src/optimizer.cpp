/*
 * @Description:
 * @Author: chengjun_xu
 * @Data: Do not edit
 * @LastAuthor: Do not edit
 * @LastEditTime: 2021-11-17 15:12:18
 */


#include "multi_human_estimation/optimizer.h"

CostFunction_cam_2d_2d
::CostFunction_cam_2d_2d(const Joint_2d _observe_point1, const Joint_2d _observe_point2,
                         const DataSetCamera _DC1, const DataSetCamera _DC2)
                :observe_point1(_observe_point1),observe_point2(_observe_point2),
                DC1(_DC1), DC2(_DC2){

}

CostFunction_cam_2d_2d::~CostFunction_cam_2d_2d(){

}

// template <typename T>
// bool CostFunction_cam_2d_2d::operator()(const T *const depths, T *residual) const{
//     Eigen::Matrix<T,3,1> Cam_point1, Cam_point2;
// 	Eigen::Matrix<T,3,1> World_point1, World_point2;

//     Cam_point1(2,0) = depths[0];
//     Cam_point2(2,0) = depths[1];
//     Cam_point1(0,0) = (T(observe_point1.x) - DC1.cx) * Cam_point1(2,0) / DC1.fx;
//     Cam_point1(1,0) = (T(observe_point1.y) - DC1.cy) * Cam_point1(2,0) / DC1.fy;
//     Cam_point2(0,0) = (T(observe_point2.x) - DC2.cx) * Cam_point2(2,0) / DC2.fx;
//     Cam_point2(1,0) = (T(observe_point2.y) - DC2.cy) * Cam_point2(2,0) / DC2.fy;

//     World_point1 = DC1.R.cast<T>() * Cam_point1 + DC1.t.cast<T>();
//     World_point2 = DC2.R.cast<T>() * Cam_point2 + DC2.t.cast<T>();

//     residual[0] = ((World_point1(0,0) - World_point2(0,0)) * (World_point1(0,0) - World_point2(0,0))
//                 +(World_point1(1,0) - World_point2(1,0)) * (World_point1(1,0) - World_point2(1,0))
//                 +(World_point1(2,0) - World_point2(2,0)) * (World_point1(2,0) - World_point2(2,0)));


//     return true;
// }
