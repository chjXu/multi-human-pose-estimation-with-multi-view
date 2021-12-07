#pragma once
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class DataSetCamera{
public:
    double fx, fy, cx, cy;
    Eigen::Matrix3d R;
    Eigen::Matrix<double, 3, 1> t;

    DataSetCamera();
    ~DataSetCamera();

    /**
     * @brief 返回ID
     * @param
     * @return int
    */
    int getID() const{
        return this->id;
    }

    /**
     * @brief 设置ID
     * @param id
     * @return void
    */
    void setID(const int id);

    /**
     * @brief 判断对象是否有效
     * @param
     * @return bool
    */
    bool valid() const;

private:
    int id;

    /**
     * @brief 判断数值是否为0
     * @param number
     * @return bool
    */
    bool isZero(const double &a) const;
};
