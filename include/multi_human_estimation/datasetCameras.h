#pragma once
#include <iostream>
#Include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class DataSetCamera{
public:
    double fx, fy, cx, cy;
    Eigen::Matrix3d R;
    Eigen::Matrix<double, 3, 1> t;

    DataSetCamera():fx(0.0), fy(0.0), cx(0.0), cy(0.0), id(-1){}
    ~DataSetCamera(){}

    int getID() const{
        return id;
    }

    void setID(const int id){
        this->id = id;
    }

    bool invalid() const {
        return !isZero(fx) && !isZero(fy) && !isZero(cx) && !isZero(cy);
    }
private:
    int id;

    bool isZero(double &a){
        return abs(a) > 1e-4;
    }
};
