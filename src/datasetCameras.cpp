#include "multi_human_estimation/datasetCameras.h"


DataSetCamera::DataSetCamera():fx(0.0), fy(0.0), cx(0.0), cy(0.0), id(-1){

}


DataSetCamera::~DataSetCamera(){

}


void DataSetCamera::setID(const int id){
    this->id = id;
}


bool DataSetCamera::valid() const{
    return (!isZero(this->fx)) && (!isZero(this->fy)) && (!isZero(this->cx)) && (!isZero(this->cy));
}


bool DataSetCamera::isZero(const double &a) const{
    return (abs(a) - 0.0 < 1e-3);
}

void DataSetCamera::updateTransformation(const Eigen::Matrix3d& R, const Eigen::Matrix<double, 3, 1>& t){
    this->R = R;
    this->t = t;
}

void DataSetCamera::updateRotation(const Eigen::Matrix3d& R){
    this->R = R;
}

void DataSetCamera::updateTranslation(const Eigen::Matrix<double, 3, 1>& t){
    this->t = t;
}

void DataSetCamera::setRotation(const Eigen::Matrix3d& R){
    for(int i=0; i<3;++i){
        for(int j=0; j<3; ++j){
            this->R(i, j) = R(i, j);
        }
    }
}

void DataSetCamera::setTranslation(const tf::Vector3& trans){
    this->t = Eigen::Matrix<double, 3, 1>(trans[0], trans[1], trans[2]);
}
