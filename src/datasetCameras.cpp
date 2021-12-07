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
    return (abs(a) - 0.0 > 1e-4);
}
