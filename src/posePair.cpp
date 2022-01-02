/*
 * @Description:
 * @Author: chengjun_xu
 * @Data: Do not edit
 * @LastAuthor: Do not edit
 * @LastEditTime: 2021-11-16 20:56:03
 */
#include "multi_human_estimation/posePair.h"

PosePair::PosePair():delta(1000){

}

PosePair::~PosePair(){

}


void PosePair::setLabel(const int _label_1, const int _label_2){
    this->label_1 = _label_1;
    this->label_2 = _label_2;
}

void PosePair::setIndex(const int _index_1, const int _index_2){
    this->index_1 = _index_1;
    this->index_2 = _index_2;
}

void PosePair::setCam(const int _cam_1, const int _cam_2){
    this->cam_1 = _cam_1;
    this->cam_2 = _cam_2;
}

void PosePair::setDelta(const double & d){
    this->delta = d;
}

void PosePair::setGroup(int group){
    this->group = group;
}
