/*
 * @Description:  All Pose Paired
 * @Author: chengjun_xu
 * @Data: Do not edit
 * @LastAuthor: Do not edit
 * @LastEditTime: 2021-11-16 20:55:36
 */
#pragma once

#include <iostream>

using namespace std;

class PosePair{
public:
    PosePair();
    ~PosePair();

    /**
     * @description: getGroup
     * @param {*}
     * @return {int}
     */
    int getGroup() const{
        return this->group;
    }

    /**
     * @description: setGroup
     * @param {int}
     * @return {int}
     */
    void setGroup(int group);

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    int getLabel_1() const{
        return this->label_1;
    }

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    int getLabel_2() const{
        return this->label_2;
    }

    /**
     * @description:
     * @param {int} _label_1
     * @param {int} _label_2
     * @return {*}
     */
    void setLabel(const int _label_1, const int _label_2);

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    int getIndex_1() const{
        return this->index_1;
    }

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    int getIndex_2() const{
        return this->index_2;
    }

    /**
     * @description:
     * @param {int} _index_1
     * @param {int} _index_2
     * @return {*}
     */
    void setIndex(const int _index_1, const int _index_2);


    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    int getCam_1() const{
        return this->cam_1;
    }

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    int getCam_2() const{
        return this->cam_2;
    }

    /**
     * @description:
     * @param {int} _cam_1
     * @param {int} _cam_2
     * @return {*}
     */
    void setCam(const int _cam_1, const int _cam_2);

    /**
     * @description:
     * @param {double} d
     * @return {*}
     */
    void setDelta(const double & d);

    /**
     * @description:
     * @param {*}
     * @return {*}
     */
    double getDelta() const{
        return this->delta;
    }

    /**
     * @description:
     * @param {PosePair&} pair_1
     * @param {PosePair&} pair_2
     * @return {*}
     */
    static bool comp(PosePair& pair_1, PosePair& pair_2){
        return pair_1.delta < pair_2.delta;
    }

private:
    int group;
    int label_1;
    int label_2;
    int index_1;
    int index_2;
    int cam_1;
    int cam_2;

    double delta;
};
