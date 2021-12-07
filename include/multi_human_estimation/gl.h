/*
 * @Description:  
 * @Author: chengjun_xu
 * @Data: Do not edit
 * @LastAuthor: Do not edit
 * @LastEditTime: 2021-11-16 14:39:13
 */
#pragma once

#include <iostream>
#include <GLFW/glfw3.h>

#include "multi_human_estimation/pose.h"

using namespace std;


class GLshow{
public:
    GLshow();
    virtual ~GLshow();

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */    
    void run();

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */    
    bool createWindows();

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */    
    void destoryWindows();

protected:
    /**
     * @description: 
     * @param {*}
     * @return {*}
     */    
    void drawAxis();

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */    
    virtual vector<Pose> loadData();

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */    
    void drawPoints();

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */    
    void drawLines();
private:
    GLFWwindow* glWindow;
};