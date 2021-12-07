/*
 * @Description:  
 * @Author: chengjun_xu
 * @Data: Do not edit
 * @LastAuthor: Do not edit
 * @LastEditTime: 2021-11-17 10:36:27
 */
#include "multi_human_estimation/gl.h"

GLshow::GLshow(){
    glfwInit();
    glWindow = glfwCreateWindow( 800, 600, "OneFLOW GLFW OpenGL Test", NULL, NULL );

    if(glWindow == nullptr){
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }
}

GLshow::~GLshow(){
    //delete glWindow;
}

void GLshow::run(){
    if(!this->createWindows()){
        vector<Pose> tmp = loadData();
        drawAxis();
        drawPoints();
        drawLines();
    }

    destoryWindows();
}

bool GLshow::createWindows(){
    return glfwWindowShouldClose( this->glWindow );
}

void GLshow::destoryWindows(){
    
}

void GLshow::drawAxis(){

}

void GLshow::drawPoints(){

}

void GLshow::drawLines(){
    
}

vector<Pose> GLshow::loadData(){
    return {};
}