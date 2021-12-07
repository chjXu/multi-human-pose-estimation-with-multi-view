#include <iostream>
#include <string>
#include <multi_human_estimation/hd.h>
#include <multi_human_estimation/camera.h>
#include <multi_human_estimation/model.h>
#include <multi_human_estimation/dataset.h>
#include <chrono>

using namespace std;

int main()
{
    string param_path = "/home/xuchengjun/catkin_ws/src/multi_human_estimation/calibration_data";
    string video_path_1 = "/media/xuchengjun/D/dataset/panoptic-toolbox/170407_haggling_a1/hdVideos/hd_00_00.mp4";
    string video_path_2 = "/media/xuchengjun/D/dataset/panoptic-toolbox/170407_haggling_a1/hdVideos/hd_00_03.mp4";
    string video_path_3 = "/media/xuchengjun/D/dataset/panoptic-toolbox/170407_haggling_a1/hdVideos/hd_00_05.mp4";
    string video_path_4 = "/media/xuchengjun/D/dataset/panoptic-toolbox/170407_haggling_a1/hdVideos/hd_00_16.mp4";
    string image_path_1 = "/media/xuchengjun/D/dataset/panoptic-toolbox/170407_haggling_a1/hdImgs/00_00/00_00_00001928.jpg";
    string image_path_3 = "/media/xuchengjun/D/dataset/panoptic-toolbox/170407_haggling_a1/hdImgs/00_03/00_03_00001928.jpg";
    string model_path = "/home/xuchengjun/catkin_ws/src/multi_human_estimation/model/model.pt";
    string model_path_1 = "/home/xuchengjun/human.pt";


    // Camera *hdCamera1 = new HD(image_path_1, false);
    // Camera *hdCamera2 = new HD(image_path_3, false);

    // Camera *hdCamera1 = new Camera(video_path_1, true, 0);
    // Camera *hdCamera2 = new Camera(video_path_2, true, 3);
    // Camera *hdCamera3 = new Camera(video_path_3, true, 5);
    // Camera *hdCamera4 = new Camera(video_path_4, true, 16);

    // Camera *hdCamera1 = new Camera(Dataset::_campus_dataset_path, false, 0);
    // Camera *hdCamera2 = new Camera(Dataset::_campus_dataset_path, false, 1);
    // Camera *hdCamera3 = new Camera(Dataset::_campus_dataset_path, false, 2);

    //hdCamera1->readParameters(param_path);
    //hdCamera1->setParameters();
    //hdCamera2->readParameters(param_path, 3);
    
    //Model model(model_path_1);

    while(1){
        auto start_time = chrono::system_clock::now();

        // hdCamera1->readImageFromVideo("hd_1");
        // hdCamera2->readImageFromVideo("hd_3");
        // hdCamera3->readImageFromVideo("hd_5");
        // hdCamera4->readImageFromVideo("hd_16");

        // hdCamera1->readImageFromFile("hd_1", "campus");
        // hdCamera2->readImageFromFile("hd_2", "campus");
        // hdCamera3->readImageFromFile("hd_3", "campus");
        

        //model.test();

        auto end_time = chrono::system_clock::now();
        auto duration = chrono::duration<double>(end_time-start_time).count();
        cout << "FPS: " << 1 / duration << endl;
    }

    // delete hdCamera1;
    ///delete hdCamera2;
    return 0;
}