#pragma once
#include <iostream>
#include <torch/torch.h>
#include <torch/script.h>
#include <torch/data/transforms.h>

#include <vector>
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <multi_human_estimation/pose.h>

using namespace std;

class Model
{
public:
    Model(string model_path, int gpu_id=0): model_path(model_path), gpu_id(gpu_id){
        if(torch::cuda::is_available() && gpu_id >= 0){
            device = torch::Device(torch::kCUDA, gpu_id);
        }else{
            device = torch::Device(torch::kCPU);
        }

        cout << device.type() << endl;

        while(1)
        {
            try
            {
                human_model = torch::jit::load(this->model_path);
                break;
                
            }
            catch(const c10::Error& e)
            {
                std::cerr << "error loading the module" << '\n';
            }
        }

        human_model.to(device);
        std::cerr << "model load successfully!\n";
    }
    virtual ~Model(){}

    at::Tensor transpose(at::Tensor& tensor, c10::IntArrayRef dims={0, 3, 1, 2});
    at::Tensor toTensor(cv::Mat &img, bool show_output = false, bool unsqueeze = false, int unsqueeze_dim = 0);
    vector<torch::jit::IValue> toInput(at::Tensor& tensor_image);
    at::Tensor predict(cv::Mat& img, torch::jit::script::Module& human_model);

    /**
     * function:给定网络输出Tensor结果，将其结果变为常用格式
     * 按照{root:(x,y,z), pose_2d:(x,y,p)}的格式保存
    */
    vector<Pose> tensor_to_pose();

    void test();

private:
    int gpu_id;
    string model_path;
    torch::Device device = torch::Device(torch::kCPU);
    torch::jit::script::Module human_model;
};