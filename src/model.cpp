#include <multi_human_estimation/model.h>


void Model::test(){
    string image_path = "/media/xuchengjun/D/dataset/panoptic-toolbox/170407_haggling_a1/hdImgs/00_00/00_00_00001928.jpg";
    cv::Mat img = cv::imread(image_path);

    at::Tensor out = predict(img, this->human_model);
}



at::Tensor Model::predict(cv::Mat& img, torch::jit::script::Module& human_model)
{
    //trans image to tensor
    cv::resize(img, img, cv::Size(512,484));
    auto tensor_img = toTensor(img, false, true);
    tensor_img = tensor_img.clamp_max(c10::Scalar(50));
    tensor_img = tensor_img.toType(c10::kFloat).div(255);
    tensor_img = transpose(tensor_img);

    //创建输入
    auto input = toInput(tensor_img);

    //at::Tensor input = torch::rand({1,3,384,192}).to(at::kCUDA);
    at::Tensor output = human_model.forward(input).toTensor();
    
    return output;
}

vector<Pose> Model::tensor_to_pose(){
    return {};
}

at::Tensor Model::toTensor(cv::Mat &img, bool show_output, bool unsqueeze, int unsqueeze_dim)
{
    //cout << "image shape: " << img.size() << endl;
    at::Tensor tensor_image = torch::from_blob(img.data, {img.rows, img.cols,3}, at::kByte);

    if(unsqueeze){
        tensor_image.unsqueeze_(unsqueeze_dim);
        //cout << "tensor new shape: " << tensor_image.sizes() << endl;
    }

    if(show_output){
        cout << tensor_image.slice(2,0,1) << endl;
    }

    //cout << "tensor shape: " << tensor_image.sizes() << endl;
    return tensor_image;
}

at::Tensor Model::transpose(at::Tensor& tensor, c10::IntArrayRef dims)
{
    //cout << "-----------shape before: " << tensor.sizes() << endl;
    tensor = tensor.permute(dims);
    //cout << "-----------shape after: " << tensor.sizes() << endl;
    return tensor;
}

vector<torch::jit::IValue> Model::toInput(at::Tensor& tensor_image)
{
    return vector<torch::jit::IValue>{tensor_image.to(at::kCUDA)};
}