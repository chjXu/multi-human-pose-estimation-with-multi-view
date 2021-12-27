#include "multi_human_estimation/dataset.h"

Dataset::Dataset(string data_path, string data_name, int _frame_num):frame_num(_frame_num){
    if(data_name == "CMU"){
        dataname = DataName::cmu;
        this->root_path = data_path;
        this->_cmu_dataset_path = data_path + "/data/" + data_name;
    }
    else if(data_name == "SHELF"){
        dataname = DataName::shelf;
        this->root_path = data_path;
        this->_shelf_dataset_path = data_path + "/data/" + data_name;
    }
    else if(data_name == "CAMPUS"){
        dataname = DataName::campus;
        this->root_path = data_path;
        this->_campus_dataset_path = data_path + "/data/" + data_name;
    }
    else{
        cout << "Invaild DataName!" << endl;
    }

    tf_listener = new tf::TransformListener();
}

Dataset::~Dataset(){
    if(tf_listener != nullptr)
        delete tf_listener;
}

// 相机参数更新完成
void Dataset::readCameraParameterFromJSONFile(){
    string dataset_path;

    if(dataname == DataName::cmu)
        dataset_path = this->root_path + "/cameraPose/cmu_cameras.json";
    else if(dataname == DataName::shelf)
        dataset_path = this->root_path + "/cameraPose/shelf_cameras.json";
    else if(dataname == DataName::campus)
        dataset_path = this->root_path + "/cameraPose/campus_cameras.json";

    fin.open(dataset_path, std::ios::binary);

    if(reader.parse(fin, root, false)){
       readCameraParametersFromFile(root);
    }
    fin.close();
}

void Dataset::listenerCameraPose(vector<int> &f_nums){
    // 读取所有相机的参数（包括内参和外参）
    readCameraParameterFromJSONFile();

    // 更新外参
    for(int i=0; i < f_nums.size(); ++i){
        tf::StampedTransform transform;

        while(ros::ok()){
            try
            {
                tf_listener->waitForTransform("world","camera_"+to_string(f_nums[i]),ros::Time(0),ros::Duration(3.0));
                tf_listener->lookupTransform("world", "camera_"+to_string(f_nums[i]), ros::Time(0), transform);
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }

            tf::Quaternion tf_q = transform.getRotation();

            Eigen::Matrix3d rot = Eigen::Quaterniond(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z()).toRotationMatrix();

            this->cameras[f_nums[i]].setRotation(rot);
            this->cameras[f_nums[i]].setTranslation(transform.getOrigin());

            break;
        }
    }

    ROS_INFO("The cameras parameters are updated.");
}

void ImageProcess::readCameraParametersFromFile(Json::Value &root, DataSetCamera& DC){
    DC.setID(root["camera"]["id"].asInt());
    DC.fx = root["camera"]["K"][0][0].asDouble();
    DC.fy = root["camera"]["K"][1][1].asDouble();
    DC.cx = root["camera"]["K"][0][2].asDouble();
    DC.cy = root["camera"]["K"][1][2].asDouble();


    for(unsigned int i=0; i<3; ++i){
        for(unsigned int j=0; j<3; ++j){
            DC.R(i, j) = root["camera"]["R"][i][j].asDouble();
        }
    }

    for(unsigned int i=0; i<3; ++i){
        DC.t(i, 0) = root["camera"]["t"][i][0].asDouble();
    }

    DC.R = DC.R.transpose();
    DC.t = DC.R * -DC.t;
}


void ImageProcess::readCameraParametersFromFile(Json::Value &root){
    this->cameras.clear();

    cout << root["cameras"].size() << endl;
    for(int i=0; i<root["cameras"].size(); ++i){
        DataSetCamera DC;
        DC.setID(root["cameras"][i]["id"].asInt());

        DC.fx = root["cameras"][i]["K"][0][0].asDouble();
        DC.fy = root["cameras"][i]["K"][1][1].asDouble();
        DC.cx = root["cameras"][i]["K"][0][2].asDouble();
        DC.cy = root["cameras"][i]["K"][1][2].asDouble();

        for(unsigned int j=0; i<3; ++j){
            for(unsigned int k=0; j<3; ++k){
                DC.R(j, k) = root["cameras"][i]["R"][j][k].asDouble();
            }
        }

        for(unsigned int j=0; j<3; ++j){
            DC.t(j, 0) = root["cameras"][i]["t"][j][0].asDouble();
        }

        DC.R = DC.R.transpose();
        DC.t = DC.R * -DC.t;

        this->cameras.emplace_back(DC);
    }
}

void ImageProcess::readImagePath(Json::Value& root){
    this->image_path = root["img_path"].asString();
}


void ImageProcess::readBodies(Json::Value& root){

}


void ImageProcess::readImage(){
    image = cv::imread(this->image_path);
}


void ImageProcess::showImage(){
    if(image.empty()) return;
    cv::imshow("Img", this->image);
    cv::waitKey(0);
}

void ImageProcess::projection(Pose &pose){
    vector<Joint_2d> joints = pose.get2DPose();
    cout << joints.size() << endl;
    for(unsigned int i=0; i < joints.size(); ++i){
        cout << joints[i].x << "  " <<  joints[i].y << endl;
        cv::circle(this->image, cv::Point(joints[i].x, joints[i].y), 4, (255,0, 255), 4);
    }
}

void Dataset::readJSONFile(int frame_index){

    char path [100];
    sprintf(path, "%d/%02d_%08d.json", 0, 0, frame_index);

    // cout << this->_cmu_dataset_path + path << endl;
    fin.open(this->_cmu_dataset_path + path, std::ios::binary);

    if(reader.parse(fin, root, false)){
        readImagePath(root);

        // 在可视化测试的时候，这儿要打开，不然报错
        // readCameraParametersFromFile(root);

        readBodies(root);
    }

    fin.close();
}

void Dataset::addFrames(vector<int> &frames){
    this->frames = frames;
}

void Dataset::readBodies(Json::Value& root){
    this->poses.clear();

    vector<double> root_3d;
    for(int i=0; i<root["bodyies"]["root_3d"].size(); ++i){
        for(int j=0; j<root["bodyies"]["root_3d"][i].size(); ++j){
            root_3d.push_back(root["bodyies"]["root_3d"][i][j].asDouble());
        }
    }

    vector<double> joint_2d;
    for(int i=0; i<root["bodyies"]["joint_2d"].size(); ++i){
        for(int j=0; j<root["bodyies"]["joint_2d"][i].size(); ++j){
            joint_2d.push_back(root["bodyies"]["joint_2d"][i][j].asDouble());
        }
    }

    Pose new_pose;
    // cout << "原始相机ID： " << root["camera"]["id"].asInt() << endl;
    new_pose.setCameraID(root["camera"]["id"].asInt());
    new_pose.set2DPose(joint_2d);
    new_pose.setRootPose(root_3d);

    poses.push_back(new_pose);  // 保存一个视角下所有的姿态信息
}

void Dataset::clear(){
    this->cameras.clear();
    this->poses.clear();
    this->frames.clear();
}

vector<Pose> Dataset::loadData(){
    return this->poses;
}

void Dataset::printCamInfo(DataSetCamera &DC){
    ROS_INFO("Test Function..........");
    cout << "fx: " << DC.fx << "  fy: " << DC.fy << " cx: " << DC.cx << " cy: " << DC.cy << endl;
    cout << "R: " << DC.R << "\n";
    cout << "t: " << DC.t << endl;
}




int main(int argc, char** argv){
    ros::init(argc, argv, "dataset_test");

    if(argc < 2)
        ROS_ERROR("Please enter dataset name.");

    string data_path = "/home/xuchengjun/catkin_ws/src/multi_human_estimation/data";

    ros::NodeHandle n;
	ros::Publisher pose_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",1);
	ros::Rate loop_rate(20);

    vector<int> frames = {0, 1,
                        // 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
                        // 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30
                        };

    Dataset *dataset = new Dataset(data_path, argv[1], frames.size());

    // Vis *vis = new Vis(pose_pub);
    //
    // Associate *ass = new Associate();

    // GLshow *gl = new GLshow();


    dataset->listenerCameraPose(frames);
    dataset->printCamInfo(dataset->getCams()[0]); // 测试用
    // ass->addCameraInfo(dataset->readCameraParameterFromJSONFile(i), i);


    // int index = 0;
    // // 这是所有文件，帧数自定义
    // int file_num = 100;
    // // for(int file=0; file<file_num; ++file)
    // while(index < 1 && ros::ok()){
    //     dataset->clear();
    //
    //     // 这是一个文件里的所有帧
    //     for(int i=0; i<frames.size(); ++i){
    //         //预处理
    //
    //         dataset->addFrames(frames);
    //
    //         dataset->readJSONFile(frames[i]);
    //
    //         dataset->readImage();
    //
    //         // 匹配
    //         ass->addPoseInfo(dataset->getPoses());
    //         if(i != 0)  ass->run(0, i, false, Mode::Ceres);
    //
    //         vis->addImage(dataset->getImage());
    //
    //         vis->showImage(index);
    //
    //         vis->drawSkeletons(ass->getResult(), cam2world);
    //
    //         // gl->run();
    //     }
    //
    //     // vis->transRootJoint(dataset->getPoses(), dataset->cameras[i]);
    //
    //     // vis->addImage(dataset->getImage());
    //
    //     // vis->showRootJoint();
    //
    //     // vis->drawPoint(dataset->getPoses());
    //
    //     // // cout << dataset->getPoses()[0].getCameraID() << endl;
    //     // // cout << dataset->cameras[i].getID() << endl;
    //     // vis->showImage(index);
    //
    //     index++;
    //
    //     ros::spinOnce();
    // }

    delete dataset;
    // delete vis;
    // delete ass;
    // delete gl;

    return 0;
}
