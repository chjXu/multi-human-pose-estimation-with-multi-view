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
        ROS_ERROR("Invaild DataName in Dataset constructor.");
        return;
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

    cout << dataset_path << endl;
    fin.open(dataset_path, std::ios::binary);

    if(reader.parse(fin, root, false)){
       readCameraParametersFromFile(root);
    }
    fin.close();
}

void Dataset::listenerCameraPose(vector<int> &f_nums){
    // 读取所有相机的参数（包括内参和外参）
    readCameraParameterFromJSONFile();
    ROS_INFO("Instric matrix are read!");

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

    for(int i=0; i<root["cameras"].size(); ++i){
        DataSetCamera DC;
        DC.setID(root["cameras"][i]["id"].asInt());

        DC.fx = root["cameras"][i]["K"][0][0].asDouble();
        DC.fy = root["cameras"][i]["K"][1][1].asDouble();
        DC.cx = root["cameras"][i]["K"][0][2].asDouble();
        DC.cy = root["cameras"][i]["K"][1][2].asDouble();

        for(unsigned int j=0; j<3; ++j){
            for(unsigned int k=0; k<3; ++k){
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
    for(int i=0; i<root["img_to_json"].size(); ++i){
        string
        image_path = "/media/xuchengjun/D/dataset/cmu_test/"
                   + root["img_to_json"][i]["cam_id"].asString() + "/";

        image_path += root["img_name"].asString();
        // cout << image_path << endl;
        this->img_paths.push_back(image_path);
    }

}


void ImageProcess::readBodies(Json::Value& root){

}


void ImageProcess::readImage(std::string img_path){
    // cout << img_path << endl;
    cv::Mat image = cv::imread(img_path);
    this->imgs.emplace_back(image.clone());
}

void ImageProcess::readImage(){
    for(auto it:this->img_paths){
        readImage(it);
    }
}


void ImageProcess::showImage(int id, cv::Mat &img){
    // cv::resize(img, img, cv::Size(832, 512));
    // cout << img.size() << endl;
    cv::imshow("Img"+to_string(id), img);
    cv::waitKey(0);
}

void ImageProcess::projection(cv::Mat &img, vector<Pose> &pose){
    for(unsigned int i=0; i < pose.size(); ++i){
        vector<Joint_2d> joints = pose[i].get2DPose();
        // cout << joints.size() << endl;
        for(unsigned int j=0; j < joints.size(); ++j){
            // cout << joints[j].x << "  " <<  joints[j].y << endl;
            cv::circle(img, cv::Point((int)joints[j].x, (int)joints[j].y), 4, (255,0, 255), 4);
        }
    }
}

void Dataset::readJSONFile(int file_index, int ass_num){

    char path [100];
    sprintf(path, "%d/%08d.json", ass_num, file_index); // 2, 00000000

    // cout << this->root_path + "/data/" + path << endl;
    fin.open(this->root_path + "/data/" + path, std::ios::binary);

    if(reader.parse(fin, root, false)){
        readImagePath(root);

        // 在可视化测试的时候，这儿要打开，不然报错
        // readCameraParametersFromFile(root);

        // readRoots(root);

        readBodies(root);
    }

    fin.close();
}

void Dataset::addFrames(vector<int> &frames){
    this->frames = frames;
}

vector<double> Dataset::getRootJoint(const Json::Value &array){
    vector<double> res;
    // cout << array.size() << endl;
    // cout << array[0].size() << endl;
    // cout << array[0][0].asDouble() << endl;
    for(int i=0; i < array.size(); ++i){ //2
        for(int j=0; j<array[i].size(); ++j){
            res.push_back(array[i][j].asDouble());
        }
    }

    if(isRootValid(res))
        return res;
    else
        return {};
}

vector<double> Dataset::getPred2DPose(const Json::Value &array){
    vector<double> res;

    for(int i=0; i < array.size(); ++i){ //2
        for(int j=0; j<array[i].size(); ++j){
            res.push_back(array[i][j].asDouble());
        }
    }

    if(is2DPoseValid(res))
        return res;
    else
        return {};
}

bool Dataset::is2DPoseValid(const vector<double> & pose){
    int count = 0;
	double prob_sum = 0.0;
	for(int i=0;i < pose.size() / 3;i++)
	{
		if(pose[3 * i + 2] > 1.0)
		{
			prob_sum += pose[3 * i + 2];
			count++;
		}
	}

    // cout << prob_sum << " " << count << " " << prob_sum/count << endl;
	double prob_eval = prob_sum/count;

	if(prob_eval > 1.8)
        return true;
    else
        return false;
}

bool Dataset::isRootValid(const vector<double> & pose){
    int count = 0;
	double prob_sum = 0.0;
	for(int i=0;i < pose.size() / 4;i++)
	{
		if(pose[4 * i + 3] > 1.0)
		{
			prob_sum += pose[4 * i + 3];
			count++;
		}
	}

    // cout << prob_sum << " " << count << " " << prob_sum/count << endl;
	double prob_eval = prob_sum/count;

	if(prob_eval > 1.8)
        return true;
    else
        return false;
}


void Dataset::readBodies(Json::Value& root){
    this->poses.clear();


    // cout << root["img_to_json"].size() << endl; // 相机数
    // cout << root["img_to_json"][0].size() << endl; // 3, cam_id, pred_2d, root_3d
    // cout << root["img_to_json"][0]["root_3d"].size() << endl; // 人数
    // cout << root["img_to_json"][0]["root_3d"][0].isArray() << endl;
    for(int i=0; i<root["img_to_json"].size(); ++i){    // 相机数
        vector<Pose> pose;
        for(int j = 0; j < root["img_to_json"][i]["root_3d"].size(); ++j){
            int cam_id = root["img_to_json"][i]["cam_id"].asInt();
            vector<double> joint_2d = getPred2DPose(root["img_to_json"][i]["pred_2d"][j]);
            vector<double> root_3d = getRootJoint(root["img_to_json"][i]["root_3d"][j]);

            // 判断人体姿态是否有效，对无效的姿态进行删除
            if(joint_2d.empty() || root_3d.empty())
                continue;

            Pose new_pose;
           // cout << "原始相机ID： " << root["camera"]["id"].asInt() << endl;
            new_pose.setCameraID(cam_id);
            new_pose.set2DPose(joint_2d);
            new_pose.setRootPose(root_3d);

            pose.push_back(new_pose);
        }

        this->poses.push_back(pose);

        // for(int j=0; j<root["img_to_json"][i]["root_3d"].size(); ++j){
        //     for(int k=0; k < 4; ++k){
        //         root_3d.push_back(root["img_to_json"][i]["root_3d"][0][j][k].asDouble());
        //     }
        // }
    }

    // for(auto it:root_3d){
    //     cout << it << endl;
    // }

    // vector<double> joint_2d;
    // for(int i=0; i<root["img_to_json"]["joint_2d"].size(); ++i){
    //     for(int j=0; j<root["img_to_json"]["joint_2d"][i].size(); ++j){
    //         joint_2d.push_back(root["img_to_json"]["joint_2d"][i][j].asDouble());
    //     }
    // }

    // Pose new_pose;
    // // cout << "原始相机ID： " << root["camera"]["id"].asInt() << endl;
    // new_pose.setCameraID(root["camera"]["id"].asInt());
    // new_pose.set2DPose(joint_2d);
    // new_pose.setRootPose(root_3d);
    //
    // poses.push_back(new_pose);  // 保存一个视角下所有的姿态信息
}

void Dataset::clear(){
    this->cameras.clear();
    this->poses.clear();
    this->frames.clear();
    this->img_paths.clear();
    this->imgs.clear();
}

vector<Pose> Dataset::loadData(){
    // return this->poses;
    return {};
}

void Dataset::printCamInfo(DataSetCamera &DC){
    ROS_INFO("Test Function..........");
    cout << "camera: " << DC.getID() << endl;
    cout << "fx: " << DC.fx << "  fy: " << DC.fy << " cx: " << DC.cx << " cy: " << DC.cy << endl;
    cout << "R: " << DC.R << "\n";
    cout << "t: " << DC.t << endl;
}

void Dataset::testData(int num){
    ROS_INFO("Test Dataset...........");

    if(this->img_paths.empty() || this->poses.empty())
        ROS_ERROR("Test ERROR!");

    // 读取图片
    for(auto it : this->img_paths){
        readImage(it); // 得到所有图像
    }

    // 画点
    // cout << this->poses.size() << endl;
    // cout << this->poses[0].size() << endl;
    for(int i=0; i < this->poses.size(); ++i){
        if(this->imgs[i].empty())
            continue;

        cout << "Camera: " << i << " has pose: " << this->poses[i].size() << endl;
        projection(this->imgs[i], this->poses[i]);
    }

    // 显示图像
    for(int i=0; i < imgs.size(); ++i){
        showImage(i, this->imgs[i]);
    }
}

void help(){
    std::cout << "Please enter two parameters: " << endl;
    std::cout << "The first parameter is dataset name. Choose CMU, Shelf or Campus." << "\n";
    std::cout << "and the second parameter is the frames number. 1~975." << endl;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "dataset_test");

    if(!ros::ok())
        return 0;

    if(argc < 3)
        ROS_ERROR("Please enter dataset name and frames.");

    std::string dataset_name;
    int frame_num;

    for(size_t i = 1; i < (size_t)argc; ++i){
        std::string param(argv[i]);

        if(param == "--h" || param == "--help" || param == "help" || param == "h"){
            help();
            ros::shutdown();
            return 0;
        }
        else if(param == "CMU"){
            dataset_name = "CMU";
        }else if(param == "Shelf"){
            dataset_name = "Shelf";
        }else if(param == "Campus"){
            dataset_name = "Campus";
        }else{
            frame_num = stoi(param);
        }
    }

    string data_path = "/media/xuchengjun/D/dataset/cmu_test";

    ros::NodeHandle n;
	ros::Publisher pose_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",1);
	ros::Rate loop_rate(20);

    vector<int> ass_frames = {0, 1,
                        // 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
                        // 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30
                        };
    if(ass_frames.size() < 0 || ass_frames.size() > 30){
        ROS_ERROR("Vector size is not equal to the frame_num.");
        ros::shutdown();
        return 0;
    }

    Dataset *dataset = new Dataset(data_path, dataset_name, ass_frames.size());

    // Vis *vis = new Vis(pose_pub);
    //
    Associate *ass = new Associate();

    // GLshow *gl = new GLshow();


    dataset->listenerCameraPose(ass_frames);
    // dataset->printCamInfo(dataset->getCams()[0]); // 测试用

    for(int i=0; i < dataset->getCams().size(); ++i)
        ass->addCameraInfo(dataset->getCams()[i], i);


    // 相机测试 (ok)
    // for(int i=0; i<dataset->getCams().size(); ++i){
    //     dataset->printCamInfo(dataset->getCams()[i]);
    // }


    int index = 0;
    ROS_INFO("There are all %d frames to be tested.", frame_num);
    // 这是所有文件，帧数自定义
    // int file_num = frame_num;
    for(int file_index=0; file_index < frame_num; ++file_index){
        // while(ros::ok()){
        dataset->clear();

        dataset->addFrames(ass_frames);

        dataset->readJSONFile(file_index, ass_frames.size());

        // dataset->testData(ass_frames.size()); //测试用（ok）

        dataset->readImage();

        // 匹配
        ass->addPoseInfo(dataset->getPoses());
        ass->run(Mode::triangulation);
        //
        // vis->addImage(dataset->getImage());
        //
        // vis->showImage(index);
        //
        // vis->drawSkeletons(ass->getResult(), cam2world);

        // gl->run();



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
    }

    delete dataset;
    // delete vis;
    delete ass;
    // delete gl;

    return 0;
}
