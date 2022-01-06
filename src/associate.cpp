#include "multi_human_estimation/associate.h"

int Associate::index = 0;
// double Associate::miu = 0.4;
// double Associate::namada = 0.6;

Associate::Associate():reference(0), target(0), namada(0.0), miu(0.0){
    tf_listener = new tf::TransformListener();

    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.logging_type = ceres::SILENT;
}


Associate::~Associate(){
    if(tf_listener != nullptr)
        delete tf_listener;
}


void Associate::printPoseInfo(const Pose& pose){
    cout << "Camera: " << pose.getCameraID() << endl;

    cout << "Label: " << pose.getLabel() << endl;

    vector<Joint_3d> pose_3d = pose.get3DPose();

    for(int i= 0; i < pose_3d.size(); ++i){
        if(pose_3d[i].available)
            cout << "Joint: " << i
                 << "  X: " << pose_3d[i].x
                 << "  Y: " << pose_3d[i].y
                 << "  Z: " << pose_3d[i].z << endl;
    }
    cout << endl;
}


void Associate::printInfo(const vector<Pose> &poses){
    if(poses.empty()){
        ROS_ERROR("Poses are empty()");
        return;
    }

    ROS_INFO("Print all poses info within a image.");
    // cout << "The Pose is under camera: " << poses[0].getCameraID()
    //      << " and the pose number is: " << poses.size() << endl;

    int pose_id = 0;
    for(Pose it : poses){
        printPoseInfo(it);
        ++pose_id;
    }
}


void Associate::run(const Mode&& mode){
    if(this->inputs.size() < 2){
        ROS_ERROR("Associate frames are less 2.");
        return;
    }

    this->reference = 0;
    this->target = 1;

    this->pose_tmp = poseFusion(this->reference, this->target, mode);

    // vector<vector<int> > pose_ass = extractAssociationResults();

    // for(int i=0; i < pose_ass.size(); ++i){
    //     for(int j=0; j<pose_ass[i].size(); ++j){
    //         cout << "Pose_ass: " << pose_ass[i][j] << endl;
    //     }
    // }

    // cout << "Pose_tmp size is: " << pose_tmp.size() << endl;
    // int count = 0;
    // for(auto it : pose_tmp){
    //     cout << "camera: " << it.getCameraID() << "  index: " << count << " label: " << it.getLabel() << endl;
    //     ++count;
    // }

    // printInfo(pose_tmp);

    // 优化计算其关节位置信息
    // optimizer3DLoc(this->inputs[reference][it.first]);
    // optimizer3DLoc(this->inputs[target][it.second]);

    if(this->inputs.size() > 2){
        cout << "Inputs size is: " << inputs.size() << endl;
        for(int i=2; i < this->inputs.size(); ++i){
            this->target = i;
            this->pose_tmp = IncrementalPoseFusion(this->pose_tmp, i, mode);
        }
    }

    this->ass_pairs_res = extract2DAssociation(this->pose_tmp);
}

vector<Pose> Associate::IncrementalPoseFusion(vector<Pose> &pose_tmp, int target, const Mode& mode){
    if(pose_tmp.empty()) return {};

    static int group_tmp = 1;

    ROS_INFO("Tmp pose has: %d, target pose has: %d", (int)pose_tmp.size(), (int)this->inputs[target].size());

    sort(pose_tmp.begin(), pose_tmp.end(), [](Pose &a, Pose &b){return a.getLabel() < b.getLabel();});

    int index = 0;
    static int _label = pose_tmp[pose_tmp.size() - 1].getLabel();

    for(auto &it : pose_tmp){
        triangularCamera(it.getCameraID(), target, mode);

        // generatePair
        list<PosePair> pose_pairs_tmp;

        for(int i=0; i < this->inputs[target].size(); ++i){
            PosePair pair;
            pair.setGroup(group_tmp);
            pair.setLabel(it.getLabel(), this->inputs[target][i].getLabel());
            pair.setIndex(index, i);
            pair.setCam(it.getCameraID(), this->inputs[target][i].getCameraID());

            pose_pairs_tmp.push_back(pair);
        }

        fusionOfEachPose(pose_pairs_tmp, it, this->inputs[target], target, _label);
        ++index;
    }
    
    ROS_INFO("Generate pairs: %d", index);
    this->ass_pairs = extractAssociationInc(pose_tmp, target);
    // this->ass_pairs_res = extract2DAssociation(pose_tmp, target);

    triangularization(ass_pairs, reference, target, mode, true);
    
    for(auto it : this->ass_pairs){
        pose_tmp.emplace_back(this->inputs[target][it.second]);
    }

    for(auto &it : this->inputs[target]){
        if(it.getLabel() < 0){
            pose_tmp.emplace_back(it);
        }else{
            continue;
        }
    }

    return pose_tmp;
}


vector<Pose> Associate::poseFusion(int reference, int target, const Mode& mode){
    vector<Pose> pose_tmp;

    // 通过tf监听得到相机相对姿态
    // triangularCamera(reference, target, mode);

    // 通过数学计算得到相机相对姿态
    triangularCamera(reference, target, mode); //得到R和t
    generatePair(this->inputs[reference], this->inputs[target]);
    fusionOfEachFrame(this->inputs[reference], this->inputs[target]);
    this->ass_pairs = extract2DAssociation();

    // 通过三角化计算3D姿态。
    // 应当做如下考虑：
    // 1.计算后的3D姿态应当在哪个坐标系中展示（将更新后的3D姿态分别保存至对应的坐标系下）
    // 2.未配对的2D进行查找并保存
    triangularization(ass_pairs, reference, target, mode);


    // averageProcess(this->inputs[reference], this->inputs[target], ass_pairs);

    // printInfo(this->inputs[reference]);

    // 保存已经更新的3D姿态
    for(auto it : this->ass_pairs){
        pose_tmp.emplace_back(this->inputs[reference][it.first]);
        pose_tmp.emplace_back(this->inputs[target][it.second]);
    }

    // 搜索未配对的2D姿态
    for(auto &it : this->inputs[reference]){
        if(it.getLabel() < 0){
            pose_tmp.emplace_back(it);
        }else{
            continue;
        }
    }

    for(auto &it : this->inputs[target]){
        if(it.getLabel() < 0){
            pose_tmp.emplace_back(it);
        }else{
            continue;
        }
    }

    return pose_tmp;
}

// void Associate::fusionOfIncremental(int reference, int target){
//     if(reference == target)
//         ROS_ERROR("The number of fusion frame is wrong.");
//
//     fusionOfEachFrame(this->inputs[reference], this->inputs[target]);
// }


void Associate::addPoseInfo(const vector<vector<Pose>> &framePoses){
    if(framePoses.empty())
        ROS_ERROR("The pose inputted is empty.");

    /**
    要进行匹配的姿态信息.
    framePoses大小最小为2，第一个表示第0帧所有的姿态信息，第二个为表示第1帧所有的姿态信息。
    framePoses大小最大为30（CMU数据集），5（Shelf）, 3（Campus）.
    */
    this->inputs = framePoses;
}


void Associate::addCameraInfo(const DataSetCamera &DC, int camera_id){
    if(!DC.valid()){
        ROS_ERROR("Camera %d data is invalid.", camera_id);
        return;
    }

    this->cameras.push_back(DC);
}


void Associate::generatePair(vector<Pose>& pose_1, vector<Pose>& pose_2){
    static int group = 1;

    // cout << "--------------------------------------" << endl;
    // ROS_INFO("The %d group are starting pairing.", group);
    ROS_INFO("Reference camera has %ld person, target camera has %ld person.", pose_1.size(), pose_2.size());

    if(pose_1.empty() && pose_2.empty()){
        ROS_ERROR("There are no humans in reference camera and target camera.");
        return;
    }

    if(pose_1.empty()){
        ROS_ERROR("There are no humans in reference camera.");
        return;
    }

    if(pose_2.empty()){
        ROS_ERROR("There are no humans in target camera.");
        return;
    }

    this->pose_pairs.clear();


    for(int i=0; i<pose_1.size(); ++i){
        for(int j=0; j<pose_2.size(); ++j){
            PosePair pair;
            pair.setGroup(group);
            pair.setLabel(pose_1[i].getLabel(), pose_2[j].getLabel());
            pair.setIndex(pose_1[i].getIndex(), pose_2[j].getIndex());
            pair.setCam(pose_1[i].getCameraID(), pose_2[j].getCameraID());

            this->pose_pairs.push_back(pair);
        }
    }


    ROS_INFO("Total pose pairs : %d.", (int)this->pose_pairs.size());
    ++group;
}


void Associate::triangularCamera(int reference, int target, const Mode& mode){
    bool listening = false;

    if(mode == Mode::triangulation || mode == Mode::OpenCV){
        // 计算相对位姿.
        this->R = this->cameras[reference].R.transpose() * this->cameras[target].R;

        // cout << "R: " << R << endl;

        this->t = this->cameras[reference].R.transpose() * (this->cameras[target].t - this->cameras[reference].t);
        // cout << "t: " << t << endl;
    }
    else if (mode == Mode::Ceres){
        // 这里使用ROS框架下的tf
        while(ros::ok()){
            tf::StampedTransform transform_ref, transform_tar;

            tf_listener->waitForTransform("world",
                                          "camera_" + to_string(reference),
                                          ros::Time(0),
                                          ros::Duration(1.0));
            tf_listener->lookupTransform("world",
                                         "camera_" + to_string(reference),
                                         ros::Time(0),
                                         transform_ref);

            tf_listener->waitForTransform("world",
                                         "camera_" + to_string(target),
                                         ros::Time(0),
                                         ros::Duration(1.0));
            tf_listener->lookupTransform("world",
                                         "camera_" + to_string(target),
                                        ros::Time(0),
                                        transform_tar);

            // cout << "R_ref: " << this->cameras[reference].R << endl;
            // cout << "t_ref: " << this->cameras[reference].t << endl;
            // cout << endl;



            tf::Quaternion q_r = transform_ref.getRotation();
            tf::Vector3 trans_r = transform_ref.getOrigin();

            tf::Quaternion q_t = transform_ref.getRotation();
            tf::Vector3 trans_t = transform_ref.getOrigin();

            Eigen::Quaterniond q_ref(q_r.getW(), q_r.getX(), q_r.getY(), q_r.getZ());
            Eigen::Vector3d trans_ref(trans_r.getX(), trans_r.getY(), trans_r.getZ());

            Eigen::Quaterniond q_tar(q_t.getW(), q_t.getX(), q_t.getY(), q_t.getZ());
            Eigen::Vector3d trans_tar(trans_t.getX(), trans_t.getY(), trans_t.getZ());

            // this->R = q.toRotationMatrix();
            // this->t = trans;

            this->cameras[reference].updateTransformation(q_ref.toRotationMatrix(), trans_ref);
            this->cameras[target].updateTransformation(q_tar.toRotationMatrix(), trans_tar);

            listening = true;
            break;
        }

        if(listening)
            ROS_INFO("The transformation of Camera %d and Camera %d are listened.", reference, target);
        else
            ROS_ERROR("The transformation of Camera %d and Camera %d are not listened.", reference, target);

    }else{
        ROS_ERROR("Mode error.");
    }
}


void Associate::fusionOfEachPose(list<PosePair> &pairs, Pose& pose, vector<Pose>& pose_set, const int target, int label){
    if(pairs.empty())
        return;

    static double threshold = 3.0;
    for(auto it = pairs.begin(); it != pairs.end(); ++it){
        calculateCorrespondence(*it, 2.0, true);
    }


    int count = 1;
    for(auto it = pairs.begin(); it != pairs.end(); ++it){
        ROS_INFO("Pair: %d, and %d:%d, label_1: %d, label_2: %d, score is: %f.",
                  count, (*it).getIndex_1(), (*it).getIndex_2(), (*it).getLabel_1(), (*it).getLabel_2(),
                  (*it).getDelta());

        ++count;
    }
    

    auto rank_min = min_element(pairs.begin(), pairs.end(), PosePair::comp);

    if(rank_min->getDelta() > threshold)
        return;
        
    if(pose.getLabel() > 0)
        this->inputs[target][(*rank_min).getIndex_2()].setLabel((*rank_min).getLabel_1());
    else{
        pose.setLabel(++label);
        this->inputs[target][(*rank_min).getIndex_2()].setLabel(++label);
    }
}


void Associate::fusionOfEachFrame(vector<Pose>& pose_1, vector<Pose>& pose_2, bool inc){
    if(this->pose_pairs.empty())
        return;

    for(auto it = pose_pairs.begin(); it != pose_pairs.end(); ++it){
        calculateCorrespondence(*it, 2.0);
    }


    int count = 1;
    for(auto it = pose_pairs.begin(); it != pose_pairs.end(); ++it){
        ROS_INFO("Pair: %d, and %d:%d, label_1: %d, label_2: %d, score is: %f.",
                  count, (*it).getIndex_1(), (*it).getIndex_2(), (*it).getLabel_1(), (*it).getLabel_2(),
                  (*it).getDelta());

        ++count;
    }

    auto rank_min = min_element(pose_pairs.begin(), pose_pairs.end(), PosePair::comp);
    int label_ite = 1;
    double threshold = 3.0;

    // while(!pose_pairs.empty()){
    while(!pose_pairs.empty() && rank_min->getDelta() <= threshold){
        
        ROS_INFO("pair: %d:%d is min.", (*rank_min).getIndex_1(), (*rank_min).getIndex_2());

        int label_1_old = (*rank_min).getLabel_1();
        int label_2_old = (*rank_min).getLabel_2();
        int id_1 = (*rank_min).getIndex_1();
        int id_2 = (*rank_min).getIndex_2();

        if(inc){
            this->inputs[target][id_2].setLabel(label_1_old);
        }else{
            this->inputs[reference][id_1].setLabel(label_ite);
            this->inputs[target][id_2].setLabel(label_ite);
        }

        // ROS_INFO("The pose %d of camera %d, and the pose %d of camera %d are paired. And their label are %d, %d.",
        //         this->reference, id_1, id_2, this->target,
        //         this->inputs[reference][id_1].getLabel(),
        //         this->inputs[target][id_2].getLabel());

        pose_pairs.erase(rank_min);

        for(auto it = this->pose_pairs.begin(); it != this->pose_pairs.end(); ++it){
            if((*it).getLabel_1() == label_1_old || (*it).getLabel_2() == label_2_old){
                it = pose_pairs.erase(it);
                ROS_INFO("pair: %d:%d erased", (*it).getIndex_1(), (*it).getIndex_2());
            }
        }

        rank_min = min_element(pose_pairs.begin(), pose_pairs.end(), PosePair::comp);
        ++label_ite;
    }

    ROS_INFO("Finish pairing.");
}


bool Associate::calculateCorrespondence(PosePair &pair, const double & threshold, bool inc){
    int cam_1, cam_2;
    cam_1 = pair.getCam_1();
    cam_2 = pair.getCam_2();

    // assert(this->reference = cam_1);
    // assert(this->target = cam_2);

    Pose pose_1, pose_2;
    if(inc){
        pose_1 = this->pose_tmp[pair.getIndex_1()];
        pose_2 = this->inputs[this->target][pair.getIndex_2()];
    }else{
        pose_1 = this->inputs[this->reference][pair.getIndex_1()];
        pose_2 = this->inputs[this->target][pair.getIndex_2()];
    }

	double sum_EX = 100;

    vector<Root_3d> root_1 = pose_1.getRootPose();
    vector<Root_3d> root_2 = pose_2.getRootPose();

    root_1 = transToWorldOfRoot(root_1, this->cameras[this->reference]);
    root_2 = transToWorldOfRoot(root_2, this->cameras[this->target]);

    if(root_1[0].available && root_1[1].available && root_2[0].available && root_2[1].available){
        sum_EX = lineToline(root_1, root_2);
    }
    else if(!root_1[0].available && root_1[1].available && !root_2[0].available && root_2[1].available){
        sum_EX = pointTopoint(root_1[1], root_2[1]);
    }else{
        if(!root_1[0].available && root_1[1].available && root_2[0].available && root_2[1].available)
            sum_EX = pointToline(root_1[1], root_2);
        else
            sum_EX = pointToline(root_2[1], root_1);
    }

    // cout << sum_EX << endl;
    pair.setDelta(fabs(sum_EX));
    if(sum_EX <= threshold){
        return true;
    }
    else
        return false;
}

vector<pair<int, int> > Associate::extract2DAssociation(){
    vector<pair<int, int> > pose_ass;

    for(int i = 0; i < this->inputs[this->reference].size(); ++i) {
        Pose pose_1 = this->inputs[this->reference][i];
        if(pose_1.getLabel() < 0) continue;

        for(int j = 0; j < this->inputs[this->target].size(); ++j){
            Pose pose_2 = this->inputs[this->target][j];
            if(pose_2.getLabel() < 0) continue;

            if(pose_1.getLabel() == pose_2.getLabel()){
                pose_ass.emplace_back(std::pair<int, int>(i, j));
            }
        }
    }

    ROS_INFO("Pose generated: %d", (int)pose_ass.size());
    return pose_ass;
}

vector<pair<int, int> > Associate::extractAssociationInc(const vector<Pose> &pose_tmp, const int target){
    // 从pose_tmp中提取姿态信息

    vector<pair<int, int> > pose_ass;

    for(int i = 0; i < this->pose_tmp.size(); ++i) {
        Pose pose_1 = this->pose_tmp[i];
        if(pose_1.getLabel() < 0) continue;

        for(int j = 0; j < this->inputs[target].size(); ++j){
            Pose pose_2 = this->inputs[target][j];
            if(pose_2.getLabel() < 0) continue;

            if(pose_1.getLabel() == pose_2.getLabel()){
                pose_ass.emplace_back(std::pair<int, int>(i, j));
            }
        }
    }

    ROS_INFO("Pose generated: %d", (int)pose_ass.size());
    return pose_ass;
}

vector<AssPair> Associate::extract2DAssociation(const vector<Pose> &pose_tmp){
    if(pose_tmp.empty())
        return {};

    // vector<Pose> tmp;
    // for(auto &it : pose_tmp){
    //     tmp.push_back(it);
    // }

    // for(auto & it : this->inputs[target]){
    //     tmp.push_back(it);
    // }


    vector<AssPair> pose_ass;
    map<int, int> PoseList; // tmp index, target index

    int id_ite = 0;
    for(int i = 0; i < pose_tmp.size(); ++i) {
        if(pose_tmp[i].getLabel() >= 0) {
            int cam = pose_tmp[i].getCameraID();
            int label = pose_tmp[i].getLabel();
            int index = pose_tmp[i].getIndex();
            if (PoseList.find(label) == PoseList.end()) {
                PoseList[label] = id_ite;
                pose_ass.resize(id_ite+1);
                pose_ass[id_ite].pose_label = label;
                pose_ass[id_ite].cam_ids.push_back(cam);
                pose_ass[id_ite].pose_indexs.push_back(index);
                id_ite++;
            }
            else {
                pose_ass[PoseList[label]].cam_ids.push_back(cam);
                pose_ass[PoseList[label]].pose_indexs.push_back(index);
            }
        }
    }

    // ROS_INFO("Pose generated: %d", (int)pose_ass.size());
    return pose_ass;
}


void Associate::calcualte3DPose(vector<pair<int, int> > & poses_ass, const Mode& mode){
    if(poses_ass.empty())
        ROS_INFO("There is no pair of two frames of Camera %d and Camera %d.", this->reference, this->target);

    for(auto pair : poses_ass){
        vector<Joint_2d> pose_1 = this->inputs[this->reference][pair.first].get2DPose();
        vector<Joint_2d> pose_2 = this->inputs[this->target][pair.second].get2DPose();

        if(pose_1.size() != pose_2.size())
            ROS_ERROR("The joint number of paired pose is not same.");

        vector<vector<double> > depths(2);

        if(mode == Mode::Ceres){
            ceres::Problem problem;
            ROS_INFO("Start with ceres solver.");
            cout << "Joint size: " << pose_1.size() << endl;
            for(int i=0; i<pose_1.size(); ++i){
                cout << "Joint: " << i << " pixel location: " << pose_1[i].x << " " << pose_1[i].y << "  "
                                                             << pose_2[i].x << " " << pose_2[i].y << endl;
                vector<double> depth = OptimizerWithCereSolver(pose_1[i], pose_2[i], this->reference, this->target, problem);

                cout << "Ceres: " << depth[0] << " " << depth[1] << endl;

                depths[0].push_back(depth[0]);
                depths[1].push_back(depth[1]);
            }
        }else{
            ROS_ERROR("Mode error.");
        }

        this->inputs[this->reference][pair.first].update3DPose(depths[0], this->cameras[this->reference], true);
        this->inputs[this->target][pair.second].update3DPose(depths[1], this->cameras[this->target], true);
    }

    if(mode == Mode::Ceres || mode == Mode::triangulation){
        averageProcess(this->inputs[this->reference], this->inputs[this->target], poses_ass);
    }
}


vector<double> Associate::OptimizerWithCereSolver(const Joint_2d& point_1, const Joint_2d& point_2, const int reference, const int target, ceres::Problem& problem){
    double depth[2] = {};

	problem.AddResidualBlock(     // 向问题中添加误差项
    	  // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
      	new ceres::AutoDiffCostFunction<CostFunction_cam_2d_2d, 1, 2>(
            new CostFunction_cam_2d_2d(point_1, point_2,
                                       this->cameras[reference], this->cameras[target]
                                       )
      	),
      	  nullptr,            // 核函数，这里不使用，为空
      	  depth              // 待估计参数
    	);
    // problem.SetParameterLowerBound(depth, 0, 0.0);
    // problem.SetParameterLowerBound(depth, 1, 0.0);

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
    // cout << summary.FullReport() << endl;

    return {depth[0], depth[1]};
}


void Associate::averageProcess(vector<Pose>& set_1, vector<Pose>& set_2, vector<pair<int, int> >& ass){
    if(set_1.empty() || set_2.empty()) return;

    bool avg = false;

    if(avg){
        for(auto it : ass){
            Pose pose = average(set_1[it.first], set_2[it.second]);
            this->poses_3d.push_back(pose);
        }
    }else{
        this->poses_3d = set_1;
    }
}

Pose Associate::average(const Pose& pose_1, const Pose& pose_2){
    if(pose_1.empty() || pose_2.empty()){
        ROS_ERROR("The Pose is empty in average process.");
        return {};
    }

    vector<Joint_3d> pose_3d_1 = pose_1.get3DPose();
    vector<Joint_3d> pose_3d_2 = pose_2.get3DPose();

    vector<Joint_2d> pose_2d_1 = pose_1.get2DPose();
    vector<Joint_2d> pose_2d_2 = pose_2.get2DPose();

    vector<Joint_3d> pose_3d;

    for(int i=0; i<pose_3d_1.size(); ++i){
        Joint_3d joint_3d;
        joint_3d.x = (pose_3d_1[i].x * pose_2d_1[i].p + pose_3d_2[i].x * pose_2d_2[i].p) / (pose_2d_1[i].p + pose_2d_2[i].p);
        joint_3d.y = (pose_3d_1[i].y * pose_2d_1[i].p + pose_3d_2[i].y * pose_2d_2[i].p) / (pose_2d_1[i].p + pose_2d_2[i].p);
        joint_3d.z = (pose_3d_1[i].z * pose_2d_1[i].p + pose_3d_2[i].z * pose_2d_2[i].p) / (pose_2d_1[i].p + pose_2d_2[i].p);
        joint_3d.available = true;

        pose_3d.push_back(joint_3d);
    }

    Pose new_pose;
    new_pose.set3DPose(pose_3d);

    return new_pose;
}


void Associate::transToWorldOfRoot(Pose &pose, const DataSetCamera& DC){

}

vector<Root_3d> Associate::transToWorldOfRoot(const vector<Root_3d> &root, const DataSetCamera& DC){
    if(root.empty()) return {};

    vector<Root_3d> _root;
    for(auto it : root){
        Root_3d r_;
        Eigen::Matrix<double, 3, 1> joint;
        joint << it.x, it.y, it.z;

        // cout << it.x << " " << it.y << " " << it.z << endl;
        joint = DC.R * joint + DC.t;

        r_.x = joint(0, 0);
        r_.y = joint(1, 0);
        r_.z = joint(2, 0);
        r_.p = it.p;
        r_.available = it.available;

        _root.push_back(r_);
    }

    return _root;
}

double Associate::getDistance(const Root_3d & root_1, const Root_3d & root_2){
    // cout << "root_1: " << root_1.x << " " << root_1.y << " " << root_1.z << endl;
    // cout << "root_2: " << root_2.x << " " << root_2.y << " " << root_2.z << endl;

    return root_1.p * root_2.p *
        std::sqrt((root_1.x - root_2.x) * (root_1.x - root_2.x)
                + (root_1.y - root_2.y) * (root_1.y - root_2.y)
                + (root_1.z - root_2.z) * (root_1.z - root_2.z));
}

double Associate::getScore(const Root_3d & root_1, const Root_3d & root_2){
    return root_1.p * root_2.p;
}


double Associate::lineToline(const vector<Root_3d> &line_1, const vector<Root_3d> &line_2){
    double res = 0.0;

    this->namada = 0.5;
    this->miu = 0.5;
    double dis_l;

    vector<double> l1 = {line_1[1].x - line_1[0].x, line_1[1].y - line_1[0].y, line_1[1].z - line_1[0].z};  //方向向量
    vector<double> l2 = {line_2[1].x - line_2[0].x, line_2[1].y - line_2[0].y, line_2[1].z - line_2[0].z};  //方向向量
    vector<double> v_ = {line_2[0].x - line_1[0].x, line_2[0].y - line_1[0].y, line_2[0].z - line_1[0].z}; // 点向量

    // 空间中的线与线的关系包含异面、相交、平行、重合。
    // 其中，异面和相交放在一起，平行和重合放在一起
    // 若相交或重合则返回0，若异面或平行则按照下面计算。
    // 首先，判断两条直线的关系
    vector<double> l1_ = {line_1[0].x, line_1[0].y, line_1[0].z, line_1[1].x, line_1[1].y, line_1[1].z};
    vector<double> l2_ = {line_2[0].x, line_2[0].y, line_2[0].z};

    if(isParallel(l1, l2)){
        // 平行
        ROS_INFO("Parallel.");
        dis_l = computeModel(crossMulti(v_, l2)) / computeModel(l2);
        // cout << dis_l << endl;
        if(dis_l < 1e-4){
            ROS_INFO("Cross or same.");
            return 0.0;
        }
    }else{
        // 异面
        ROS_INFO("Skew.");
        vector<double> n_ = crossMulti(l1, l2);
        dis_l = dot(v_, n_) / computeModel(n_);
    }


    double dis_p = 0.0, sco_p = 0.0;

    for(int i=0; i<2; ++i){
        dis_p += getDistance(line_1[i], line_2[i]) * getScore(line_1[i], line_2[i]);
        sco_p += getScore(line_1[i], line_2[i]);
    }

    res = (this->namada * dis_l + this->miu * dis_p) / (this->namada + this->miu * sco_p);
    return res;
}

double Associate::pointToline(const Root_3d &root, const vector<Root_3d> &line){
    double res = 0.0;

    this->namada = 0.8;
    this->miu = 0.2;
    double dis_lp;

    vector<double> l = {line[1].x - line[0].x, line[1].y - line[0].y, line[1].z - line[0].z};  //方向向量
    vector<double> v_ = {root.x - line[0].x, root.y - line[0].y, root.z - line[0].z}; // 点向量

    // 空间中的点与线的关系包含共面、重合。
    // 若重合返回0，若共面则按照下面计算。
    // 首先，判断点与线的关系
    if(isParallel(l, v_)){
        // 重合
        ROS_INFO("Cross.");
        return 0.0;
    }else{
        // 共面
        ROS_INFO("Coplaner.");
        double h_ = dot(v_, l) / computeModel(l);
        dis_lp = std::sqrt(computeModel(v_) * computeModel(v_) - h_ * h_);
    }


    double dis_p = 0.0, sco_p = 0.0;


    dis_p = getDistance(root, line[1]);
    sco_p = getScore(root, line[1]);


    res = (this->namada * dis_lp + this->miu * dis_p * sco_p) / (this->namada + this->miu * sco_p);
    return res;
}


double Associate::pointTopoint(const Root_3d &root_1, const Root_3d &root_2){
    ROS_INFO("PointTOPoint.");
    return getDistance(root_1, root_2) / getScore(root_1, root_2);
}


void Associate::triangularization(const vector<pair<int, int> > &pose_ass, const int reference, const int target, const Mode& method, bool inc){
    if(pose_ass.empty())
        return;

    for(auto it : pose_ass){
        if(inc){
            triangulatePose(it, reference, target, true);
        }else{
            triangulatePose(it, reference, target);
        }
    }
}


void Associate::triangulatePose(const pair<int, int> & it, const int reference, const int target, bool inc){
    vector<Joint_2d> pose_2d_1, pose_2d_2;
    
    if(inc){
        pose_2d_1 = this->pose_tmp[it.first].get2DPose();
    }else{
        pose_2d_1 = this->inputs[reference][it.first].get2DPose();
    }
    
    pose_2d_2 = this->inputs[target][it.second].get2DPose();

    vector<double> depths[2];
    for(int i=0; i<pose_2d_1.size(); ++i){
        // cout << "joint: " << i << "  x1: " << pose_2d_1[i].x << " y1: " << pose_2d_1[i].y
        //      << "    x2:" << pose_2d_2[i].x << " y2: " << pose_2d_2[i].y << endl;

        // 迭代初始值
        vector<double> depth = triangularPoints(pose_2d_1[i], pose_2d_2[i], reference, target);
        // if(depths.empty())
        //     continue;

        // cout << depth[0] << " " << depth[1] << endl;

        depths[0].push_back(depth[0]);
        depths[1].push_back(depth[1]);
    }

    if(inc){
        this->pose_tmp[it.first].update3DPose(depths[0], this->cameras[reference], false);
    }else{
        this->inputs[reference][it.first].update3DPose(depths[0], this->cameras[reference], false);
    }

    this->inputs[target][it.second].update3DPose(depths[1], this->cameras[target], false);
}

void Associate::triangulatePose(const pair<int, int> & it, const int target){
    vector<Joint_2d> pose_2d_1 = this->pose_tmp[it.first].get2DPose();
    vector<Joint_2d> pose_2d_2 = this->inputs[target][it.second].get2DPose();

    vector<double> depths[2];
    for(int i=0; i<pose_2d_1.size(); ++i){
        cout << "joint: " << i << "  x1: " << pose_2d_1[i].x << " y1: " << pose_2d_1[i].y
             << "    x2:" << pose_2d_2[i].x << " y2: " << pose_2d_2[i].y << endl;

        // 迭代初始值
        vector<double> depth = triangularPoints(pose_2d_1[i], pose_2d_2[i], reference, target);
        // if(depths.empty())
        //     continue;

        cout << depth[0] << " " << depth[1] << endl;

        depths[0].push_back(depth[0]);
        depths[1].push_back(depth[1]);
    }


    this->pose_tmp[it.first].update3DPose(depths[0], this->cameras[reference], false);
    this->inputs[target][it.second].update3DPose(depths[1], this->cameras[target], false);
}

void Associate::optimizer3DLoc(Pose &pose, const int reference, const int target){
    if(!pose.isUpdated()) return;

    // vector<Joint_3d> pose_3d = pose.get3DPose();
    // int count = 0;
    // for(int p = 0; p < pose_3d.size(); ++p) {
    //     double point[3] = {pose_3d[p].x, pose_3d[p].y, pose_3d[p].z};
    //     ceres::Problem problem;
    //
    //     double prob = pose_3d[p].p;
    //     //ROS_INFO("prob:%.2f", prob);
    //     if (prob > 0) {
    //
    //         count++;
    //         int cam = AvailablePose[as_id[i]].camera_index-1;
    //         problem.AddResidualBlock(     // 向问题中添加误差项
    //               // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
    //             new ceres::AutoDiffCostFunction<CostFunction_3d_loc, 1, 3>(
    //                 new CostFunction_3d_loc(camera_rot[cam], camera_trans[cam], cam_param[cam], AvailablePose[as_id[i]].pose_joints[p])
    //             ),
    //             nullptr,            // 核函数，这里不使用，为空
    //             point               // 待估计参数
    //             );
    //     }
    //
    //     if(count >= 2) {
    //         ceres::Solver::Summary summary;
    // 		ceres::Solve(options, &problem, &summary);
    //         //cout << summary.BriefReport() << endl;
    //
    //         pose_joints_3d[p].available = true;
    //         pose_joints_3d[p].x = point[0];
    //         pose_joints_3d[p].y = point[1];
    //         pose_joints_3d[p].z = point[2];
    //     }
    //     else {
    //         //ROS_INFO("no enough angle");
    //         pose_joints_3d[p].available = false;
    //     }
    // }
}

vector<double> Associate::triangularPoints(const Joint_2d& joint_1, const Joint_2d& joint_2, const int reference, const int target){

    // 对关节点进行归一化
    Eigen::Matrix<double, 3, 1> nor_joint_1 = normalization(joint_1, reference);
    Eigen::Matrix<double, 3, 1> nor_joint_2 = normalization(joint_2, target);

    vector<double> depths = {0.0, 0.0};

    if(joint_1.p < 1.0 || joint_2.p < 1.0)
        return depths;

    Eigen::Matrix<double, 3, 2> A;
    A << nor_joint_2, this->R * nor_joint_1;

    const Eigen::Matrix2d AtA = A.transpose() * A;
    if(AtA.determinant() < 0.000001)
        return {};

    const Eigen::Vector2d depth =
        -AtA.inverse()* A.transpose() * this->t;

    depths[0] = fabs(depth[0]);
    depths[1] = fabs(depth[1]);

    return depths;
}

Eigen::Matrix<double, 3, 1> Associate::normalization(const Joint_2d& joint, const int ref){
    Eigen::Matrix<double, 3, 1> res;

    cv::Point2d c_p = pixel2cam(joint, this->cameras[ref]);

    res(0, 0) = c_p.x;
    res(1, 0) = c_p.y;
    res(2, 0) = 1.0;

    return res;
}


void Associate::transToReference(vector<Pose> &pose, int reference){

}


void Associate::triangulatePointsWithOpenCV(vector<Joint_2d>& set_1, vector<Joint_2d>& set_2, DataSetCamera& DC1, DataSetCamera& DC2, vector<cv::Point3d>& points){

}

cv::Point2d Associate::pixel2cam(const Joint_2d& p, const DataSetCamera& DC){
    return cv::Point2d(
        (p.x - DC.cx) / DC.fx,
        (p.y - DC.cy) / DC.fy
    );

    return cv::Point2f(0.0, 0.0);
}

void Associate::getPoseResult(int reference){
    this->poses_3d = this->inputs[reference];
}


template <typename T>
T Associate::computeModel(const vector<T>& v){
    assert(!v.empty() && v.size() == 3);
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

template <typename T>
T Associate::dot(const vector<T>& v1, const vector<T>& v2){
    assert(!v1.empty() && !v2.empty() && v1.size() == v2.size() == 3);

    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}


template <typename T>
vector<T> Associate::crossMulti(const vector<T>& v1, const vector<T>& v2){
    assert(!v1.empty() && !v2.empty() && v1.size() == v2.size() == 3);

    vector<T> res;

    res.push_back(v1[1] * v2[2] - v1[2] * v2[1]);
    res.push_back(v1[2] * v2[0] - v1[0] * v2[2]);
    res.push_back(v1[0] * v2[1] - v1[1] * v2[0]);

    return res;
}

template <typename T>
T Associate::det(const vector<T>& v1, const vector<T>& v2){
    return v1[0]*v2[4]*v2[2]+v1[1]*v1[2]*v1[3]+v1[3]*v2[1]*v1[3]-v1[2]*v1[4]*v2[0]
            -v1[1]*v1[3]*v2[2]-v1[0]*v2[1]*v1[5];
}

template <typename T>
bool Associate::isParallel(const vector<T>& v1, const vector<T>& v2){
    return (v1[0] / v2[0] - v1[1] / v2[1]) < 1e-4
            && (v1[0] / v2[0] - v1[2] / v2[2]) < 1e-4
            && (v1[1] / v2[1] - v1[2] / v2[2]) < 1e-4;
}
