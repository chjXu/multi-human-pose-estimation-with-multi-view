#include "multi_human_estimation/associate.h"

int Associate::index = 0;

Associate::Associate():reference(0), target(0){
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
    cout << "It's label is: " << pose.getLabel() << "\n";

    vector<Joint_3d> pose_3d = pose.get3DPose();
    int id = 0;
    for(auto it : pose_3d){
        if(it.available)
            cout << "Joint: " << id << "X: " << it.x << "Y: " << it.y << "Z: " << it.z << endl;

        ++id;
    }
}


void Associate::printInfo(const vector<Pose> &poses){
    if(poses.empty()){
        ROS_ERROR("Poses are empty()");
        return;
    }

    ROS_INFO("Cout all poses info within a image.");
    cout << "The Pose is under camera: " << poses[0].getCameraID() << " and ";

    int pose_id = 0;
    for(Pose it : poses){
        printPoseInfo(it);
        ++pose_id;
    }
}


void Associate::run(int reference, int target, bool single, const Mode&& mode){
    this->reference = reference;
    this->target = target;

    if(single){

    }
    else{
        triangularCamera(this->reference, this->target, mode);
        generatePair(this->inputs[reference], this->inputs[target]);
        fusionOfIncremental(this->reference, this->target);
        vector<pair<int, int> > poses_ass = extract2DAssociation();
        calcualte3DPose(poses_ass, mode);
        getPoseResult(this->reference);
    }
}


void Associate::addPoseInfo(const vector<Pose> &framePoses){
    if(framePoses.empty())
        ROS_ERROR("The pose inputted is empty.");

    this->inputs.push_back(framePoses);
}


void Associate::addCameraInfo(const DataSetCamera &DC, int camera_id){
    if(DC.valid()){
        ROS_ERROR("Camera %d data is invalid.", camera_id);
        return;
    }

    this->cameras.push_back(DC);
}


void Associate::generatePair(vector<Pose>& pose_1, vector<Pose>& pose_2){
    int group = 0;

    if(pose_1.empty() && pose_2.empty()){
        ROS_ERROR("There are 0 humans in Camera %d and Camera %d.", this->reference, this->target);
        return;
    }

    if(pose_1.empty()){
        ROS_ERROR("There are 0 humans in Camera %d.", this->reference);
        return;
    }

    if(pose_2.empty()){
        ROS_ERROR("There are 0 humans in Camera %d.", this->target);
        return;
    }

    this->pose_pairs.clear();

    for(int i=0; i<pose_1.size(); ++i){
        for(int j=0; j<pose_2.size(); ++j){
            PosePair pair;
            pair.setGroup(group);
            pair.setLabel(pose_1[i].getLabel(), pose_2[j].getLabel());
            pair.setIndex(i, j);
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
        // do nothing now.
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
    }else{
        ROS_ERROR("Mode error.");
    }

    if(listening)
        ROS_INFO("The transformation of Camera %d and Camera %d are listened.", reference, target);
    else
        ROS_ERROR("The transformation of Camera %d and Camera %d are not listened.", reference, target);
}


void Associate::fusionOfIncremental(int reference, int target){
    if(reference == target)
        ROS_ERROR("The number of fusion frame is wrong.");

    fusionOfEachFrame(this->inputs[reference], this->inputs[target]);
}


void Associate::fusionOfEachFrame(vector<Pose>& pose_1, vector<Pose>& pose_2){
    if(this->pose_pairs.empty())
        return;

    for(auto it = pose_pairs.begin(); it != pose_pairs.end(); ++it){
        calculateCorrespondence(*it);
    }

    int count = 1;
    for(auto it = pose_pairs.begin(); it != pose_pairs.end(); ++it){
        ROS_INFO("Pair: %d, and index_1: %d, index_2: %d, label_1: %d, label_2: %d, score is: %f.",
                  count, (*it).getIndex_1(), (*it).getIndex_2(), (*it).getLabel_1(), (*it).getLabel_2(),
                  (*it).getDelta());

        ++count;
    }

    auto rank_min = min_element(pose_pairs.begin(), pose_pairs.end(), PosePair::comp);
    int label_ite = 1;
    double threshold = 0.4;

    while(!pose_pairs.empty() && rank_min->getDelta() <= threshold){
        rank_min = min_element(pose_pairs.begin(), pose_pairs.end(), PosePair::comp);

        int label_1_old = (*rank_min).getLabel_1();
        int label_2_old = (*rank_min).getLabel_2();
        int id_1 = (*rank_min).getIndex_1();
        int id_2 = (*rank_min).getIndex_2();

        ROS_INFO("The pose %d of camera %d, and the pose %d of camera %d will be paired.",
                this->reference, id_1, id_2, this->target);

        this->inputs[reference][id_1].setLabel(label_ite);
        this->inputs[target][id_2].setLabel(label_ite);

        pose_pairs.erase(rank_min);

        for(auto it = this->pose_pairs.begin(); it != this->pose_pairs.end();){
            if((*it).getLabel_1() == label_1_old || (*it).getLabel_2() == label_2_old ||
               (*it).getLabel_1() == label_2_old || (*it).getLabel_2() == label_1_old){
                it = pose_pairs.erase(it);
            }
        }

        ++label_ite;
    }

    ROS_INFO("Finish pairing.");
}


bool Associate::calculateCorrespondence(PosePair &pair, const double & threshold){
    pair.setDelta(0.1);

    if(pair.getDelta() <= threshold)
        return true;
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
            ROS_INFO("Start with ceres solver.");
            for(int i=0; i<pose_1.size(); ++i){
                cout << "Joint: " << i << " pixel location: " << pose_1[i].x << " " << pose_1[i].y << "  "
                                                             << pose_2[i].x << " " << pose_2[i].y << endl;
                vector<double> depth = OptimizerWithCereSolver(pose_1[i], pose_2[i], this->reference, this->target);

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


vector<double> Associate::OptimizerWithCereSolver(const Joint_2d& point_1, const Joint_2d& point_2, const int reference, const int target){
    double depth[2] = {};

	ceres::Problem problem;
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

    for(auto it : ass){
        vector<Joint_3d> joint_3d = average(set_1[it.first], set_2[it.second]);

        this->inputs[this->reference][it.first].update3DPose(joint_3d, this->cameras[this->reference], false);
    }
}

vector<Joint_3d> Associate::average(const Pose& pose_1, const Pose& pose_2){
    if(pose_1.empty() || pose_2.empty()){
        ROS_ERROR("The Pose is empty in average process.");
        return {};
    }

    vector<Joint_3d> pose_3d_1 = pose_1.get3DPose();
    vector<Joint_3d> pose_3d_2 = pose_2.get3DPose();

    vector<Joint_2d> pose_2d_1 = pose_1.get2DPose();
    vector<Joint_2d> pose_2d_2 = pose_2.get2DPose();

    vector<Joint_3d> new_pose;

    for(int i=0; i<pose_3d_1.size(); ++i){
        Joint_3d joint_3d;
        joint_3d.x = (pose_3d_1[i].x * pose_2d_1[i].p + pose_3d_2[i].x * pose_2d_2[i].p) / (pose_2d_1[i].p + pose_2d_2[i].p);
        joint_3d.y = (pose_3d_1[i].y * pose_2d_1[i].p + pose_3d_2[i].y * pose_2d_2[i].p) / (pose_2d_1[i].p + pose_2d_2[i].p);
        joint_3d.z = (pose_3d_1[i].z * pose_2d_1[i].p + pose_3d_2[i].z * pose_2d_2[i].p) / (pose_2d_1[i].p + pose_2d_2[i].p);
        joint_3d.available = true;

        new_pose.push_back(joint_3d);
    }

    return new_pose;
}


void Associate::transToWorld(Pose &pose, const DataSetCamera& DC){

}


double Associate::lineToline(const vector<Root_3d> &line_1, const vector<Root_3d> &line_2){
    return 0.0;
}

double Associate::pointToline(const Root_3d &root, const vector<Root_3d> &line){
    return 0.0;
}


double Associate::pointTopoint(const Root_3d &root_1, const Root_3d &root_2){
    return 0.0;
}


void Associate::triangularization(const vector<pair<int, int> > &pose_ass, const int reference, const int target, int method){

}

vector<double> Associate::triangularPoints(const Joint_2d& joint_1, const Joint_2d& joint_2){
    return {};
}

void Associate::transToReference(vector<Pose> &pose, int reference){

}


void Associate::triangulatePointsWithOpenCV(vector<Joint_2d>& set_1, vector<Joint_2d>& set_2, DataSetCamera& DC1, DataSetCamera& DC2, vector<cv::Point3d>& points){

}

cv::Point2f Associate::pixel2cam(const Joint_2d& p, const cv::Mat& K){
    return cv::Point2f(0.0, 0.0);
}

void Associate::getPoseResult(int reference){
    this->poses_3d = this->inputs[reference];
}


template <typename T>
T computeModel(vector<T>& v){
    return 0.0;
}

template <typename T>
T dot(vector<T>& v1, vector<T>& v2){
    return 0.0;
}


template <typename T>
vector<T> crossMulti(vector<T>& v1, vector<T>& v2){
    return 0.0;
}
