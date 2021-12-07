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


void Associate::run(int reference, int target, bool single, Mode& mode){
    this->reference = reference;
    this->target = target;

    if(single){

    }
    else{
        if(mode == Mode.triangulation || mode == Mode.OpenCV){
            triangularCamera(this->reference, this->target);
        } else if(mode == Mode.Ceres){
            // do nothing
        }else{
            ROS_ERROR("Mode error.");
        }

        generatePair(this->inputs[reference], this->inputs[target]);
        fusionOfIncremental(this->reference, this->target);
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

void Associate::triangularCamera(int reference, int target){
    // 这里使用ROS框架下的tf
    tf::Transform transform;

    tf_listener->waitForTransform("Camera_" + to_string(reference),
                                  "Camera_" + to_string(target),
                                  ros::Time(0),
                                  ros::Duration(1.0));
    tf_listener->lookupTransform("Camera_" + to_string(reference),
                                 "Camera_" + to_string(target),
                                 ros::Time(0),
                                 transform);

    tf::Quaternion q_f = transform.getRotation();
    tf::Vector3 trans_f = transform.getOrigin();

    Eigen::Quaterniond q(q_f.getW(), q_f.getX(), q_f.getY(), q_f.getZ());
    Eigen::Vector3d trans(trans_f.getX(), trans_f.getY(), trans_f.getZ());

    this->R = q.toRotationMatrix();
    this->t = trans;
}


void Associate::fusionOfIncremental(int reference, int target){
    if(reference == target)
        ROS_ERROR("The number of fusion frame is wrong.");

    fusionOfEachFrame(this->inputs[reference], this->inputs[target]);
}


void Associate::fusionOfEachFrame(vector<Pose>& pose_1, vector<Pose>& pose_2){
    if(this->pose_pairs.empty())
        return;

    for(PosePair it : this->pose_pairs){
        calculateCorrespondence(*it);
    }

    auto rank_min = min_element(pose_pairs.begin(), pose_pairs.end(), PosePair::comp);
    int label_ite = 1;
    double threshold = 0.4;

    while(!pose_pairs.empty() && rank_min->delta <= threshold){
        int label_1_old = (*rank_min).getLabel_1();
        int label_2_old = (*rank_min).getLabel_2();
        int id_1 = (*rank_min).getIndex_1();
        int id_2 = (*rank_min).getIndex_2();

        this->inputs[reference][id_1].setLabel(label_ite);
        this->inputs[target][id_2].setLabel(label_ite);

        pose_pairs.erase(rank_min);
    }
}


bool Associate::calculateCorrespondence(PosePair &pair, const double & threshold){
    return false;
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

vector<pair<int, int> > Associate::extract2DAssociation(){
    return {};
}


void Associate::triangularization(const vector<pair<int, int> > &pose_ass, const int reference, const int target, int method){

}

vector<double> Associate::triangularPoints(const Joint_2d& joint_1, const Joint_2d& joint_2){
    return {};
}

vector<double> Associate::OptimizerWithCereSolver(const Joint_2d point_1, const Joint_2d point_2, const int reference, const int target){
    return {};
}

void Associate::transToReference(vector<Pose> &pose, int reference){

}

void Associate::averageProcess(vector<Pose>& set_1, vector<Pose>& set_2, pair<int, int>& ass){

}

vector<Joint_3d> Associate::average(const Pose& pose_1, const Pose& pose_2){
    return {};
}

void Associate::triangulatePointsWithOpenCV(vector<Joint_2d>& set_1, vector<Joint_2d>& set_2, DataSetCamera& DC1, DataSetCamera& DC2, vector<cv::Point3d>& points){

}

cv::Point2f Associate::pixel2cam(const Joint_2d& p, const cv::Mat& K){
    return cv::Point2f(0.0, 0.0);
}

void Associate::getPoseResult(int reference){

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
