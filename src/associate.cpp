#include "multi_human_estimation/associate.h"

int Assoclate::index = 0;

Associate::Associate(){
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


void Associate::run(int reference, int target, bool single){
    if(single){

    }
    else{
        triangularCamera(reference, target);

    }
}


void Associate::addPoseInfo(const vector<Pose> &framePoses){
    if(framePoses.empty())
        ROS_ERROR("The pose inputted is empty.")

    this->inputs.push_back(framePoses);
}


void Associate::addCameraInfo(const DataSetCamera &DC, int camera_id){
    if(DC.invalid()){
        ROS_ERROR("Camera %d data is invalid.", camera_id);
        return;
    }

    this->cameras.push_back(DC);
}


void Associate::generatePair(vector<Pose>& pose_1, vector<Pose>& pose_2){
    int group = 0;

    if(pose_1.empty() && pose_2.empty()){
        ROS_ERROR("There are 0 humans in Camera %d and Camera %d.", pose_1.getCameraID(), pose_2.getCameraID());
        return;
    }

    if(pose_1.empty()){
        ROS_ERROR("There are 0 humans in Camera %d.", pose_1.getCameraID());
        return;
    }

    if(pose_2.empty()){
        ROS_ERROR("There are 0 humans in Camera %d.", pose_2.getCameraID());
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

            this->pose_pairs.push_banck(pair);
        }
    }

    ROS_INFO("Total pose pairs : %d.", this->pose_pairs.size());
    ++group;
}


void Associate::fusionOfFirstFrame(vector<Pose>& pose_1, vector<Pose>& pose_2){

}


void Associate::fusionOfIncremental(vector<Joint_3d>& pose_1, vector<Pose>& pose_2){

}

bool Associate::calculateCorrespondence(PosePair &pair, const double & threshold){

}

void Associate::transToWorld(Pose &pose, const DataSetCamera& DC){

}


double Associate::lineToline(const vector<Root_3d> &line_1, const vector<Root_3d> &line_2){

}

double Associate::pointToline(const Root_3d &root, const vector<Root_3d> &line){

}


double Associate::pointTopoint(const Root_3d &root_1, const Root_3d &root_2){

}

vector<pair<int, int> > Associate::extract2DAssociation(){

}


void Associate::triangularization(const vector<pair<int, int> > &pose_ass, const int reference, const int target, int method){

}

vector<double> Associate::triangularPoints(const Joint_2d& joint_1, const Joint_2d& joint_2){

}

void Associate::triangularCamera(const int reference, const int target){

}

vector<double> Associate::OptimizerWithCereSolver(const Joint_2d point_1, const Joint_2d point_2, const int reference, const int target){

}

void Associate::transToReference(vector<Pose> &pose, int reference){

}

void Associate::averageProcess(vector<Pose>& set_1, vector<Pose>& set_2, pair<int, int>& ass){

}

vector<Joint_3d> Associate::average(const Pose& pose_1, const Pose& pose_2){

}

void Associate::triangulatePointsWithOpenCV(vector<Joint_2d>& set_1, vector<Joint_2d>& set_2, DataSetCamera& DC1, DataSetCamera& DC2, vector<cv::Point3d>& points){

}

cv::Point2f Associate::pixel2cam(const Joint_2d& p, const cv::Mat& K){

}

void Associate::getPoseResult(int reference){

}


template <typename T>
T computeModel(vector<T>& v){

}

template <typename T>
T dot(vector<T>& v1, vector<T>& v2){

}


template <typename T>
vector<T> crossMulti(vector<T>& v1, vector<T>& v2){

}
