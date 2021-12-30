/*
 * @Author: your name
 * @Date: 2021-08-13 19:24:06
 * @LastEditTime: 2021-12-06 21:18:32
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /multi_human_estimation/src/pose.cpp
 */
#include <multi_human_estimation/pose.h>

Pose::Pose():camera_id(-1), label(-1), updated(false), num_kpt(15){
    pose_3d.resize(num_kpt);
    pose_2d.resize(num_kpt);
    root_3d.resize(2);

    for(int i=0; i<num_kpt; ++i){
        pose_3d[i].x = 0.0;
        pose_3d[i].y = 0.0;
        pose_3d[i].z = 0.0;
        pose_3d[i].available = false;
    }

    for(int i=0; i<num_kpt; ++i){
        pose_2d[i].x = 0.0;
        pose_2d[i].y = 0.0;
        pose_2d[i].p = 0.0;
    }

    for(int i=0; i<2; ++i){
        root_3d[i].x = 0.0;
        root_3d[i].y = 0.0;
        root_3d[i].z = 0.0;
        root_3d[i].p = 0.0;
        root_3d[i].available = false;
    }
}

Pose::~Pose(){

}

void Pose::setLabel(const int _label){
    this->label = _label;
}

void Pose::setCameraID(int _camera_id){
    this->camera_id = _camera_id;
}

void Pose::set2DPose(vector<double>& joints){
    if(joints.empty() || joints.size() % 3 != 0) return;

    // pose_2d.clear();
    // 关节方法系数
    // cout << "alpha: " << alpha << endl;
    // cout << "belta: " << belta << endl;

    for(int i=0; i<joints.size() / 3; ++i){
        // Joint_2d joint2d;
        pose_2d[i].x = joints[3*i] * alpha;
        pose_2d[i].y = joints[3*i + 1] * belta;
        pose_2d[i].p = joints[3*i + 2];
        // pose_2d.push_back(joint2d);
    }
}


void Pose::setRootPose(vector<double> &root){
    if(root.empty() || root.size() % 4 != 0) return;

    // root_3d.clear();
    assert(root.size() == 8);

    for(int i=0; i<root.size() / 4; ++i){
        // Root_3d root3d;
        root_3d[i].x = root[4*i] / 100.0;
        root_3d[i].y = root[4*i + 1] / 100.0;
        root_3d[i].z = root[4*i + 2] / 100.0;
        root_3d[i].p = root[4*i + 3] / 3.0;

        if (root_3d[i].p > 0.4)
            root_3d[i].available = true;
        else
            root_3d[i].available = false;
        // root_3d.push_back(root3d);
    }
}

void Pose::update3DPose(vector<Joint_3d> &joint3d, DataSetCamera &DC, bool worldOrCamera){
    if(joint3d.empty()) return;

    assert(joint3d.size() == 15);
    this->pose_3d = joint3d;

    updated = true;

    if(worldOrCamera){
        for(int i=0; i<pose_3d.size(); ++i){
            Eigen::Matrix<double, 3, 1> tmp =
                DC.R * Eigen::Matrix<double, 3, 1>(pose_3d[i].x, pose_3d[i].y, pose_3d[i].z) + DC.t/100;
            pose_3d[i].x = tmp(0, 0);
            pose_3d[i].y = tmp(1, 0);
            pose_3d[i].z = tmp(2, 0);
        }
    }
}

void Pose::update3DPose(vector<double> &depth, DataSetCamera &DC, bool worldOrCamera){
    if(depth.empty()) return;

    // pose_3d.resize(pose_2d.size());

    for(int i=0; i<num_kpt; ++i){
        if(depth[i] < 1e-3) continue;
        pose_3d[i].z = depth[i];
        pose_3d[i].x = ( pose_2d[i].x - DC.cx ) * pose_3d[i].z / DC.fx;
        pose_3d[i].y = ( pose_2d[i].y - DC.cy ) * pose_3d[i].z / DC.fy;
    }

    updated = true;

    if(worldOrCamera){
        for(int i=0; i<num_kpt; ++i){
            Eigen::Matrix<double, 3, 1> tmp =
                DC.R * Eigen::Matrix<double, 3, 1>(pose_3d[i].x, pose_3d[i].y, pose_3d[i].z) + DC.t/100;
            pose_3d[i].x = tmp(0, 0);
            pose_3d[i].y = tmp(1, 0);
            pose_3d[i].z = tmp(2, 0);
        }
    }
}

void Pose::update3DPose(vector<cv::Point3d> &points3d, DataSetCamera &DC, bool worldOrCamera){
    if(points3d.empty()) return;

    // pose_3d.resize(pose_2d.size());

    // this->pose_3d.clear();

    for(int i=0; i<points3d.size(); ++i){
        // Joint_3d joint_3d;
        pose_3d[i].x = points3d[i].x;
        pose_3d[i].y = points3d[i].y;
        pose_3d[i].z = points3d[i].z;
        pose_3d[i].available = true;

        // this->pose_3d.push_back(joint_3d);
    }

    updated = true;

    cout << "In Pose.cpp" << endl;
    for(auto it:this->pose_3d){
        cout << "X: " << it.x <<
                "  Y: " << it.y <<
                "  Z: " << it.z << endl;
    }

    if(worldOrCamera){
        for(int i=0; i < num_kpt; ++i){
            Eigen::Matrix<double, 3, 1> cam_joint;
            cam_joint << pose_3d[i].x, pose_3d[i].y, pose_3d[i].z;

            Eigen::Matrix<double, 3, 1> tmp = DC.R.transpose() * (cam_joint - DC.t/100);
            pose_3d[i].x = tmp(0, 0);
            pose_3d[i].y = tmp(1, 0);
            pose_3d[i].z = tmp(2, 0);
        }
    }

    cout << "In World" << endl;
    for(auto it:this->pose_3d){
        cout << "X: " << it.x <<
                "  Y: " << it.y <<
                "  Z: " << it.z << endl;
    }

}

vector<Joint_2d> Pose::pixel2cam(DataSetCamera &DC){
    // if(this->pose_2d.empty()) return {};

    vector<Joint_2d> tmp;
    for(auto it:this->pose_2d){
        if(it.p >= 1.0){
            Joint_2d joint;
            joint.x = (it.x - DC.cx) / DC.fx;
            joint.y = (it.y - DC.cy) / DC.fy;

            tmp.push_back(joint);
        }
    }

    return tmp;
}


void Pose::set3DPose(vector<Eigen::Matrix<double, 3, 1> >& pose_3d){
    if(pose_3d.empty()) return;
    // vector<Joint_3d> pose;

    assert(pose_3d.size() == num_kpt);

    int i = 0;
    for(auto it : pose_3d){
        if(it(0, 0) - 0 < 1e-4 && it(1, 0) - 0 < 1e-4 && it(2, 0) - 0 < 1e-4)
            continue;

        // Joint_3d joint;
        this->pose_3d[i].x = it(0, 0);
        this->pose_3d[i].y = it(1, 0);
        this->pose_3d[i].z = it(2, 0);
        this->pose_3d[i].available = true;

        // pose.push_back(joint);
        ++i;
    }

    // this->pose_3d = pose;
}

void Pose::set3DPose(const vector<Joint_3d>& pose_3d){
    if(pose_3d.empty()) return;

    // this->pose_3d = pose_3d;
    for(int i=0; i<pose_3d.size(); ++i){
        this->pose_3d[i].x = pose_3d[i].x;
        this->pose_3d[i].y = pose_3d[i].y;
        this->pose_3d[i].z = pose_3d[i].z;
        this->pose_3d[i].available = pose_3d[i].available;
    }

    updated = true;
}

void Pose::set3DPose(const vector<cv::Point3d>& pose_3d){
    if(pose_3d.empty()) return;

    assert(pose_3d.size() == num_kpt);
    for(int i=0; i<pose_3d.size(); ++i){
        // Joint_3d joint;
        this->pose_3d[i].x = pose_3d[i].x;
        this->pose_3d[i].y = pose_3d[i].y;
        this->pose_3d[i].z = pose_3d[i].z;
        this->pose_3d[i].available = true;

        // this->pose_3d.push_back(joint);
    }
}
