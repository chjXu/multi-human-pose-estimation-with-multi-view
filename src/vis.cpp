/*
 * @Description:  visulization
 * @Author: chengjun_xu
 * @Data: Do not edit
 * @LastAuthor: Do not edit
 * @LastEditTime: 2021-12-06 20:12:37
 */
#include "multi_human_estimation/vis.h"
#include <typeinfo>

static cv::Scalar randomColor(cv::RNG &rng){
    int icolor = (unsigned)rng;
    return cv::Scalar(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);
}

vector<vector<int>> Vis::body_edges = {
                   {0, 1}, // head
                   {0, 2}, {0, 3}, {0, 9}, // torso
                   {3, 4}, {4, 5}, // right_hand
                   {9, 10}, {10, 11}, // left hand
                   {2, 12},{12, 13}, {13, 14}, // left leg
                   {2, 6},{6, 7}, {7, 8} // right leg
                   };

Vis::Vis(ros::Publisher &pub):person_nums(10){
    cv::RNG rng(0xFFFFFFFF);
    this->pose_pub = &pub;

    for(int i=0; i<person_nums; ++i){
        colors.push_back(randomColor(rng));
    }
}

Vis::~Vis(){

}


void Vis::transRootJoint(const vector<Pose> &root_joint_3d, const DataSetCamera& DC){
    if(root_joint_3d.empty()) return;

    this->rootPixelLocation.clear();

    for(unsigned int i=0; i<root_joint_3d.size(); ++i){
        pixel_location root_2d;

        vector<pixel_location> roots_2d;
        vector<Root_3d> root_3d = root_joint_3d[i].getRootPose();
        assert(root_3d.size() <= 2);

        for(unsigned int j=0; j<root_3d.size(); ++j){
            // 转换到像素坐标系下
            if(root_3d[j].z == 0) continue;
            root_2d.x = (int)(root_3d[j].x * DC.fx / root_3d[j].z + DC.cx);
            root_2d.y = (int)(root_3d[j].y * DC.fy / root_3d[j].z + DC.cy);

            roots_2d.push_back(root_2d);
        }

        this->rootPixelLocation.push_back(roots_2d);
    }
}

void Vis::showRootJoint(){
    if(this->rootPixelLocation.empty()) return;

    int person_num = rootPixelLocation.size();
    for(int person = 0; person < person_num; ++person){
        for(int joint=0; joint < rootPixelLocation[person].size(); ++joint){
            //cout << rootPixelLocation[person][joint].x << " " << rootPixelLocation[person][joint].y << endl;

            // cv::circle(this->image, cv::Point2d(rootPixelLocation[person][joint].x, rootPixelLocation[person][joint].y),
            //            4, colors[person], 6);
            drawPoint(rootPixelLocation[person][joint], person);
        }
        drawLine(rootPixelLocation[person][0], rootPixelLocation[person][1], person);
    }
}

void Vis::addImage(const cv::Mat &img){
    this->image = img;
}

void Vis::drawLine(const pixel_location &a, const pixel_location &b, const int person_index){
    if(a.x != 0 && a.y != 0 && b.x != 0 && b.y != 0)
        cv::line(this->image, cv::Point2d(a.x, a.y), cv::Point2d(b.x, b.y), colors[person_index], 4);
}

void Vis::drawPoint(const pixel_location &a, const int person_index){
    cv::circle(this->image, cv::Point2d(a.x, a.y), 4, colors[person_index], 6);
}

void Vis::drawPoint(const vector<Pose> &poses){
    if(poses.empty()) return;
    int len = poses.size();

    for(int i=0; i<len; ++i){
        vector<Joint_2d> joints_2d = poses[i].get2DPose();
        for(int j=0; j<joints_2d.size(); ++j){
            drawPoint(pixel_location(joints_2d[j].x, joints_2d[j].y), i);
        }
    }
}

void Vis::showImage(int camera_index){
    if(!this->image.empty()){
        cv::imshow("Camera" + to_string(camera_index), this->image);
        cv::waitKey(0);
    }
}


Eigen::Matrix<double, 3, 1> Vis::cameraToWorld(const Eigen::Matrix<double, 3, 1>& joint, const DataSetCamera& DC){
    return DC.R.transpose() * (joint - DC.t);
}

void Vis::transToWorld(const vector<Pose> &skeletons, const DataSetCamera& DC){
    if(skeletons.empty()) return;

    this->new_poses.clear();

    for(auto is : skeletons){
        Pose new_pose;

        vector<Eigen::Matrix<double, 3, 1> > joints_3d;

        vector<Joint_3d> pose = is.get3DPose();
        for(int ip = 0; ip < pose.size(); ++ip){
            Eigen::Matrix<double, 3, 1> joint_camera;
            joint_camera << pose[ip].x, pose[ip].y, pose[ip].z;

            Eigen::Matrix<double, 3, 1> joint_world = DC.R.transpose() * (joint_camera - DC.t);

            joints_3d.push_back(joint_world);

        }
        new_pose.set3DPose(joints_3d);
        this->new_poses.push_back(new_pose);
    }
}

void Vis::drawSkeletons(const vector<Pose> &skeletons, const DataSetCamera& DC){
    if(skeletons.empty()) return;

    // transToWorld(skeletons, DC);

    while(ros::ok()){
        for(int i = 0; i < skeletons.size(); ++i) {
            printInfo(skeletons);
            visualization_msgs::Marker line_list;
            line_list.id = i;
            line_list.type = visualization_msgs::Marker::LINE_LIST;

            line_list.header.frame_id = "world";
            line_list.header.stamp = ros::Time::now();
            line_list.ns = "humans";
            line_list.action = visualization_msgs::Marker::ADD;
            line_list.pose.orientation.w = 1.0;

            line_list.scale.x = 0.01;
            line_list.scale.y = 0.01;
            line_list.scale.z = 0.01;
            line_list.color.r = 1.0;
            line_list.color.a = 1.0;

            vector<Joint_3d> pub_pose = skeletons[i].get3DPose();
            for(int edge = 0; edge < this->body_edges.size(); ++edge){
                drawLine(line_list, pub_pose, body_edges[edge][0], body_edges[edge][1]);
            }

            cout << line_list.points.size() << endl;
            pose_pub->publish(line_list);
            ROS_INFO("Info has published.");
        }
    }

}


void Vis::drawLine(visualization_msgs::Marker& line_list, const vector<Joint_3d>& pose_3d, int a, int b){
    // cout << pose_3d[a].available  << " " << pose_3d[b].available << endl;
    if(pose_3d[a].available && pose_3d[b].available) {
        geometry_msgs::Point p_a,p_b;
        p_a.x = pose_3d[a].x;
        p_a.y = pose_3d[a].y;
        p_a.z = pose_3d[a].z;

        p_b.x = pose_3d[b].x;
        p_b.y = pose_3d[b].y;
        p_b.z = pose_3d[b].z;

        line_list.points.push_back(p_a);
        line_list.points.push_back(p_b);
    }
}

void Vis::printInfo(const vector<Pose>& pose){
    if(pose.empty())
        cout << "Empty!" << endl;

    int size = pose.size();

    int id = 0;
    for(auto it : pose){
        cout << "The pose of " << id << " and its label is: " << it.getLabel() << endl;
        ++id;
    }

    for(auto it:pose){
        cout << "The 3D pose of Camera " << it.getCameraID() << " is :" << size << "\n";
        vector<Joint_3d> pose_3d = it.get3DPose();
        for(int i=0; i<pose_3d.size(); ++i){
            cout << "x: " << pose_3d[i].x << " y: " << pose_3d[i].y << " z: " << pose_3d[i].z << endl;
        }
    }
}
