/*
 * @Description:  this is a cpp file to tset the transformation of the cameras of the CMU dataset
 * @Author: chengjun_xu
 * @Data: Do not edit
 * @LastAuthor: Do not edit
 * @LastEditTime: 2021-12-02 18:47:43
 */


#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <jsoncpp/json/json.h>
#include <sys/types.h>
#include <dirent.h>
#include <fstream>
#include <sstream>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace std;


struct CamParams
{
    Eigen::Matrix3d R;
    Eigen::Matrix<double, 3, 1> t;
    int id;
};



class Camera{
public:
    Camera(){
        br = new tf::TransformBroadcaster;
    }

    ~Camera(){
        delete br;
    }

    void run(string _calibrate_file_path){
        this->calibrate_file_path = _calibrate_file_path;

        readJSON();

        publishTFInfo();

        // ListenerInfo(0, 1);
    }

protected:
    // 读取JSON文件
    void readJSON();

    // 转化TF信息
    void transTFInfo(Json::Value& root);

    //Eigen::Quaterniond transRotation(Eigen::Matrix3d& R);

    tf::Matrix3x3 transRotation(Eigen::Matrix3d& R);

    tf::Vector3 transTranslation(Eigen::Matrix<double, 3, 1>& t);

    // 发布TF
    void publishTFInfo();

    // 打印信息
    void PrintInfo(CamParams &cam);

    // 监听信息
    void ListenerInfo(int source, int target);

private:
    Json::Reader reader;
    Json::Value root;

    string calibrate_file_path;

    vector<CamParams> CamSets;

    tf::TransformBroadcaster* br;

    std::ifstream fin;

    vector<tf::Transform> TFs;

    tf::TransformListener tf_listener;
};

void Camera::readJSON(){
    fin.open(this->calibrate_file_path, std::ios::binary);

    if(reader.parse(fin, root, false)){
        transTFInfo(root);
    }

    fin.close();
}

void Camera::transTFInfo(Json::Value& root){
    CamParams camera;

    int cam_size = root["cameras"].size();

    ROS_INFO("Camera number is: %d", cam_size);

    CamSets.clear();

    for(int i=0; i<cam_size; ++i){
        for(int j=0; j<3; ++j){
            for(int k=0; k<3; ++k){
                camera.R(j, k) = root["cameras"][i]["R"][j][k].asDouble();
            }
        }

        for(int m=0; m<3; ++m){
            camera.t(m, 0) = root["cameras"][i]["t"][m][0].asDouble();
        }

        camera.id = i;

        CamSets.push_back(camera);
    }

    // for(int i=0; i<CamSets.size(); ++i){
    //     PrintInfo(CamSets[i]);
    // }
}

// vector<double> to Eigen::Quaterniond
// Eigen::Quaterniond Camera::transRotation(Eigen::Matrix3d& R){
//     // Eigen::Matrix3d mat;
//     // mat << R[0], R[1], R[2],
//     //              R[3], R[4], R[5],
//     //              R[6], R[7], R[8];

//     Eigen::Quaterniond q(R);

//     return q;
// }

tf::Matrix3x3 Camera::transRotation(Eigen::Matrix3d& R){
    tf::Matrix3x3 rot(R(0,0), R(0,1), R(0,2),
                      R(1,0), R(1,1), R(1,2),
                      R(2,0), R(2,1), R(2,2));

    return rot;
}


tf::Vector3 Camera::transTranslation(Eigen::Matrix<double, 3, 1>& t){
    tf::Vector3 vec;
    vec.setValue(t(0, 0) / 100, t(1, 0)/100, t(2, 0)/100);

    return vec;
}

void Camera::publishTFInfo(){
    int size = CamSets.size();

    TFs.clear();

    for(int i=0; i<size; ++i){
        // tf::Transform tf_tmp;
        tf::Matrix3x3 tf_rot = transRotation(CamSets[i].R);
        // tf_rot = tf_rot.inverse();
        tf::Vector3 tf_trans = transTranslation(CamSets[i].t);
        tf::Transform transform(tf_rot, tf_trans);

        transform = transform.inverse();

        // Eigen::Quaterniond eq = transRotation(CamSets[i].R);
        // tf::Vector3 vec = transTranslation(CamSets[i].t);
        // tf_tmp.setRotation(tf::Quaternion(eq.x(), eq.y(), eq.z(), eq.w()));
        // tf_tmp.setOrigin(vec);

        // tf_tmp = tf_tmp.inverse();
        TFs.push_back(transform);
    }

    for(int i=0; i<size; ++i){
        ostringstream oss_1;
        oss_1 << "camera_" << i;

        br->sendTransform(tf::StampedTransform(TFs[i], ros::Time::now(), "world", oss_1.str()));

        ROS_INFO("Camera_%d's Pose Had Published!", i);
    }
}

void Camera::PrintInfo(CamParams &cam){
    cout << "Camera: " << cam.id << "\n"
         << "Rotation: " << cam.R << "\n"
         << "Translation: " << cam.t << endl;

    cout << endl;
}

void Camera::ListenerInfo(int source, int target){
    tf::StampedTransform transform;

    try
    {
        tf_listener.lookupTransform("camera_"+to_string(target), "camera_"+to_string(source), ros::Time(0), transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    tf::Quaternion tf_q = transform.getRotation();

    Eigen::Matrix3d rot = Eigen::Quaterniond(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z()).toRotationMatrix();

    cout << "X: " << transform.getOrigin().getX()
         << "  Y: " << transform.getOrigin().getY()
         << "  Z: " << transform.getOrigin().getZ() << endl;

    cout << "Rotation: " << rot(0,0) << rot(0,1) << rot(0,2) << "\n"
                         << rot(1,0) << rot(1,1) << rot(1,2) << "\n"
                         << rot(2,0) << rot(2,1) << rot(2,2) << endl;

}


int main(int argc, char** argv){
    ros::init(argc, argv, "camPose");

    string cam_file_path;
    string root_path = "/home/xuchengjun/catkin_ws/src/multi_human_estimation";

    if(argc < 2){
        ROS_WARN("Please enter dataset name.");
        ros::shutdown();
    }
    if(!strcmp(argv[1], "CMU")){
        ROS_INFO("CMU dataset.");
        cam_file_path = root_path + "/data/cameraPose/cmu_cameras.json";
    }
    else if(!strcmp(argv[1], "Shelf")){
        ROS_INFO("Shelf dataset.");
        cam_file_path = root_path + "/data/cameraPose/shelf_cameras.json";
    }
    else if(!strcmp(argv[1], "Campus")){
        ROS_INFO("Campus dataset.");
        cam_file_path = root_path + "/data/cameraPose/campus_cameras.json";
    }
    else{
        ROS_ERROR("Dataset name error.");
    }

    Camera cam;

    cout << cam_file_path << endl;

    while(ros::ok()){
        cam.run(cam_file_path);
        ros::spinOnce();
    }
}
