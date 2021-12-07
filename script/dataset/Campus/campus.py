##
# 将数据集格式进行转换 （campus）
# 
# #

import os.path as osp
import numpy as np
import pickle
import scipy.io as scio
import logging
import copy
import os
import json
import cv2

from collections import OrderedDict
from cameras import *

from IPython import embed

CAMPUS_JOINTS_DEF = {
    'Right-Ankle': 0,
    'Right-Knee': 1,
    'Right-Hip': 2,
    'Left-Hip': 3,
    'Left-Knee': 4,
    'Left-Ankle': 5,
    'Right-Wrist': 6,
    'Right-Elbow': 7,
    'Right-Shoulder': 8,
    'Left-Shoulder': 9,
    'Left-Elbow': 10,
    'Left-Wrist': 11,
    'Bottom-Head': 12,
    'Top-Head': 13
}

body_edges = [
    [0, 1],
    [1, 2],
    [3, 4],
    [4, 5],
    [2, 3],
    [6, 7],
    [7, 8],
    [9, 10],
    [10, 11],
    [2, 8],
    [3, 9],
    [8, 12],
    [9, 12],
    [12, 13]
]

# class JointsDataset:
#     def __init__(self) -> None:
#         self.num_joints = 0

class Campus:
    def __init__(self) -> None:
        self.dataset_root = "/media/xuchengjun/D/dataset/CampusSeq1"
        self.frame_range = list(range(350, 471)) + list(range(650, 751))
        self.cam_list = [0, 1, 2]
        self.num_views = len(self.cam_list)

        self.num_joints = len(CAMPUS_JOINTS_DEF)

        self.db = self._get_db()

        self.db_size = len(self.db)


    def _get_db(self):
        width = 360
        height = 288

        db = []
        cameras = self._get_cam()

        datafile = os.path.join(self.dataset_root, 'actorsGT.mat')
        data = scio.loadmat(datafile)
        actor_3d = np.array(np.array(data['actor3D'].tolist()).tolist()).squeeze() # num_person * num_frame
        actor_2d = np.array(np.array(data['actor2D'].tolist()).tolist()).squeeze() # num_person * num_frame


        num_person = len(actor_3d)
        num_frame = len(actor_3d[0])

        for i in self.frame_range:
            for k, cam in cameras.items():
                image = osp.join(self.dataset_root, "Camera" + k, "campus4-c{0}-{1:05d}.png".format(k, i))
                print(image)
                image = cv2.imread(image)

                all_poses_3d = []
                all_poses_vis_3d = []
                all_poses = []
                all_poses_vis = []

                for person in range(num_person):
                    pose3d = actor_3d[person][i] * 1000 # m to mm
                    pose2d_gt = actor_2d[person][i]
        
                    if len(pose3d[0]) > 0:
                        all_poses_3d.append(pose3d)
                        all_poses_vis_3d.append(np.ones((self.num_joints,3)))
                
                        pose2d = project_pose(pose3d, cam)
                        
                        #pose2d = pose2d_gt

                        x_check = np.bitwise_and(pose2d[:, 0] >= 0,
                                                 pose2d[:, 0] <= width - 1)

                        y_check = np.bitwise_and(pose2d[:, 1] >= 0,
                                                 pose2d[:, 1] <= height - 1)

                        check = np.bitwise_and(x_check, y_check)

                        joints_vis = np.ones((len(pose2d), 1))
                        joints_vis[np.logical_not(check)] = 0
                        all_poses.append(pose2d)
                        all_poses_vis.append(
                            np.repeat(
                                np.reshape(joints_vis, (-1, 1)), 2, axis=1))

                self.drawInImage(pose2d, image)
            # image = cv2.resize(image, (960, 540))
            # cv2.imshow("Camera", image)
            # cv2.waitKey(3)
                # pred_index = '{}_{}'.format(k, i)
                # preds = self.pred_pose2d[pred_index]
                # preds = [np.array(p["pred"]) for p in preds]

                # db.append({
                #     'image': osp.join(self.dataset_root, image),
                #     'joints_3d': all_poses_3d,
                #     'joints_3d_vis': all_poses_vis_3d,
                #     'joints_2d': all_poses,
                #     'joints_2d_vis': all_poses_vis,
                #     'camera': cam,
                #     #'pred_pose2d': preds
                # })
            
        return db


    def _get_cam(self):
        cam_file = osp.join("../../config", "campus_camera_parameters.json")
        with open(cam_file) as cfile:
            cameras = json.load(cfile)

        for id, cam in cameras.items():
            for k, v in cam.items():
                cameras[id][k] = np.array(v)

        return cameras

    def __len__(self):
        return self.db_size // self.num_views


    def drawInImage(self, poses_2d, image):
        colors = ([255, 0, 0], [0, 0, 255], [0, 255, 0], [0, 255, 255], [255, 0, 255])

        for person in range(len(poses_2d)):
            pose_2d = poses_2d[person]

            for part_id in range(len(body_edges)):
                kpt_a_id = body_edges[part_id][0]
         
                #a_conf = poses_2d[kpt_a_id, 0]
                a_conf = 1
                if a_conf != -1:  # Point exist
                    joint_2d_a = pose_2d[kpt_a_id]
                    
                    cv2.circle(image, (int(joint_2d_a[0]), int(joint_2d_a[1])), 3, colors[person], 4)
                kpt_b_id = body_edges[part_id][1]
                #b_conf = poses_2d[kpt_b_id, 0]
                b_conf = 1
                if b_conf != -1:
                    joint_2d_b = pose_2d[kpt_b_id]
                    cv2.circle(image, (int(joint_2d_b[0]), int(joint_2d_b[1])), 3, colors[person], 4)
                if a_conf != -1  and b_conf != -1:
                    cv2.line(image, (int(joint_2d_a[0]), int(joint_2d_a[1])), (int(joint_2d_b[0]), int(joint_2d_b[1])), 
                            colors[person], 4)

        # image = cv2.resize(image, (1920, 1080))
        cv2.imshow("Camera", image)
        cv2.waitKey(0)

    def drawInImage_copy(self, poses_2d, image):
        colors = ([255, 0, 0], [0, 0, 255], [0, 255, 0], [0, 255, 255], [255, 0, 255])

        for person in range(len(poses_2d)):
            pose_2d = poses_2d[person]

            for part_id in range(len(body_edges)):
                kpt_a_id = body_edges[part_id][0]
                #a_conf = poses_2d[kpt_a_id, 0]
                a_conf = 1
                if a_conf != -1:  # Point exist
                    joint_2d_a = pose_2d[kpt_a_id]
                    cv2.circle(image, (int(joint_2d_a[0]), joint_2d_a[1]), 3, colors[person], 4)
                kpt_b_id = body_edges[part_id][1]
                #b_conf = poses_2d[kpt_b_id, 0]
                b_conf = 1
                if b_conf != -1:
                    joint_2d_b = pose_2d[kpt_b_id]
                    cv2.circle(image, (joint_2d_b[0], joint_2d_b[1]), 3, colors[person], 4)
                if a_conf != -1  and b_conf != -1:
                    cv2.line(image, (joint_2d_a[0], joint_2d_a[1]), (joint_2d_b[0], joint_2d_b[1]), 
                            colors[person], 4)

        # image = cv2.resize(image, (1920, 1080))
        cv2.imshow("Camera", image)
        cv2.waitKey(3)

def main():
    campus = Campus()
    campus._get_db()

if __name__=="__main__":
    main()
