##
# 数据集格式转化
# 这里是一个数据集相机的参数设置文件，包含读取相机参数和投影函数等功能
# #


from __future__ import division
import numpy as np
from IPython import embed
from numpy import mat


def unfold_camera_param(camera):
    R = np.array(camera['R'], dtype=np.float)
    T = np.array(camera['T'], dtype=np.float)
    fx = np.array(camera['fx'], dtype=np.float)
    fy = np.array(camera['fy'], dtype=np.float)
    cx = np.array(camera['cx'],dtype=np.float)
    cy = np.array(camera['cy'],dtype=np.float)

    K = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]]
    )

    return R, T, K


def project_point_radial(x, R, T, K):
    """
    Args
        x: Nx3 points in world coordinates
        R: 3x3 Camera rotation matrix
        T: 3x1 Camera translation parameters
        f: (scalar) Camera focal length
        c: 2x1 Camera center
        k: 3x1 Camera radial distortion coefficients
        p: 2x1 Camera tangential distortion coefficients
    Returns
        ypixel.T: Nx2 points in pixel space
    """

    pose2d = np.zeros((x.shape[0], 2), dtype=np.int32)

    R = mat(R)

    for i in range(x.shape[0]):
        point_cam = np.asarray(R * x[i].reshape(3, 1) + T)

        pose2d[i][0] = int(point_cam[0] * K[0, 0] / point_cam[2] + K[0, 2])
        pose2d[i][1] = int(point_cam[1] * K[1, 1] / point_cam[2] + K[1, 2])

    return pose2d

def project_pose_from_world(pose_3d, R, t, K):
    pose_3d = transPose(pose_3d)
    # pose_3d = pose_3d.reshape((-1, 3)).transpose()
    R = mat(R)
    x = np.asarray(R*pose_3d + t)
    # x = pose_3d[0:3, :] = np.dot(R, pose_3d[0:3, :] + t)
    x[0:2, :] = x[0:2, :]/x[2, :]
    r = x[0, :] * x[0, :] + x[1, :] * x[1, :]

    #x[0, :] = x[0, :] * (1 + kd[0]*r + kd[1]*r*r + kd[4]*r*r*r) + 2*kd[2]*x[0, :]*x[1, :] + kd[3]*(r + 2*x[0, :]*x[0, :])
    #x[1, :] = x[1, :] * (1 + kd[0]*r + kd[1]*r*r + kd[4]*r*r*r) + 2*kd[3]*x[0, :]*x[1, :] + kd[2]*(r + 2*x[1, :]*x[1, :])

    x[0, :] = K[0, 0]*x[0, :] + K[0, 1]*x[1, :] + K[0, 2]
    x[1, :] = K[1, 0]*x[0, :] + K[1, 1]*x[1, :] + K[1, 2]

    pose = returnPose(x)
    return pose


def project_pose(x, camera):
    R, T, K = unfold_camera_param(camera)
    return project_point_radial(x, R, T, K)
    # return project_pose_from_world(x, R, T, K)


def world_to_camera_frame(x, R, T):
    """
    Args
        x: Nx3 3d points in world coordinates
        R: 3x3 Camera rotation matrix
        T: 3x1 Camera translation parameters
    Returns
        xcam: Nx3 3d points in camera coordinates
    """

    xcam = np.matmul(R, np.transpose(x) - T)
    return np.transpose(xcam)


def camera_to_world_frame(x, R, T):
    """
    Args
        x: Nx3 points in camera coordinates
        R: 3x3 Camera rotation matrix
        T: 3x1 Camera translation parameters
    Returns
        xcam: Nx3 points in world coordinates
    """
    xcam = np.matmul(np.transpose(R), np.transpose(x))
    xcam = xcam + T  # rotate and translate
    return np.transpose(xcam)


def transPose(pose_3d):
    pose = np.ones((3, 15), dtype=np.float32)
    for joint_id in range(pose_3d.shape[0]):
        pose[0, joint_id] = pose_3d[joint_id, 0]
        pose[1, joint_id] = pose_3d[joint_id, 1]
        pose[2, joint_id] = pose_3d[joint_id, 2]

    return pose

def returnPose(pose_3d):
    pose = np.ones((15, 2), dtype=np.float32)
    for joint_id in range(pose_3d.shape[1]):
        pose[joint_id, 0] = pose_3d[0, joint_id]
        pose[joint_id, 1] = pose_3d[1, joint_id]
        #pose[joint_id, 2] = pose_3d[2, joint_id]

    return pose