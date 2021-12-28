'''
author: chengjun_xu
转换数据集为我们自己的格式
CMU 原始数据集测试文件
'''

import argparse
import os
import numpy as np
import json
import cv2
from IPython import embed

# Edges in the skeleton
edges = np.array([[1,2],[1,4],[4,5],[5,6],[1,3],[3,7],[7,8],[8,9],[3,13],[13,14],[14,15],[1,10],[10,11],[11,12]])-1

# Random colors to draw skeletons
colors = np.random.randint(50,256,(100,3))

class MyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return super(MyEncoder, self).default(obj)

def load_cameras( data_path, seq_name ):
    """Load the camera calibration file for a given sequence"""

    # Load camera calibration file
    with open(data_path+'/calibration_{0}.json'.format(seq_name)) as cfile:
        calib = json.load(cfile)

    # Cameras are identified by a tuple of (panel#,node#)
    cameras = {(cam['panel'],cam['node']):cam for cam in calib['cameras']}

    # Convert data into numpy arrays for convenience
    for k,cam in cameras.items():
        cam['K'] = np.matrix(cam['K'])
        cam['distCoef'] = np.array(cam['distCoef'])
        cam['R'] = np.matrix(cam['R'])
        cam['t'] = np.array(cam['t']).reshape((3,1))

    return cameras

def load_image( data_path, panel, node, frame_index ):
    """Load an image given a camera name and frame number"""

    prefix = "hd" if panel==0 else "vga"
    img_path = data_path+'/{0}Imgs/'.format(prefix)
    img_fname = img_path+'{0:02d}_{1:02d}/{0:02d}_{1:02d}_{2:08d}.jpg'.format(panel, node, frame_index)

    im = cv2.imread(img_fname)
    if im is None:
        print('Image file not found: {0}'.format(img_fname))
        exit(1)
    return im, img_fname

def load_skeletons( data_path, panel, frame_index ):
    """Load skeletons given a panel and frame number"""

    # HD and VGA frames are indexed differently
    prefix = "hd" if panel==0 else "vga"
    skel_json_path = data_path+'/{0}Pose3d_stage1/'.format(prefix)

    bframe = { "bodies": [] }
    skel_json_fname = skel_json_path+'body3DScene_{0:08d}.json'.format(frame_index)
    try:
        with open(skel_json_fname) as fid:
            bframe = json.load( fid )

    except IOError as e:
        print('Error reading skeletons {0}\n'.format(skel_json_fname)+e.strerror)
        print('No skeletons available')

    return bframe

def draw_skeletons( im, bframe, cam ):
    """Plot skeletons onto image"""

    for body in bframe['bodies']:
        if body['id']<0:
            # This is an outlier.
            continue


        skel = np.array(body['joints19']).reshape((-1,4)).transpose()


        # Project skeleton into view
        pt = cv2.projectPoints(skel[0:3,:].transpose().copy(),
                      cv2.Rodrigues(cam['R'])[0], cam['t'], cam['K'],
                      cam['distCoef'])

        pt = np.squeeze(pt[0], axis=1).transpose()

        # Show only points detected with confidence
        valid = skel[3,:]>0.1

        # Plot edges for each bone
        col = tuple(colors[body['id']])
        for edge in edges:
            if valid[edge[0]] or valid[edge[1]]:
                cv2.line(im,
                tuple(pt[0:2,edge[0]].astype(int)),
                tuple(pt[0:2,edge[1]].astype(int)), col, 2)


def convert_wc_to_cc(X,R, t):
    """ Projects points X (3xN) using camera intrinsics K (3x3),
    extrinsics (R,t) and distortion parameters Kd=[k1,k2,p1,p2,k3].

    Roughly, x = K*(R*X + t) + distortion

    See http://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
    or cv2.projectPoints
    """

    x = np.asarray(R*X + t)

    return x


''''
数据集格式：00000001.json
{
    "img_path:" xxx
    "camera":[
        {
            "id":0
            "K":[]
            "D":[]
            "R":[]
            "t":[]
        },{

        }
    ]
    "bodies:"[
        "id":0,
        "joint_2d:" [],
        "root_3d:"[]
    ]

    "gt_3d:"[
        {
            "id:"0,
            "joint_3d:"[]
        }
    ]
}

'''

def translate( im, bframe,img_path, cam, panel,node,frame,seq_name, write):
    """Plot skeletons onto image"""

    outputfile = './data/' + str(node)
    if os.path.exists(outputfile) is False:
        os.mkdir(outputfile)


    dict = {'img_path':'',
            'camera':{
                'id':-1,
                'K':[],
                'D':[],
                'R':[],
                't':[]
            },
            'bodyies':{
                'id':0,
                'joint_2d':[],
                'root_3d':[],
            },
            'gt_3d':{
                'id':0,
                'joint_3d':[]
            }}

    dict['img_path'] = img_path

    for body in bframe['bodies']:
        if body['id']<0:
            # This is an outlier.
            continue


        skel = np.array(body['joints19']).reshape((-1,4)).transpose()   # 这是世界坐标系下的三维点，即gt值

        dict['camera']['id'] = cam['node']
        dict['camera']['K'] = cam['K']
        dict['camera']['D'] = cam['distCoef']
        dict['camera']['R'] = cam['R']
        dict['camera']['t'] = cam['t']


        # Project Root into view
        root= convert_wc_to_cc(skel[0:3,:].copy(),
                                cam['R'], cam['t'])

        dict['bodyies']['id'] = body['id']
        dict['bodyies']['root_3d'].append(root.transpose()[0])
        dict['bodyies']['root_3d'].append(root.transpose()[2])

        dict['gt_3d']['id'] = body['id']
        dict['gt_3d']['joint_3d'] = root.transpose()


        # Project skeleton into view
        pt = cv2.projectPoints(skel[0:3,:].transpose().copy(),
                      cv2.Rodrigues(cam['R'])[0], cam['t'], cam['K'],
                      cam['distCoef'])


        pt = np.squeeze(pt[0], axis=1).transpose()

        pt_list = []

        for i in range(19):
            point = pt[:,i]
            pt_list.append(point)

        dict['bodyies']['joint_2d'] = pt_list

        # Show only points detected with confidence
        valid = skel[3,:]>0.1

        # Plot edges for each bone
        col = tuple(colors[body['id']])
        col = ( int (col [ 0 ]), int (col [ 1 ]), int (col [ 2 ]))
        for edge in edges:
            if valid[edge[0]] or valid[edge[1]]:
                cv2.line(im,
                tuple(pt[0:2,edge[0]].astype(int)),
                tuple(pt[0:2,edge[1]].astype(int)), col, 2)

        cv2.imshow("img", im)
        cv2.waitKey(1000)

    if write:
        json_str = json.dumps(dict, cls=MyEncoder)
        if outputfile is not None:
            json_path = outputfile + '/{0:02d}_{1:08d}.json'.format(node,frame)
            print('Writing {0}'.format(outputfile))
        with open(json_path,'w') as json_file:
            json_file.write(json_str)


def main():
    # parser = argparse.ArgumentParser(description="Show a frame with overlaid skeletons.")
    # parser.add_argument('data_path', type=str, required=False)
    # parser.add_argument('panel', type=int, required=False)
    # parser.add_argument('node', type=int, required=False)
    # parser.add_argument('frame', type=int, required=False)
    # parser.add_argument('--outputfile', type=str, nargs='?', default=None, required=False)
    # args = parser.parse_args()

    data_path = "/media/xuchengjun/D/dataset/panoptic-toolbox/171204_pose1_sample"
    panel = 0
    node = 0
    frame = 0

    node_num = len( os.listdir(data_path + "/hdImgs" ))

    # data_path = args.data_path
    # panel = args.panel
    # node = args.node
    # frame = args.frame
    # outputfile = args.outputfile

    # 所有视角的第i张图片信息
    dict_all = []

    for i in range(100):
        # outputfile = './data/' + str(i)
        # if os.path.exists(outputfile) is False:
        #     os.mkdir(outputfile)

        img_num = len(os.listdir(data_path + "/hdImgs/{0:02d}_{1:02d}".format(panel, i)))

        for j in range(30):

            seq_name = os.path.basename(os.path.normpath(data_path))

            if panel<0 or panel>20:
                print('Panel {0} not yet supported'.format(panel))
                exit(1)

            cameras = load_cameras(data_path, seq_name)

            im, img_path = load_image(data_path, panel, j, frame)
            bframe = load_skeletons(data_path, panel, frame)

            translate(im, bframe, img_path,cameras[(panel, j)], panel, i, j, seq_name, False)

            #dict_all.append(di)
            # draw_skeletons(im, bframe, cameras[(panel,node)])


        # json_str = json.dumps(dict_all, cls=MyEncoder)
        # if outputfile is not None:
        #     outputfile = '{0:08d}.json'.format(node,frame)
        #     print('Writing {0}'.format(outputfile))
        # with open(outputfile,'w') as json_file:
        #     json_file.write(json_str)


if __name__=='__main__':
    main()
