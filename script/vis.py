import json
import os.path as osp
import numpy as np
import argparse
import cv2
import cmath
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from IPython import embed

pairs = [[0, 1], [0, 2], [0, 9], [9, 10], [10, 11],
         [0, 3], [3, 4], [4, 5], [2, 12], [12, 13],
         [13, 14], [2, 6], [6, 7], [7, 8]]

colors = ['r', 'g', 'b', 'y', 'k', 'p']

def vis_result():
    parser = argparse.ArgumentParser()
    parser.add_argument("--img_dir", "-i", type=str, default="/home/xuchengjun/catkin_ws/src/multi_human_estimation/data/res")
    parser.add_argument("--json_path", "-p", type=str, default="/home/xuchengjun/catkin_ws/src/multi_human_estimation/data/res/demo.json")
    args = parser.parse_args()

    if args.img_dir is None:
        print("please input img_dir path using '-i' argument")
        exit()

    with open(args.json_path, 'r') as f:
        data = json.load(f)['human']

    for idata in data:
        pred_3d = np.array(idata['pred_3d'])
        img_path = osp.join(args.img_dir, "2")
        # embed()
        print(img_path + "/0.jpg");
        img = cv2.imread(img_path + "/0.jpg");

        fig = plt.figure(figsize=(20, 10))
        ax1 = fig.add_subplot(121)
        ax1.imshow(img)
        ax1.set_xticks([])
        ax1.set_yticks([])

        ax2 = fig.add_subplot(122, projection='3d')
        for ip in range(len(pred_3d)):
            p3d = pred_3d[ip]
            # embed()
            # ax2.scatter(p3d[:,0], p3d[:,1], p3d[:,2], s=200, cmap='jet')

            for pair in pairs:
                # embed()
                if all(abs(p3d[pair, 0]) != 0) \
                    and all(abs(p3d[pair, 1]) != 0)\
                    and all(abs(p3d[pair, 2]) != 0):
                    ax2.plot(p3d[pair, 0], p3d[pair, 1], p3d[pair, 2], c=colors[ip%len(colors)])
                    
                else:
                    continue;

        ax2.view_init(azim=-90, elev=-45)

        plt.show()


if __name__ == "__main__":
    vis_result()
