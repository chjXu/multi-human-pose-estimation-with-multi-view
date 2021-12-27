'''
author: chengjun_xu
function: 2D 姿态可视化脚本
'''

import json
import os.path as osp
import numpy as np
import argparse
import cv2

from IPython import embed

pairs = [[0, 1], [0, 2], [0, 9], [9, 10], [10, 11],
         [0, 3], [3, 4], [4, 5], [2, 12], [12, 13],
         [13, 14], [2, 6], [6, 7], [7, 8]]

colors = [(0,0,255), (0,255,255), (255,0,255), (255,255,0), (120,19,200), (100,100,180)]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_type", "-d", type=str, default=None,
                        description="dataset type. Campus, Shelf and CMU.")
    parser.add_argument("--index", "-i", type=int, default=None,
                        description="view index. 0, 1, 2, ...")
    args = parser.parse_args()

    data_type = args.data_type  #CMU, Shelf
    index = args.index
    camera_id = str(index)
    json_name = "_{0}.json".format(index)

    img_name = None
    exist = 0
    if data_type == "CMU":
        exist = 0
        img_name = "00_{0}_00010600.jpg".format(index)
    elif data_type == "Shelf":
        img_name = "img_000572.png"
        exist = 0
    else:
        Error("Dataset Type is Error.")

    root_path = osp.join("../../data", data_type, camera_id)
    img_path = osp.join(root_path, img_name)
    json_path = osp.join(root_path, json_name)

    with open(json_path, 'r') as f:
        data = json.load(f)['3d_pairs']

    pred_2d = np.array(data['pred_2d'])

    img = cv2.imread(img_path)
    img = cv2.resize(img, (832, 512))

    for ip in range(len(pred_2d)): #总的人数
        p2d = pred_2d[ip]
        line_width = 1
        for p in range(len(pairs)): #每一个人
            if p == 1:
                line_width = 5
            else:
                line_width = 2
            if p2d[pairs[p]][0][0] != int(exist) and p2d[pairs[p]][0][1] != int(exist):
                cv2.circle(img, (int(p2d[pairs[p]][0][0]), int(p2d[pairs[p]][0][1])), line_width, colors[ip], -1)
            if p2d[pairs[p]][0][0] !=int(exist) and p2d[pairs[p]][0][1] != int(exist) \
                and p2d[pairs[p]][1][0] !=int(exist) and p2d[pairs[p]][1][1] != int(exist):
                cv2.line(img, (int(p2d[pairs[p]][0][0]), int(p2d[pairs[p]][0][1])),
                        (int(p2d[pairs[p]][1][0]), int(p2d[pairs[p]][1][1])), colors[ip], line_width)

    cv2.imshow(img_name, img)
    cv2.waitKey(0)
    print(root_path)
    cv2.imwrite(root_path + '/res.png',img)


if __name__ == '__main__':
    main()
