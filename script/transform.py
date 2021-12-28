'''
author: chengjun_xu
转换数据集为我们自己的格式
网络输出结果转为我们自己的格式
'''

import json
import os
import os.path as osp
import numpy as np
import argparse
import cv2
from IPython import embed
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d

pairs = [[0, 1], [0, 2], [0, 9], [9, 10], [10, 11],
         [0, 3], [3, 4], [4, 5], [2, 12], [12, 13],
         [13, 14], [2, 6], [6, 7], [7, 8]]

colors = ['r', 'g', 'b', 'y', 'k', 'p']

''''
数据集格式：00000001.json
{
    img_path: xxx,
    "img_to_json:"
    [
        {
            cam_id: x,
            person_num:x,
            pred_2d:[mxNx3], (x,y,p)
            root_3d:[mx2x4]  (x,y,z,p)
        },
        {
            cam_id: x,
            person_num:x,
            pred_2d:[mxNx3], (x,y,p)
            root_3d:[mx2x4]  (x,y,z,p)
        },
        ...
        ...
    ]
}

'''

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

def get_keys(d, index):
    for i in range(len(d)):
        img_name = d[i]['image_path'].strip().split('_')
        img_name = img_name[2][:-4]
        if int(img_name) == index:
            return i

    return -1

def process(data, data_index, f):
    dict = {
        "cam_id": "",
        "pred_2d":[],
        "root_3d":[]
    }

    dict['cam_id'] = f

    for i in range(len(data[data_index]['pred_2d'])):
        if data[data_index]['pred_2d'][i] is None:
            continue

        tmp = []
        for j in range(len(data[data_index]['pred_2d'][i])):
            tmp.append([data[data_index]['pred_2d'][i][j][0], data[data_index]['pred_2d'][i][j][1],
                        data[data_index]['pred_2d'][i][j][3]])

        dict['pred_2d'].append(tmp)

    for i in range(len(data[data_index]['pred_3d'])):
        if data[data_index]['pred_3d'][i] is None:
            continue

        tmp = []
        tmp.append([data[data_index]['pred_3d'][i][0][0], data[data_index]['pred_3d'][i][0][1],
                    data[data_index]['pred_3d'][i][0][2], data[data_index]['pred_2d'][i][0][3]])

        tmp.append([data[data_index]['pred_3d'][i][2][0], data[data_index]['pred_3d'][i][2][1],
                    data[data_index]['pred_3d'][i][2][2], data[data_index]['pred_2d'][i][2][3]])

        dict['root_3d'].append(tmp)

    return dict
    # dict['pred_2d'] = data[data_index]['pred_2d'].tolist()



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--img_dir", "-i", type=str, default=None)
    parser.add_argument("--frames", "-f", type=int, default=0)
    parser.add_argument("--write", "-w", type=bool, default=True)
    args = parser.parse_args()

    root_path = "/media/xuchengjun/D/dataset/cmu_test"
    output_path = "/media/xuchengjun/D/dataset/cmu_test/data"

    if args.frames < 2:
        print("please input frames numbers using '-f' argument")
        exit()

    if args.img_dir is None:
        print("please input img_dir path using '-i' argument")
        exit()

    img_dir = osp.join(root_path, args.img_dir)
    json_path = osp.join(root_path, "network")

    index = 14022
    i = 0
    while index < 14993:
        outputfile = osp.join(output_path, str(args.frames))
        if os.path.exists(outputfile) is False:
            os.mkdir(outputfile)

        dicts = {
            "img_name": "",
            "img_to_json":[]
        }

        dicts['img_name'] = "{:08d}.jpg".format(index)

        for fi in range(args.frames):
            with open(osp.join(json_path, "{}.json".format(fi)), 'r') as f:
                data = json.load(f)['3d_pairs']

            data_index = get_keys(data, index)

            if data_index < 0 or data_index > 971:
                print("Index are unvaild")
                continue

            #print(data[data_index]['pred_2d'])
            dict = process(data, data_index, fi)
            #print("--------------------------------")
            #print(dict['pred_2d'])

            dicts['img_to_json'].append(dict)

        if i == 100 or i == 203 or i == 500:
            print(data[data_index]['pred_2d'])
            print("--------------------------------")
            print(dict['pred_2d'])

        if args.write:
            json_str = json.dumps(dicts, cls=MyEncoder)
            if outputfile is not None:
                outputfile = outputfile + '/{:08d}.json'.format(i)
                print('Writing {0}'.format(outputfile))
            with open(outputfile,'w') as json_file:
                json_file.write(json_str)


        index += 1
        i += 1


if __name__ == "__main__":
    main()
