##########图片重命名 Rename，设置标签##########
import os
import argparse

if __name__=='__main__':
    parser = argparse.ArgumentParser(description="Translate dataset type.")
    parser.add_argument('--camera_index', '-c', type=str, required=True)
    parser.add_argument('--type', '-t', type=str, required=True,
                        description="暂时不可用")
    # parser.add_argument('node', type=int, required=False)
    # parser.add_argument('frame', type=int, required=False)
    # parser.add_argument('--outputfile', type=str, nargs='?', default=None, required=False)
    args = parser.parse_args()

    ROOT_PATH = "/media/xuchengjun/D/dataset/cmu_test"
    # ROOT_PATH = "/home/xuchengjun/catkin_ws/src/multi_human_estimation/data/CMU"
    img_path = os.path.join(ROOT_PATH, args.camera_index)
    gt_path = os.path.join(ROOT_PATH, "gt")
    imglist = os.listdir(img_path)

    input = args.type

    i = 14022
    for img in imglist:
        print(i)
        src = None
        if input == "img":
            src = os.path.join(os.path.abspath(img_path), "00_{%2d}_000{}.jpg".format(args.camera_index, i))  # original name
        elif input == "gt":
            src = os.path.join(os.path.abspath(gt_path), "body3DScene_000{}.json".format(i))
        print(src)
        #src /home/jovyan/work/data/gyx/Test_all/stag_01_test/0001.jpg
        img=src.strip().split('/')
        print(img)
        img_n = img[7][:-4]
        print(img_n) #0001
        filename=img_n.strip().split('_')
        print(filename)
        if filename:
            id = filename[2]
            if(i != int(id)):
                break
        else:
            print('Wrong filename')
        print(id)
        dst = os.path.join(img_path,'{}.jpg'.format(id))
        print(dst)
        os.rename(src, dst)  # rename==>recover the original name
        i+=1
