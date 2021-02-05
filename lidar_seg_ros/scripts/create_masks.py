#! /usr/bin/env python

import copy
import cv2
import glob
import matplotlib.pyplot as plt
import numpy as np
import pdb


if __name__=="__main__":

    basePath = "/home/rxth/catkin_ws/src/LidarSeg/lidar_seg_ros/pointclouds/"
    basePathLen = len(basePath)

    bDebug = False

    numClasses = 2

    inFolder = "4_png_rangeImg_forSeg/"
    srcPath = basePath + inFolder
    print("Looking for images in: "+srcPath)
    dstPath = basePath + "5_mask_visualize/" # mask with [0, 255] ... to visualize mask easily
    dstPath2 = basePath + "6_mask_index/" # mask with label indexes (for two class case this also works as single probability mask, since labels indices will be 0 and 1 which belong to [0,1])
    # dstPath3 = basePath + "7_mask_probabChannels/" # mask with a probability [0,1] channel for each class 
    print("Saving images in: "+dstPath)
    print("Saving images in: "+dstPath2)

    for file in list( glob.glob(srcPath+'*.png') ):

        fileName = file[basePathLen+len(inFolder):] # plus len of "4_png_rangeImg_forSeg/" folder
        fileBaseName = fileName[:-4] # ignore ".png"
        print(fileName)

        img = cv2.imread(file)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # ret,img_bin = cv2.threshold(img_gray,200,255,cv2.THRESH_BINARY)
        # ret,img_bin = cv2.threshold(img_gray,190,255,cv2.THRESH_BINARY)
        ret,img_bin = cv2.threshold(img_gray,0,10,cv2.THRESH_BINARY_INV) # highlight all tree points...which are now being suppressed to appear as black points

        tmp_img = np.array(img_bin)
        img_bin_0_255 = copy.deepcopy(tmp_img)
        img_bin_0_255[img_bin_0_255 > 0] = 255
        img_bin_0_1 = copy.deepcopy(tmp_img)
        img_bin_0_1[img_bin_0_1 > 0] = 1

        if(bDebug):
            print(np.shape(img))
            print(np.shape(img_gray))
            print(np.shape(img_bin))

            cv2.imshow("input_image", img)
            cv2.waitKey(2000)

            cv2.imshow("image_gray", img_gray)
            cv2.waitKey(2000)

            cv2.imshow("image_binary", img_bin)
            cv2.waitKey(2000)

        # cv2.imwrite(dstPath+fileBaseName+'_mask_viz.png', img_bin)

        cv2.imwrite(dstPath+fileBaseName+'_mask_viz.png', img_bin_0_255)
        cv2.imwrite(dstPath2+fileBaseName+'_mask_idx.png', img_bin_0_1)


        ### Pytorch negative log likelihood metrix does not require multi-channel input.
        ### Single channel with indices is sufficient.
        # # # create multi-channel probability image (ie. one class highlighted as 1 on each channel)
        # height, width = img_bin.shape
        # image_probab_channels = np.zeros([height, width, numClasses])
        # for i in range(0,height):
        #     for j in range(0, width):

        #         # if class 1 (tree) exists, fill (1st) channel
        #         if(img_bin[i][j]):
        #             image_probab_channels[i][j][1] = 1
        #         # else fill 0th channel to be high
        #         else:
        #             image_probab_channels[i][j][0] = 1

        # # PIL images can only of 1 channel, 3 channels or 4 channels
        # # cv2.imwrite(dstPath3+fileBaseName+'_multichannel_probabs.png', image_probab_channels)
        # # So, saving as numpy binary file
        # np.save(dstPath3+fileBaseName+'_multichannel_probabs.npy', image_probab_channels)


        












