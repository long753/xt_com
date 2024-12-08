# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBoxes
import camera_config
import random
import math
import time
import sys



def pixel_points( x, y,param):
    global objector_postion
threeD = param
objector_postion[0]= -threeD[y][x][2]/1000.0#x
objector_postion[1]= threeD[y][x][0]/1000.0#y
objector_postion[2]= threeD[y][x][1]/1000.0#z
print(objector_postion)

# print('\n像素坐标 x = %d, y = %d' % (x, y))

# print("xyz :", threeD[y][x][0]/ 1000.0 , threeD[y][x][1]/1000.0 , threeD[y][x][2]/1000.0 , "m")
distance = math.sqrt( threeD[y][x][0] **2 + threeD[y][x][1] **2 + threeD[y][x][2] **2 ) 

distance = distance / 1000.0  # mm -> m
# print("distance:", distance, "m")
def boundingbox_callback(data):
    global pixel_x,pixel_y
pixel_x=1
pixel_y=2
# for target in data.bounding_boxes: 
#         pixel_x = (target.xmax+target.xmin)/2
#         pixel_y = target.ymax
# for target in data.bounding_boxes:
#     pixel_x=(target.xmax+target.xmin)/2
#     pixel_y=target.ymax

def image_callback1(message):
    global frame1
frame1 = bridge.imgmsg_to_cv2(message, "bgr8")
# cv2.namedWindow("Frame1", cv2.WINDOW_NORMAL)
# cv2.imshow("Frame1", frame1) 

def image_callback2(message):
    global frame2
frame2 = bridge.imgmsg_to_cv2(message, "bgr8")
# cv2.namedWindow("Frame2", cv2.WINDOW_NORMAL)
# cv2.imshow("Frame2", frame2)


#camera=Camera()
bridge = CvBridge()
vehicle_id = sys.argv[1]
rospy.init_node('image_node', anonymous=True)
objector_postion=[0,0,0]
while not rospy.is_shutdown():
    rospy.Subscriber("/iris_"+vehicle_id+"/stereo_camera/left/image_raw", Image, image_callback1)
rospy.Subscriber("/iris_"+vehicle_id+"/stereo_camera/right/image_raw", Image,image_callback2)
if 'frame1' in globals() and 'frame2' in globals():
    imgL = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)  # 将BGR格式转换成灰度图片
imgR = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
img1_rectified = cv2.remap(imgL, camera_config.left_map1, camera_config.left_map2, cv2.INTER_LINEAR)
img2_rectified = cv2.remap(imgR, camera_config.right_map1, camera_config.right_map2, cv2.INTER_LINEAR)  
imageL = cv2.cvtColor(img1_rectified, cv2.COLOR_GRAY2BGR)  
imageR = cv2.cvtColor(img2_rectified, cv2.COLOR_GRAY2BGR)

# BM
numberOfDisparities = ((752 // 8) + 15) & -16  # 640对应是分辨率的宽

stereo = cv2.StereoBM_create(numDisparities=96, blockSize=9)  #立体匹配
stereo.setROI1(camera_config.validPixROI1)
stereo.setROI2(camera_config.validPixROI2)
stereo.setPreFilterCap(31)
stereo.setBlockSize(15)
stereo.setMinDisparity(0)
stereo.setNumDisparities(numberOfDisparities)
                                stereo.setTextureThreshold(10)
                                stereo.setUniquenessRatio(15)
                                stereo.setSpeckleWindowSize(100)
                                stereo.setSpeckleRange(32)
                                stereo.setDisp12MaxDiff(1)

                                disparity = stereo.compute(img1_rectified, img2_rectified) # 计算视差
        
        disp = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)  #归一化函数算法
        #print(disp,"success")

        
        threeD = cv2.reprojectImageTo3D(disparity, camera_config.Q, handleMissingValues=True)  #计算三维坐标数据值
        threeD = threeD * 16
                                rospy.Subscriber("/uav_"+vehicle_id+"_l"+"/darknet_ros/bounding_boxes", BoundingBoxes, boundingbox_callback,queue_size=1)
                                # rospy.Subscriber("/uav_"+vehicle_id+"_l"+"/darknet_ros/bounding_boxes", BoundingBoxes, boundingbox_callback,queue_size=10)
                                print(pixel_x,pixel_y)
                                # threeD[y][x] x:0~640; y:0~480;   !!!!!!!!!!
                                # if 'pixel_x' in globals() and 'pixel_y' in globals():
                                #     print('success')
                                #     rospy.Subscriber("/uav_"+vehicle_id+"_l"+"/darknet_ros/bounding_boxes", BoundingBoxes, boundingbox_callback,queue_size=1)
                                #     pixel_points(pixel_x,pixel_y,threeD)

                                #cv2.imshow("left", frame1)
                                #cv2.imshow("right", frame2)
                                # cv2.imshow("left_r", imgL)
                                #cv2.imshow("right_r", imgR)
                                # cv2.imshow(WIN_NAME, disp)  #显示深度图的双目画面

                                # key = cv2.waitKey(1)
                                # if key == ord("q"):
                                #         break
                                # time.sleep(0.1)

                                cv2.destroyAllWindows()
                                # while not rospy.is_shutdown():
                                #   rospy.Subscriber("/iris_0/stereo_camera/left/image_raw", Image, camera.image_callback1)
                                #   rospy.Subscriber("/iris_0/stereo_camera/right/image_raw", Image,camera.image_callback2)
                                #   key = cv2.waitKey(1)
                                #   if key == ord("q"):
                                #     break
                                #   rospy.spin()
                                # cv2.destroyAllWindows()