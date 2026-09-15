#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

rospy.init_node('virtual_camera')
bridge = CvBridge()

# 发布两个话题，和你原来的相机完全一致
image_pub = rospy.Publisher('/ks1a293_camera/image_raw', Image, queue_size=1)
info_pub = rospy.Publisher('/ks1a293_camera/camera_info', CameraInfo, queue_size=1)

rate = rospy.Rate(10)  # 10Hz
while not rospy.is_shutdown():
    # 创建一张 640x480 的黑色图像
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    # 发布图像
    image_msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    image_msg.header.frame_id = 'ks1a293_camera'
    image_msg.header.stamp = rospy.Time.now()
    image_pub.publish(image_msg)
    
    # 发布空的相机信息（AprilTag 只需要话题存在就行）
    info_msg = CameraInfo()
    info_msg.header = image_msg.header
    info_msg.width = 640
    info_msg.height = 480
    info_pub.publish(info_msg)
    
    rate.sleep()
