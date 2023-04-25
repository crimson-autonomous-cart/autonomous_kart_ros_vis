#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Temperature
from first_package.msg import kartTemperatures
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from yolov5.detect import *

def callback(data):
    global idx
    global yolo_detect

    rospy.loginfo("Images:{}".format(idx))
    cv2_image_bgr = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    img_full_path = "imgs/frames%d.png" % idx
    yolo_detect.run(cv2_image_bgr, idx, img_full_path)
    # cv2.imwrite("imgs/frame%d.jpg" % idx, cv_image)
    idx += 1

def listener(camera_publisher_topic):
    global yolo_detect
    # Initialize yolo
    opt = parse_opt()
    yolo_detect = YoloDetect(**vars(opt))
    

    rospy.init_node('cones_node_listener', anonymous=False)
    rospy.Subscriber(camera_publisher_topic, Image, callback)
    print("listener started")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    CAMERA_PUBLISHER_TOPIC = '/camera/color/image_raw'
    idx = 1
    # create a cv_bridge object
    bridge = CvBridge()
    try:
        listener(CAMERA_PUBLISHER_TOPIC)
    except rospy.ROSInterruptException:
        pass