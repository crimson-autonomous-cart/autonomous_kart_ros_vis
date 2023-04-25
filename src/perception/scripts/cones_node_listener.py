#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Temperature
from first_package.msg import kartTemperatures
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def callback(data):
    global idx
    rospy.loginfo("Images:{}".format(idx))
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    # cv2.imwrite("imgs/frame%d.jpg" % idx, cv_image)
    idx += 1

def listener(camera_publisher_topic):
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