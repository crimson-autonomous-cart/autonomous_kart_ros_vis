#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ROS_Cones_Test_Publisher:
    def __init__(self, recorded_ros_bag_path, rate_hz=5):
        self.recorded_ros_bag_path = recorded_ros_bag_path

        rospy.init_node('camera_test_publisher', anonymous=True)
        self.camera_pub = rospy.Publisher(CAMERA_PUBLISHER_TOPIC, Image, queue_size=1)
        self.rate = rospy.Rate(rate_hz)
        
    def talker(self):
        # create a cv_bridge object
        bridge = CvBridge()
        bag = rosbag.Bag(self.recorded_ros_bag_path)
        for idx, (topic, msg, t) in enumerate(bag.read_messages(topics=[CAMERA_PUBLISHER_TOPIC])):
            if rospy.is_shutdown():
                break
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # cv2.imwrite("imgs/frame%d.jpg" % idx, cv_image)
            print("published frame number:", idx, " with timestamp: ", t)
            self.rate.sleep()
        bag.close()

if __name__ == '__main__':
    TEST_ROS_BAG_PATH = "/mnt/Ubuntu/rosbags/2023-04-20-11-03-23.bag"
    CAMERA_PUBLISHER_TOPIC = '/camera/color/image_raw'

    ros_cones_test_publisher = ROS_Cones_Test_Publisher(TEST_ROS_BAG_PATH)
    try:
        ros_cones_test_publisher.talker()
    except rospy.ROSInterruptException:
        pass
