#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import rosbag

def talker():
    pub = rospy.Publisher('velodyne_points', PointCloud2, queue_size=20)
    rospy.init_node('talker', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    bag = rosbag.Bag(RECORDED_ROS_BAG_PATH)
    for idx, (topic, msg, t) in enumerate(bag.read_messages(topics=['/velodyne_points'])):
        if idx < 75:
            continue # The first 75 frames are not useful (The car is stationary)
        if rospy.is_shutdown():
            break
        # print(msg.data)
        print("published frame number:", idx, " with timestamp: ", t)
        # rospy.loginfo(msg.data)
        pub.publish(msg)
        rate.sleep()
    bag.close()

if __name__ == '__main__':
    RECORDED_ROS_BAG_PATH = '/home/bakr/Downloads/UrbanNav-HK_TST-20210517_sensors.bag'
    try:
        talker()
    except rospy.ROSInterruptException as e:
        print("Error:", e)
