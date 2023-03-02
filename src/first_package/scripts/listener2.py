#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Temperature
from first_package.msg import kartTemperatures

def callback(data):
    print("hello")
    rospy.loginfo("Motor Temperature:{}, CPU Temperature: {}".format(data.motor_temperature, data.cpu_temperature))

def listener():
    rospy.init_node('custom_listener', anonymous=False)
    rospy.Subscriber("temperatures", kartTemperatures, callback)
    print("listener started")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass