#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32, Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import PointCloud2, Temperature
from first_package.msg import kartTemperatures
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import rosbag
import random

def talker():
    lidar_pub = rospy.Publisher('velodyne_points', PointCloud2, queue_size=1)
    speed_pub = rospy.Publisher('car_speeds', Float32MultiArray, queue_size=1)
    # Temperatures test topics
    temp_pub = rospy.Publisher('temperatures_custom', kartTemperatures, queue_size=1)
    temp_pub2 = rospy.Publisher('temperatures_std', Temperature, queue_size=1)
    temp_pub3 = rospy.Publisher('temperatures_int', Int32, queue_size=1)
    temp_arr_pub = rospy.Publisher('temperatures', Int32MultiArray, queue_size=1)
    trajectory_pub = rospy.Publisher('markers/trajectory', MarkerArray, queue_size=1)

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
        lidar_pub.publish(msg)
        trajectory_pub.publish(trajectoryPath(t, "velodyne", 20, 0.5, 0.2, 2.0, 1.8))

        actual_speed = float(random.randint(45, 50)) # Randomly change of speed as a test
        target_speed = actual_speed + 10.3
        publish_speeds(speed_pub, actual_speed, target_speed)

        if idx % 10 == 0: # change the temperature every 1 second
            motor_temperature = random.randint(70, 80) # Randomly change of temperature as a test
            cpu_temperature = random.randint(60, 80) # Randomly change of temperature as a test
            publish_temperatures(temp_pub, temp_pub2, temp_pub3, temp_arr_pub, motor_temperature, cpu_temperature)        

        rate.sleep()
    bag.close()

def publish_temperatures(temp_pub: rospy.Publisher, temp_pub2: rospy.Publisher, temp_pub3: rospy.Publisher, temp_arr_pub: rospy.Publisher, motor_temperature: int, cpu_temperature: int):
    temperature_msg = kartTemperatures()
    temperature_msg.motor_temperature = motor_temperature
    temperature_msg.cpu_temperature = cpu_temperature 
    temp_pub.publish(temperature_msg)

    temperature_msg = Temperature()
    temperature_msg.temperature = motor_temperature
    rospy.loginfo(temperature_msg)
    temp_pub2.publish(temperature_msg)

    temp_pub3.publish(Int32(motor_temperature))

    temperature_arr = [motor_temperature, cpu_temperature]
    temp_arr_pub.publish(Int32MultiArray(data=temperature_arr))

def publish_speeds(speed_pub: rospy.Publisher, actual_speed: float, target_speed: float):
    speed_msg = Float32MultiArray()
    speed_msg.data = [actual_speed, target_speed]
    speed_pub.publish(speed_msg)

def trajectoryPath(timestamp, frame_id, amount, separation, size, vehicle_length, vehicle_height):
    markerArray = MarkerArray()
    for i in range(amount):
        marker = Marker()
        marker.header.stamp = timestamp
        marker.header.frame_id = frame_id
        marker.id = i
        marker.lifetime = rospy.Duration(0.1)
        marker.ns = "my_namespace"
        marker.type = Marker.POINTS
        marker.pose.orientation.w = 0.0
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        point = Point()
        point.x = 0.0
        point.y = vehicle_length + (i * separation)
        point.z = -vehicle_height
        marker.points
        marker.points.append(point)
        markerArray.markers.append(marker)      
    return markerArray


if __name__ == '__main__':
    #RECORDED_ROS_BAG_PATH = '/home/bakr/Downloads/UrbanNav-HK_TST-20210517_sensors.bag'
    RECORDED_ROS_BAG_PATH = 'UrbanNav-HK_CHTunnel-20210518_sensors.bag'
    try:
        talker()
    except rospy.ROSInterruptException as e:
        print("Error:", e)