#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32, Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import PointCloud2, Temperature
from first_package.msg import kartTemperatures
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from first_package.msg import kartPID
from first_package.msg import kartCurrents
from first_package.msg import kartMotorRPM
import rosbag
import random

class ROS_Vis_Publisher:
    def __init__(self) -> None:
        self.lidar_pub = rospy.Publisher('velodyne_points', PointCloud2, queue_size=1)
        self.speed_pub = rospy.Publisher('car_speeds', Float32MultiArray, queue_size=1)
        # Temperatures test topics
        self.temp_pub = rospy.Publisher('temperatures_custom', kartTemperatures, queue_size=1)
        self.temp_pub2 = rospy.Publisher('temperatures_std', Temperature, queue_size=1)
        self.temp_pub3 = rospy.Publisher('temperatures_int', Int32, queue_size=1)
        self.temp_arr_pub = rospy.Publisher('temperatures', Int32MultiArray, queue_size=1)
        self.trajectory_pub = rospy.Publisher('markers/trajectory', MarkerArray, queue_size=1)
        # PID test topics
        self.PID_pub = rospy.Publisher('PID', kartPID, queue_size=1)
        # Current test topics
        self.current_pub = rospy.Publisher('currents', kartCurrents, queue_size = 1)
        # Motor RPM test topics
        self.motor_RPM_pub = rospy.Publisher('motor_RPM', kartMotorRPM, queue_size = 1)

    def talker(self):
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
            self.publish_100ms_cyclic_function(msg)
            if idx % 10 == 0:
                self.publish_1_second_cyclic_function()          

            rate.sleep()
        bag.close()

    def publish_100ms_cyclic_function(self, lidar_msg):
        self.lidar_pub.publish(lidar_msg)
        # self.trajectory_pub.publish(trajectoryPath(t, "velodyne", 20, 0.5, 0.2, 2.0, 1.8)) # TODO: AB: To be replaced with the code from Andreas

        # Speeds Publishers
        actual_speed = float(random.randint(45, 50)) # Randomly change of speed as a test
        target_speed = actual_speed + 10.3
        self.publish_speeds(self.speed_pub, actual_speed, target_speed)

        # RPM Publisher
        motor_RPM = random.randint(1000,2000)
        self.publish_motor_RPM(self.motor_RPM_pub, motor_RPM)



    def publish_1_second_cyclic_function(self):
        motor_temperature = random.randint(70, 80) # Randomly change of temperature as a test
        cpu_temperature = random.randint(60, 80) # Randomly change of temperature as a test
        self.publish_temperatures(self.temp_pub, self.temp_pub2, self.temp_pub3, self.temp_arr_pub, motor_temperature, cpu_temperature)

        # PID Publishers
        P = random.randint(0,5)
        I = random.randint(10,15)
        D = random.randint(20,25)
        self.publish_PID(self.PID_pub, P, I, D)

        # Currents publishers
        steering_servo_current = random.randint(0,20) / 20.0
        brakes_servo_current = random.randint(0,20) / 20.0
        motor_DC_current = random.randint(0,20) / 20.0
        self.publish_currents(self.current_pub, steering_servo_current, brakes_servo_current, motor_DC_current)

    def publish_temperatures(self, temp_pub: rospy.Publisher, temp_pub2: rospy.Publisher, temp_pub3: rospy.Publisher, temp_arr_pub: rospy.Publisher, motor_temperature: int, cpu_temperature: int):
        # Temperature publishers
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



    def publish_speeds(self, speed_pub: rospy.Publisher, actual_speed: float, target_speed: float):
        speed_msg = Float32MultiArray()
        speed_msg.data = [actual_speed, target_speed]
        speed_pub.publish(speed_msg)

    def publish_PID(self, PID_pub: rospy.Publisher, P: float, I: float, D: float):
        PID_msg = kartPID()
        PID_msg.P = P
        PID_msg.I = I
        PID_msg.D = D
        PID_pub.publish(PID_msg)

    def publish_currents(self, current_pub: rospy.Publisher, steering_servo_current: float, brakes_servo_current: float, motor_DC_current: float):
        current_msg = kartCurrents()
        current_msg.steering_servo_current = steering_servo_current
        current_msg.brakes_servo_current = brakes_servo_current
        current_msg.motor_DC_current = motor_DC_current
        current_pub.publish(current_msg)

    def publish_motor_RPM(self, motor_RPM_pub: rospy.Publisher, motor_RPM: int):
        motor_RPM_msg = kartMotorRPM()
        motor_RPM_msg.motor_RPM = motor_RPM
        motor_RPM_pub.publish(motor_RPM_msg)

    def trajectoryPath(self, timestamp, frame_id, amount, separation, size, vehicle_length, vehicle_height):
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
    RECORDED_ROS_BAG_PATH = '/mnt/Ubuntu/UrbanNav-HK_TST-20210517_sensors.bag'
    try:
        ros_vis_publisher = ROS_Vis_Publisher()
        ros_vis_publisher.talker()
    except rospy.ROSInterruptException as e:
        print("Error:", e)