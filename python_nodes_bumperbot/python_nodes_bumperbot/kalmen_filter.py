#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class KalmenFilter(Node):
    def __init__(self):
        super().__init__("kalmen_filter")

        #Veriables
        self.mean_ = 0.0
        self.variance_ = 1000.0

        self.imu_angular_z = 0.0
        self.last_angular_z = 0.0
        self.is_first_odom_ = True

        self.motion_ = 0.0

        self.kalmen_odom_ = Odometry()

        self.motion_variance_ = 4.0
        self.measurement_variance_ = 0.5

        #Creating Subscriber for IMU
        self.imu_sub_ = self.create_subscription(Imu, "imu/out", self.imuCallBack, 10)

        #Creating Subscriber for Encoder data
        self.odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom_noisy", self.odomCallBack, 10)

        #Creating the publisher that will publishe the filtered odometery ddata of Robot
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom_kalmen", 10)

    #Defination of IMU Callback function
    def imuCallBack(self, imu: Imu):
        self.imu_angular_z = imu.angular_velocity.z

        
    # Function Defination for measurementUpdate function
    def measurementUpdate(self):
        self.mean_ = ((self.measurement_variance_ * self.mean_) + (self.variance_ * self.imu_angular_z)) / (self.variance_ + self.measurement_variance_)
        self.variance_ = (self.variance_ * self.measurement_variance_) / (self.variance_ + self.measurement_variance_)


    # Function Defination for statePrediction function
    def statePrediction(self):
        self.mean_ = self.mean_ + self.motion_
        self.variance_ = self.variance_ + self.motion_variance_


    #Defination of Odom Callback Function
    def odomCallBack(self, odom: Odometry):
        self.kalmen_odom_ = odom

        
        if self.is_first_odom_:
            self.mean_ = odom.twist.twist.angular.z
            self.last_angular_z = odom.twist.twist.angular.z

            self.is_first_odom_ = False
            return

        self.motion_ = odom.twist.twist.angular.z - self.last_angular_z

        self.statePrediction()

        self.measurementUpdate()

        #Publishing the updated result
        self.kalmen_odom_.twist.twist.angular.z = self.mean_

        self.odom_pub_.publish(self.kalmen_odom_)



def main(args=None):
    rclpy.init(args=args)
    node = KalmenFilter() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
