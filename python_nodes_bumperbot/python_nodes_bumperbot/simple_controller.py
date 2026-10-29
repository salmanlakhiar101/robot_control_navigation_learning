#!/usr/bin/env python3
import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
 
 
class SimpleController(Node): 
    def __init__(self):
        super().__init__("simple_controller") 

        #Making Parameters for wheel radius and seperation
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        #Using those parameters and storing them in variables
        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        #Printing the values of wheel radius and seperation
        self.get_logger().info("Using wheel radius: %f" %self.wheel_radius_)
        self.get_logger().info("Using wheel separation: %f" %self.wheel_separation_)

        #---------- Inverse Kinematics ----------

        #Creating a publisher for publishing wheel velocities
        self.wheel_cmd_pub_  = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)

        #Creating a subscriber for a topic where joystick publisher the velocity
        self.vel_sub_ = self.create_subscription(TwistStamped, "bumperbot_controller/cmd_vel", self.velCallBack, 10)

        #Conversion matrix using numpy
        self.speed_conversion_ = np.array([[(self.wheel_radius_/2), (self.wheel_radius_/2)],
                                          [(self.wheel_radius_/self.wheel_separation_), -(self.wheel_radius_/self.wheel_separation_)]])

        #Diplaying the conversion matrix
        self.get_logger().info("The conversion matrix is: %s" %self.speed_conversion_)

        #---------- Forward Kinematics ----------

        # For Velocity Calculation from Wheel
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()

        #Creating a joint subscriber that will subscribe to joint_state topic (for simulated robot, otherwise it will get data directly form wheel encoders)
        #It will get the position of wheels
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallBack, 10)

    #---------- Inverse Kinematics ----------

    #Defining the callback function for vel_sub
    def velCallBack(self, msg: TwistStamped):
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])

        #Wheel speed can be calculated by multiplying robot_speed matrix with the inverse of speed_conversion matrix for differential drive robot
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)

        #Now the speed value that will be published
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[0, 0], wheel_speed[1, 0]]

        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    #---------- Forward Kinematics ----------

    #Defining callback function for the Joint State Subscriber
    def jointCallBack(self, msg: JointState):

        #self.get_logger().info("Joint callback triggered!")

        left_index = msg.name.index("base_left_joint")
        right_index = msg.name.index("base_right_joint")

        dp_left = msg.position[left_index] - self.left_wheel_prev_pos_
        dp_right = msg.position[right_index] - self.right_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        #Updating previous states
        self.left_wheel_prev_pos_ = msg.position[left_index]
        self.right_wheel_prev_pos_ = msg.position[right_index]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        linear = ((self.wheel_radius_ * fi_left) + (self.wheel_radius_ * fi_right)) / 2
        angular = ((self.wheel_radius_ * fi_right) - (self.wheel_radius_ * fi_left)) / self.wheel_separation_ 

        self.get_logger().info("Linear Velocity: %f, Angular Velocity: %f" %(linear, angular))




 
def main(args=None):
    rclpy.init(args=args)
    node = SimpleController() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
