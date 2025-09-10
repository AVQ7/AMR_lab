import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from irobot_create_msgs.msg import IrIntensityVector
from geometry_msgs.msg import Twist

import random
import numpy as np

class TTBController(Node):

    def __init__(self):
        super().__init__('ttb_turn')
        # TODO: Update 'TTBXX/' topics with your team's robot number (e.g. TTB02 for robot 2)

        # setup publisher for Twist msg to /TTBXX/cmd_vel with buffer size = 10
        self.publisher_ = self.create_publisher(
            Twist,
            '/TTB08/cmd_vel',
            10
        )

        # setup subscriber to Imu ms
        # g from /TTB08/imu with buffer size = 10
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscriber_ = self.create_subscription(
            Imu,
            '/TTB08/imu',
            self.imu_callback,
            qos_profile

        )

    
        self.subscriber_ = self.create_subscription(
            Joy,
            '/TTB08/joy',
            self.joy_callback,
            qos_profile

        )

        self.subscriber_ = self.create_subscription(
            IrIntensityVector,
            '/TTB08/ir_intensity',
            self.ir_callback,
            qos_profile

        )

        # setup controller to run at 10hz (period=.1s) and call method controller_callback
        timer_period = 0.1
        self.timer   = self.create_timer(timer_period, self.controller_callback)

        # when count >=30, stop moving vehicle
        self.count = 0

        #step1
        self.forward = 0
        self.angular = 0

        #step 2
        self.distance = 0
        self.turn = False
        self.turnDirection = 0
        self.random = 0
        self.time = 0

        #step3
        self.previous_error = 0
        self.integral = 0
        self.setpoint_two = 0.2
        self.setpoint_four = 0.4
        self.output = 0
        self.measured_value = 0
        self.count_three = 0
        self.dt = 5
        self.kp = 0.1
        self.ki = 0.001
        self.kd = 0.03


    def controller_callback(self):
        # create msg which makes TTB speed 0.1 m/s and angular velocity 0.4rad/s
        # if count >=30 (~3 seconds), stop moving

        msg = Twist()
        #if self.count < 30:
         #   msg.linear.x  = 0.1
          #  msg.angular.z = 0.4
            
        #step 1 
        if self.forward !=0:
            msg.linear.x = 0.3*self.forward
        else:
            msg.linear.x  = 0.0

        if self.angular !=0:
            msg.angular.z= 0.3*self.angular
        else:
            msg.angular.z = 0.0



        #step 2

        if self.distance >= 500:
            self.turn = True
            self.time = 0
        else:
            self.turn = False
            self.random = float(random.uniform(-1,1))
            if self.random > 0:
                self.turnDirection = 1
            else:
                self.turnDirection = -1
        
        if self.time < 30 and self.turn == True:
            msg.angular.z = float(self.turnDirection) * 5
            self.time += 1

  

        #step3
        # msg.linear.x = 0.1

        # if self.count_three ==0 :
        #     self.count_three = self.dt
        #     error = self.setpoint_two - self.measured_value
        #     self.integral += self.error*(self.dt/10)
        #     self.derivative = (self.error - self.previous_error)/(self.dt/10)
        #     self.output += self.kp*self.error + self.ki*self.integral + self.kd*self.derivative
        #     self.previous_error = error
        
        # self.count_three -= 1
        
        




        self.publisher_.publish(msg)
        self.count += 1

    def imu_callback(self, msg):
        # print angular velocity from imu message to console
        ang_vel = msg.angular_velocity.z
        self.get_logger().info(f'Angular velocity: {ang_vel:0.4f}')


    #step 1
    def joy_callback(self, msg):
        # print angular velocity from imu message to console
        self.forward = msg.axes[4]
        self.angular = msg.axes[6]
        
        #forward_vel = 
        self.get_logger().info(f'joy_axes: {self.forward:0.10f}')


    #step 2
    def ir_callback(self, msg):
        # print angular velocity from imu message to console
        self.distance = max(msg.readings[3].value, msg.readings[4].value, msg.readings[2].value, msg.readings[5].value)
        
        self.get_logger().info(f'ir_readings: {self.distance:0.10f}')

    '''
    #step 3
    def odom_callback(self, msg):
        self.

    '''

def main(args=None):
    rclpy.init(args=args)

    ttb_controller = TTBController()

    rclpy.spin(ttb_controller)

    ttb_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()