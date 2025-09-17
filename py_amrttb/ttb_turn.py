import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from irobot_create_msgs.msg import IrIntensityVector
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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

        self.subscriber_ = self.create_subscription(
            Odometry,
            '/TTB08/odom',
            self.odom_callback,
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
        self.error = 0
        self.previous_error = 0
        self.integral = 0
        self.setpoint = [0.1, 0.2, 0.4]
        self.output = 0
        self.measured_value = 0
        self.cruise = 0
        
        self.count_three = 0
        self.dt = 0.1
        self.kp = 0.1
        self.ki = 0.001
        self.kd = 0.03

        self.x_vel = None
        self.ang_vel = 0

        # step 4
        self.accel = 0
        self.i = 0

    def controller_callback(self):
        if self.x_vel is None:
            return

        msg = Twist()

            
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
        if(self.cruise == 1):
            if self.distance >= 200:
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
            
  

        #step3 4
            print("speed")
            print(self.x_vel)
            print()

            
            if self.accel == 1:
                self.i += 1
            elif self.accel == -1:
                self.i -= 1
            if self.i == 3 or self.i == -1:
                self.i = 0

            print("setpoint")
            self.error = self.setpoint[self.i] - self.x_vel # speed difference
            print(self.i)
            print(self.setpoint[self.i])
            print()

            print("error")
            print(self.error)
            print()
            self.integral += self.error*(self.dt)
            print("integral")
            print(self.integral)
            print()
            self.derivative = (self.error - self.previous_error)/(self.dt)
            print("derivative")
            print(self.derivative)
            print()
            self.output += self.kp*self.error + self.ki*self.integral + self.kd*self.derivative
            print("output")
            print(self.output)
            print()
            self.previous_error = self.error
            print("updated previous error")
            print(self.previous_error)
            print()
            print()
            print()

            self.forward = self.output

            self.count_three += 1

            msg.linear.x  = self.forward
            
        # task left: set a single button of joystick, press it to loop through the three setpoint speed
        




        self.publisher_.publish(msg)
        self.count += 1

    def imu_callback(self, msg):
        # print angular velocity from imu message to console
        self.ang_vel = msg.angular_velocity.z
        #self.get_logger().info(f'Angular velocity: {self.ang_vel:0.4f}')


    #step 1
    def joy_callback(self, msg):
        # print angular velocity from imu message to console
        self.forward = msg.axes[4]
        self.angular = msg.axes[6]
        self.accel = msg.axes[7]
        if msg.buttons[0] == 1: # this makes it
            self.cruise = 0
        elif msg.buttons[0] == 0:
            self.cruise = 1
        #forward_vel = 
        #self.get_logger().info(f'joy_axes: {self.forward:0.10f}')


    #step 2
    def ir_callback(self, msg):
        # print angular velocity from imu message to console
        self.distance = max(msg.readings[3].value, msg.readings[4].value, msg.readings[2].value, msg.readings[5].value)
        
        #self.get_logger().info(f'ir_readings: {self.distance:0.10f}')

    
    #step3
    def odom_callback(self, msg):
        self.x_vel = msg.twist.twist.linear.x


def main(args=None):
    rclpy.init(args=args)

    ttb_controller = TTBController()

    rclpy.spin(ttb_controller)

    ttb_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()