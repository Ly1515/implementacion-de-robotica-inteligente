import rclpy
import rclpy.qos
from rclpy.node import Node
import math
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point, Twist
import cv2
from cv_bridge import CvBridge, CvBridgeError



class TopLevel(Node):
    def __init__(self):
        super().__init__('top_level_node')

        self.sub_red = self.create_subscription(Float32, '/img_propeties/red/density', self.red_density, rclpy.qos.qos_profile_sensor_data)
        self.sub_green = self.create_subscription(Float32, '/img_propeties/green/density',self.green_density, rclpy.qos.qos_profile_sensor_data)
        self.sub_vel_line = self.create_subscription(Twist, 'vel_line', self.vel_line_callback, rclpy.qos.qos_profile_sensor_data)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_sensor_data)
        self.sub_stop = self.create_subscription(Bool, 'stop', self.stop_callback, rclpy.qos.qos_profile_sensor_data)


        #Variables

        self.red_p = Float32()
        self.green_p = Float32()
        self.green = False
        self.red = False
        self.stop = Bool()

        self.vel_final = Twist()
        self.vel_linear =  0.0
        
        # variables timer
        self.timer_period = Float32()
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
       # self.get_logger().info('Ready image :D!!')

    def green_density(self, msg):
        self.green_p.data  = msg.data

    def red_density(self, msg):
        self.red_p.data = msg.data

    def stop_callback(self, msg):
        self.stop.data = msg.data

    def vel_line_callback(self, msg):
        #self.get_logger().info('Received image :D!!')
        self.vel_linear =  msg.linear.x
        self.vel_angular =  msg.angular.z

 
    def timer_callback(self):
        # Colores

        if self.red_p.data >= 15.0:
            self.red = True
        elif self.green_p.data >= 15.0:
            self.green = True

        if self.red == True:
           #self.get_logger().info('Parateeeee pero parateee')
           self.vel_final.linear.x = 0.0
           self.vel_final.angular.z = 0.0
           
           if self.green_p.data >= 15.0:
               self.green = True
               self.red = False

        elif self.green == True:
           #self.get_logger().info('Patitas pa que las quieres')
           self.vel_final.linear.x = self.vel_linear
           self.vel_final.angular.z = self.vel_angular

           if self.red_p.data >= 15.0:
               self.red = True
               self.green = False

        elif self.stop.data == True:
           self.vel_final.linear.x = 0.0
           self.vel_final.angular.z = 0.0

        else:
           #self.get_logger().info('Nada nadotaaaa')
           self.vel_final.linear.x = 0.0
           self.vel_final.angular.z = 0.0


        # Publisher
        self.pub_vel.publish(self.vel_final)

def main(args=None):
    rclpy.init(args=args)
    tl = TopLevel()
    rclpy.spin(tl)
    tl.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main()
