import rclpy
import rclpy.qos
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool, Int32
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge, CvBridgeError




class TopLevel(Node):
    def __init__(self):
        super().__init__('top_level_node')

        self.sub_red = self.create_subscription(Float32, '/img_propeties/red/density', self.red_density, rclpy.qos.qos_profile_sensor_data)
        self.sub_green = self.create_subscription(Float32, '/img_propeties/green/density', self.green_density, rclpy.qos.qos_profile_sensor_data)
        self.sub_yellow = self.create_subscription(Float32, '/img_propeties/yellow/density', self.yellow_density, rclpy.qos.qos_profile_sensor_data)
        self.sub_vel_line = self.create_subscription(Twist, 'vel_line', self.vel_line_callback, rclpy.qos.qos_profile_sensor_data)
        
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_sensor_data)
        self.pub_cont = self.create_publisher(Int32, 'contador', rclpy.qos.qos_profile_sensor_data)
        self.sub_senal = self.create_subscription(Int32, 'senal', self.senal_callback, rclpy.qos.qos_profile_sensor_data)

        # Variables
        self.red_p = Float32()
        self.green_p = Float32()
        self.yellow_p = Float32()
        self.indice_senal = Int32()
        
        self.yellow = False
        self.green = False
        self.red = False

        self.vel_final = Twist()
        self.vel_linear = 0.0
        self.vel_angular = 0.0
        
        
        self.contador = Int32()
        self.indice_senal_ant = Int32()
        self.estados_senales = [False] * 7

        self.timer_period = 0.2  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def green_density(self, msg):
        self.green_p.data = msg.data

    def red_density(self, msg):
        self.red_p.data = msg.data

    def yellow_density(self, msg):
        self.yellow_p.data = msg.data


    def vel_line_callback(self, msg):
        self.vel_linear = msg.linear.x
        self.vel_angular = msg.angular.z

    def senal_callback(self, msg):
        self.indice_senal.data = msg.data

    def colores_vel(self):
        
        if self.red_p.data >= 15.0:
            self.red = True
        elif self.green_p.data >= 15.0:
            self.green = True
        elif self.yellow_p.data >= 15.0:
            self.yellow = True

        if self.red == True:
           #self.get_logger().info('Parateeeee pero parateee')
           self.vel_final.linear.x = 0.0
           self.vel_final.angular.z = 0.0

           if self.green_p.data >= 15.0:
               self.green = True
               self.red = False
               self.yellow = False
           elif self.yellow_p.data >= 15.0:
               self.green = False
               self.red = False
               self.yellow = True

        elif self.green == True:
            #self.get_logger().info('Patitas pa que las quieres')
            self.vel_final.linear.x = self.vel_linear
            self.vel_final.angular.z = self.vel_angular

            if self.red_p.data >= 15.0:
               self.red = True
               self.green = False
               self.yellow = False
            elif self.yellow_p.data >= 15.0:
               self.green = False
               self.red = False
               self.yellow = True

        elif self.yellow == True:
            if self.vel_final.linear.x > 0.01:
               self.vel_final.linear.x -= 0.01
            else:
               self.vel_final.linear.x = 0.0

            if self.vel_final.angular.z > 0.01:
               self.vel_final.angular.z -= 0.01
            else:
               self.vel_final.angular.z = 0.0

            if self.red_p.data >= 15.0:
               self.red = True
               self.green = False
               self.yellow = False
            elif self.green_p.data >= 15.0:
               self.green = True
               self.red = False
               self.yellow = False

        else:
           #self.get_logger().info('Nada nadotaaaa')
           self.vel_final.linear.x = 0.0
           self.vel_final.angular.z = 0.0
           
           
    def senales_vel(self):

            if self.estados_senales[0]:  # Stop sign
                self.vel_final.linear.x = 0.0
                self.vel_final.angular.z = 0.0
            elif self.estados_senales[1]:  # Give way
                self.vel_final.linear.x = self.vel_linear + 0.04
                self.vel_final.angular.z = self.vel_angular
            elif self.estados_senales[2]:  # Straigth
                self.vel_final.linear.x = self.vel_linear
                self.vel_final.angular.z = self.vel_angular
            elif self.estados_senales[4]:  # Turn Left
                self.vel_final.linear.x = self.vel_linear
                self.vel_final.angular.z = - 0.01
            elif self.estados_senales[5]:  # Turn Right
                self.vel_final.linear.x = self.vel_linear
                self.vel_final.angular.z = 0.1
            elif self.estados_senales[6]:  # Work in Progress
                self.vel_final.linear.x = max(0.0, self.vel_final.linear.x - 0.01)
                self.vel_final.angular.z = max(0.0, self.vel_final.angular.z - 0.01)
            else:
                # Resetear todos los estados a False si no se detecta ninguna señal
                self.estados_senales = [False] * 7
                self.vel_final.linear.x = self.vel_linear
                self.vel_final.angular.z = self.vel_angular
  

    def timer_callback(self): 
        
        if self.indice_senal is not None:
            # Establecer el estado de la señal detectada a True
            self.estados_senales[self.indice_senal.data] = True
        
            if self.indice_senal.data != self.indice_senal_ant.data:
                self.contador.data = 0
                self.indice_senal_ant.data = self.indice_senal.data
            else:
                self.contador.data += 1
                


        if self.contador.data == 0 or self.contador.data < 20 :
            self.colores_vel()
        elif self.contador.data > 40:
            self.vel_final.linear.x = self.vel_linear
            self.vel_final.angular.z = self.vel_angular
        else:
            self.senales_vel()
        
        
        self.pub_vel.publish(self.vel_final)
        self.pub_cont.publish(self.contador)


def main(args=None):
    rclpy.init(args=args)
    tl = TopLevel()
    rclpy.spin(tl)
    tl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()