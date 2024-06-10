import rclpy
import rclpy.qos
from rclpy.node import Node
import numpy as np
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
        self.yellow_salir = False

        self.vel_final = Twist()
        self.vel_linear = 0.0
        self.vel_angular = 0.0
        self.vel_senales_lin = 0.01
        self.vel_senales_ang = 0.0
        
        
        self.contador = Int32()
        self.indice_senal_ant = Int32()
       
        self.estados_senales = [False] * 8
        self.cont_senales = [0]  * 8

        self.timer_period = 0.2  #  self.estados_senales = [False] * 7seconds
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

        if self.red == True:
           self.get_logger().info('Parateeeee pero parateee')
           self.vel_final.linear.x = 0.0
           self.vel_final.angular.z = 0.0

           if self.green_p.data >= 0.4:
               self.green = True
               self.red = False
               self.yellow = False
           elif self.yellow_p.data >= 0.08 and self.yellow_p.data <= 1.5:
                self.yellow = True
                self.green = False
                self.red = False

        elif self.green == True:
            self.get_logger().info('Patitas pa que las quieres')
            self.vel_final.linear.x = self.vel_linear
            self.vel_final.angular.z = self.vel_angular

            if self.red_p.data >= 0.9:
               self.red = True
               self.green = False
               self.yellow = False
            elif self.yellow_p.data >= 0.08 and self.yellow_p.data <= 1.5:
                self.yellow = True
                self.green = False
                self.red = False
                
        elif self.yellow == True:
                self.get_logger().info('Avanzale que se pone el rojo')
                if self.vel_final.linear.x > 0.01:
                    self.vel_final.linear.x -= 0.01
                else:
                    self.vel_final.linear.x = 0.0

                if self.vel_final.angular.z > 0.01:
                    self.vel_final.angular.z -= 0.01
                else:
                    self.vel_final.angular.z = 0.0
                    
                if self.red_p.data >= 0.9:
                    self.red = True
                    self.green = False
                    self.yellow = False
                elif self.green_p.data >= 0.4:
                    self.green = True
                    self.red = False
                    self.yellow = False


        else:
           #self.get_logger().info('Nada nadotaaaa')
           self.vel_final.linear.x = 0.0
           self.vel_final.angular.z = 0.0
           
           
    def senales_vel(self):
            #self.get_logger().info('final: {}'.format(self.estados_senales))

            if self.estados_senales[0]:  # Stop sign
                self.vel_final.linear.x = 0.0
                self.vel_final.angular.z = 0.0
            elif self.estados_senales[1]:  # Give way
                self.vel_final.linear.x = max(0.005, self.vel_final.linear.x - 0.001)
                self.vel_final.angular.z = max(0.005, self.vel_final.angular.z - 0.001)
            elif self.estados_senales[2]:  # Straigth
                self.vel_final.linear.x = 0.03
                self.vel_final.angular.z =  0.0
            elif self.estados_senales[4]:  # Turn Left
                self.vel_final.linear.x = self.vel_linear
                self.vel_final.angular.z = 0.025
            elif self.estados_senales[5]:  # Turn Right
                self.vel_final.linear.x = self.vel_linear
                self.vel_final.angular.z =  - 0.025
            elif self.estados_senales[6]:  # Work in Progress
                self.vel_final.linear.x = max(0.005, self.vel_final.linear.x - 0.001)
                self.vel_final.angular.z = max(0.0, self.vel_final.angular.z - 0.001)
            else:
                # Resetear todos los estados a False si no se detecta ninguna señal
                self.estados_senales = [False] * 8
                self.vel_final.linear.x = self.vel_linear
                self.vel_final.angular.z = self.vel_angular
                
    def senales_vel_fuera(self):
            #self.get_logger().info('final: {}'.format(self.estados_senales))

            if self.estados_senales[0]:  # Stop sign
                self.vel_final.linear.x = 0.0
                self.vel_final.angular.z = 0.0
            elif self.estados_senales[1]:  # Give way
                self.vel_final.linear.x = max(0.005, self.vel_final.linear.x - 0.001)
                self.vel_final.angular.z = max(0.005, self.vel_final.angular.z - 0.001)
            elif self.estados_senales[2]:  # Straigth
                self.vel_final.linear.x = 0.03
                self.vel_final.angular.z =  0.0
            elif self.estados_senales[4]:  # Turn Left
                self.vel_final.linear.x = 0.02
                self.vel_final.angular.z = 0.025
            elif self.estados_senales[5]:  # Turn Right
                self.vel_final.linear.x = 0.02
                self.vel_final.angular.z =  - 0.025
            elif self.estados_senales[6]:  # Work in Progress
                self.vel_final.linear.x = max(0.005, self.vel_final.linear.x - 0.001)
                self.vel_final.angular.z = max(0.0, self.vel_final.angular.z - 0.001)
            else:
                # Resetear todos los estados a False si no se detecta ninguna señal
                self.estados_senales = [False] * 8
                self.vel_final.linear.x = self.vel_senales_lin
                self.vel_final.angular.z = self.vel_senales_ang
  

    def timer_callback(self): 
        
        if self.indice_senal is not None:
            if self.indice_senal.data != self.indice_senal_ant.data:
                self.get_logger().info('cero cerote')
                self.contador.data = 0
                self.cont_senales= [0] * 8
                self.estados_senales = [False] * 8
                self.indice_senal_ant.data = self.indice_senal.data
            else:
                self.contador.data += 1
            
            if self.indice_senal.data == 7:
                self.estados_senales = [False] * 8
                self.cont_senales= [0] * 8
            else:
                self.cont_senales[self.indice_senal.data] += 1
        
    
            
            
        if self.yellow_salir == True:
            self.get_logger().info('señales fuera pista')
            
            if self.contador.data == 0 or self.contador.data < 10 :
                self.vel_final.linear.x = self.vel_senales_lin
                self.vel_final.angular.z = self.vel_senales_ang
                
            elif self.contador.data > 70:
                self.vel_final.linear.x = self.vel_senales_lin
                self.vel_final.angular.z = self.vel_senales_ang
            else:
                
                #self.get_logger().info('antes: {}'.format(self.estados_senales))
                index = int(np.argmax(self.cont_senales))
                if self.cont_senales[index] == 0:
                    #self.get_logger().info('estados 0: {}'.format(self.estados_senales))
                    self.estados_senales = [False] * 8
                else: 
                    #self.get_logger().info('despues: {}'.format(self.estados_senales))
                    self.estados_senales[index] = True
                
                self.senales_vel_fuera()
        else:
            if self.contador.data == 0 or self.contador.data < 70 :
                self.vel_final.linear.x = self.vel_linear
                self.vel_final.angular.z = self.vel_angular
                
                if self.red_p.data >= 0.6:
                    self.red = True
                    self.colores_vel()
                elif self.green_p.data >= 0.4:
                    self.green = True
                    self.colores_vel()
                elif self.yellow_p.data >= 0.08 and self.yellow_p.data <= 1.5:
                    self.yellow = True
                elif self.yellow_p.data >= 1.8:
                    self.yellow_salir = True

        
            elif self.contador.data > 130:
                self.vel_final.linear.x = self.vel_linear
                self.vel_final.angular.z = self.vel_angular
                
                if self.red_p.data >= 0.6:
                    self.red = True
                    self.colores_vel()
                elif self.green_p.data >= 0.4:
                    self.green = True
                    self.colores_vel()
                elif self.yellow_p.data >= 0.08 and self.yellow_p.data <= 1.5:
                    self.yellow = True
                elif self.yellow_p.data >= 1.8:
                    self.yellow_salir = True
                
            else:
                self.get_logger().info('antes: {}'.format(self.estados_senales))
                index = int(np.argmax(self.cont_senales))
                if self.cont_senales[index] == 0:
                    self.get_logger().info('estados 0: {}'.format(self.estados_senales))
                    self.estados_senales = [False] * 8
                else: 
                    self.get_logger().info('despues: {}'.format(self.estados_senales))
                    self.estados_senales[index] = True
                
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
