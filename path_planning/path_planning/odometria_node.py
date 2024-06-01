import rclpy
from rclpy.node import Node 
import rclpy.qos
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import math as m


class Odometria(Node):
    def __init__(self):
        super().__init__('odometria_node')
        self.publisher_datoFinal = self.create_publisher(Pose2D,'DatoFinal', rclpy.qos.qos_profile_sensor_data)

        self.sub_vangD = self.create_subscription(Float32, 'VelocityEncR', self.odometria_vangD_callback, rclpy.qos.qos_profile_sensor_data)
        self.sub_vangI = self.create_subscription(Float32, 'VelocityEncL', self.odometria_vangI_callback, rclpy.qos.qos_profile_sensor_data)
        
        
        # variables 
        
        self.VangD = Float32()
        self.VangI = Float32()  


        self.VangT = Float32()
        self.VlT = Float32()

        self.Vl_x = Float32()
        self.Vl_y = Float32()

        self.msg_distancia = Float32()
        self.msg_pos_x = 1.2
        self.msg_pos_y = 2.4
        self.msg_theta = 0.0
        
        
        self.msg_pos = Pose2D()
        
        self.msg_vel = Float32()
    
        self.radio = Float32()
        self.long = Float32()

        self.radio = 0.05
        self.long = 0.18

        # variables timer 
        self.timer_period  = Float32()
        self.timer_period  = 0.01


        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Ready to calculate :D!!')



    def odometria_vangD_callback(self, msg):
        self.VangD.data = msg.data
        
        
    def odometria_vangI_callback(self,msg):
        self.VangI.data = msg.data
    

    def timer_callback(self):
        
        ## velocidad angular 
        self.VangT = self.radio  * (self.VangD.data - self.VangI.data)/self.long
        
        self.msg_theta += self.VangT *self.timer_period
        
        ## velocidad lineal   
        self.VlT= self.radio * (self.VangD.data + self.VangI.data)/2.0

        self.Vl_x = self.VlT * m.cos(self.msg_theta)
        self.Vl_y = self.VlT * m.sin(self.msg_theta)


        ## distancia 
        self.msg_distancia.data += self.VlT*self.timer_period
        
        ## Posicion
        self.msg_pos_x += self.Vl_x*self.timer_period
        self.msg_pos_y += self.Vl_y*self.timer_period
        
        self.msg_pos.x = self.msg_pos_x
        self.msg_pos.y = self.msg_pos_y
        self.msg_pos.theta = self.msg_theta


        ## Publicar  
        self.publisher_datoFinal.publish(self.msg_pos)



        


def main(args=None):
    rclpy.init(args=args)
    od = Odometria()
    rclpy.spin(od)
    od.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
