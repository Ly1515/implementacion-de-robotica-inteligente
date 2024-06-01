import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
import math
import numpy as np
import yaml

class pura_persecucion_node(Node):
    def __init__(self):
        super().__init__('pura_persecucion_node')

        self.pos_global_subscription = self.create_subscription(Pose2D, 'pos_global', self.pos_global_callback, rclpy.qos.qos_profile_sensor_data)

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', rclpy.qos.qos_profile_sensor_data)

        self.cmd_vel = Twist()

        self.k = 0.0
        self.alpha = 0.0

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_theta = 0.0

        self.angular_velocity = 0.0

        self.seleccion = 2
        self.waypoints_x = []
        self.waypoints_y = []
        self.n = 0
        self.linear_velocity = 0.0

        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_velocity', rclpy.Parameter.Type.DOUBLE),
                ('linea.waypoints_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('linea.waypoints_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('linea.n', rclpy.Parameter.Type.INTEGER),
                ('cambio_carril.waypoints_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('cambio_carril.waypoints_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('cambio_carril.n', rclpy.Parameter.Type.INTEGER),
                ('cuadrado.waypoints_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('cuadrado.waypoints_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('cuadrado.n', rclpy.Parameter.Type.INTEGER),
                ('triangulo.waypoints_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('triangulo.waypoints_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('triangulo.n', rclpy.Parameter.Type.INTEGER)
            ]
        )

        self.timer_period = 0.1 #10Hz
        self.proc = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('New Signal generator node succesfully initialized')

    def pos_global_callback(self,msg):
        self.pos_x = msg.x
        self.pos_y = msg.y
        self.pos_theta = msg.theta

    def timer_callback(self):
        self.linear_velocity = self.get_parameter('linear_velocity').get_parameter_value().double_value

        if self.seleccion == 0:
                self.waypoints_x = self.get_parameter('linea.waypoints_x').get_parameter_value().double_array_value
                self.waypoints_y = self.get_parameter('linea.waypoints_y').get_parameter_value().double_array_value
                self.n = self.get_parameter('linea.n').get_parameter_value().integer_value
        elif self.seleccion == 1:
                self.waypoints_x = self.get_parameter('cambio_carril.waypoints_x').get_parameter_value().double_array_value
                self.waypoints_y = self.get_parameter('cambio_carril.waypoints_y').get_parameter_value().double_array_value
                self.n = self.get_parameter('cambio_carril.n').get_parameter_value().integer_value
        elif self.seleccion == 2:
                self.waypoints_x = self.get_parameter('cuadrado.waypoints_x').get_parameter_value().double_array_value
                self.waypoints_y = self.get_parameter('cuadrado.waypoints_y').get_parameter_value().double_array_value
                self.n = self.get_parameter('cuadrado.n').get_parameter_value().integer_value
        elif self.seleccion == 3 :
                self.waypoints_x = self.get_parameter('triangulo.waypoints_x').get_parameter_value().double_array_value
                self.waypoints_y = self.get_parameter('triangulo.waypoints_y').get_parameter_value().double_array_value
                self.n = self.get_parameter('triangulo.n').get_parameter_value().integer_value
        else:
                self.waypoints_x = 0
                self.waypoints_y = 0
                self.n = 0


        for i in range(self.n):
                dx = self.waypoints_x[i] - (self.pos_x)
                dy = self.waypoints_y[i] - (self.pos_y)

                orientation_difference = math.atan2(dy, dx)

                while orientation_difference > math.pi:
                    orientation_difference -= 2 * math.pi
                while orientation_difference < -math.pi:
                    orientation_difference += 2 * math.pi

                distance_to_target = math.sqrt(dx**2 + dy**2)

                self.alpha = np.arctan2((dy),(dx)) - self.pos_theta

                self.k = (2.0*0.18*math.sin(self.alpha))/0.4

                if distance_to_target > 0.1:
                    self.linear_velocity = 0.1
                    self.angular_velocity = math.atan2(self.k,1)
                else:
                    self.linear_velocity = 0.0
                    self.angular_velocity = 0.0

                self.cmd_vel.linear.x = self.linear_velocity
                self.cmd_vel.angular.z = self.angular_velocity

        self.cmd_vel_publisher.publish(self.cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    m_s = pura_persecucion_node()

    rclpy.spin(m_s)
    m_s.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
