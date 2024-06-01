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



class LineDetection(Node):
    def __init__(self):
        super().__init__('line_detection_node')

        self.sub_image = self.create_subscription(Image,'/video_source/raw', self.image_callback, rclpy.qos.qos_profile_sensor_data)
        self.pub_vel_line =  self.create_publisher(Twist, 'vel_line', rclpy.qos.qos_profile_sensor_data)
        self.prueba = self.create_publisher(Image, 'prueba', rclpy.qos.qos_profile_sensor_data)
        self.pub_stop = self.create_publisher(Bool, 'stop', rclpy.qos.qos_profile_sensor_data)


        #Variables
        self.bridge = CvBridge()

        self.minLineLength = 10
        self.maxLineGap = 10
        self.theta = 0
        self.mitad = 300
        self.k = 0.05

        self.img_msg = Image()
        self.img = Image()

        self.stop = False
        self.v_lineal = 0.02
        self.v_ang = 0.0
        self.vel = Twist()


        # variables timer
        self.timer_period = Float32()
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
       # self.get_logger().info('Ready image :D!!')



    def image_callback(self, msg):
        self.get_logger().info('Received image :D!!')
        max_d = 0.0
        desired_line = 0


        img_camara = self.bridge.imgmsg_to_cv2(msg, desired_encoding = "rgb8")
        img_camara = cv2.flip(img_camara, -1)
        img_camara = img_camara[600:950,340:940]
        img_gray = cv2.cvtColor(img_camara, cv2.COLOR_RGB2GRAY)

        # Calibracion luz
        kernel = np.ones((5,5), np.uint8)

        total_pixels = np.prod(img_gray.shape[:2])
        mask_pixels = cv2.countNonZero(img_gray)
        percentage = (mask_pixels / total_pixels)

        if percentage < 0.3:
           img_gray = cv2.add(img_gray, (np.ones(img_gray.shape, np.uint8)) * 50)
        elif percentage > 0.7:
           img_gray = cv2.subtract(img_gray, (np.ones(img_gray.shape, np.uint8)) * 50)
        else:
           img_gray = img_gray



        # Bordes Canny y HoughLines
        blurred = cv2.GaussianBlur(img_gray, (7, 7), 0)
        edged = cv2.Canny(blurred, 80,80)
        lines = cv2.HoughLinesP(edged,1,np.pi/180,1,self.minLineLength, self.maxLineGap)
        if lines is not None:
           for x in range(len(lines)):
                line = lines[x][0]
                x1_t,y1_t,x2_t,y2_t = line

                d = np.sqrt((x2_t - x1_t)**2 + (y2_t - y1_t)**2)
                if d > max_d:
                    max_d = d
                    desired_line = line

                x1,y1,x2,y2 = desired_line
                x_desired = max(x1, x2)

                epsilon = self.mitad - x_desired
                e_norm = (epsilon/self.mitad)
                self.v_ang = self.k * e_norm
                #self.v_lineal = self.v_lineal * (1 - (epsilon/self.mitad))


                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                cv2.circle(edged, (cx,cy), 5, (255,0,0),-1)
                cv2.line(edged, (x1,y1),(x2,y2),(255,0,0), 4)
        self.vel.linear.x = self.v_lineal
        self.vel.angular.z = self.v_ang

        #Paso Cebra
        lines_p = cv2.HoughLinesP(edged, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)
        if lines_p is not None:
            for lines_p in lines_p:
                x1, y1, x2, y2 = lines_p[0]

                # Calcular la pendiente de la línea
                m = (y2 - y1) / (x2 - x1 + 1e-5)
                # Si la pendiente es grande, es probable que sea una línea discontinua
                if abs(m) < 0.1:
                    self.get_logger().info('Parateeeee pero parateee')
                    cv2.line(edged,(x1,y1),(x2,y2),(255,0,0), 3)
                    #self.vel_lineal = 0.0
                    #self.vel_angular = 0.0
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 0.0

                    self.stop = True
                    break


        img_camara = cv2.resize(edged, (900,600),fx = 0.1,fy = 0.1)
        self.img_msg = self.bridge.cv2_to_imgmsg(edged, encoding = 'mono8')



    def timer_callback(self):
        #self.get_logger().info('Received image :D!!')


        # Publisher
        self.pub_vel_line.publish(self.vel)
        self.prueba.publish(self.img_msg)


def main(args=None):
    rclpy.init(args=args)
    ld = LineDetection()
    rclpy.spin(ld)
    ld.destroy_node()
    rclpy.shutdown()
    
if __name__== '__main__':
    main()

