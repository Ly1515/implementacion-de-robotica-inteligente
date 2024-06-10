import rclpy
import rclpy.qos
from rclpy.node import Node
import math
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Point, Twist
import cv2
from cv_bridge import CvBridge, CvBridgeError



class LineandColorDetection(Node):
    def __init__(self):
        super().__init__('line_and_color_detection_node')

        self.sub_image = self.create_subscription(Image,'/video_source/raw', self.image_callback, rclpy.qos.qos_profile_sensor_data)

        #Linea
        self.pub_vel_line =  self.create_publisher(Twist, 'vel_line', rclpy.qos.qos_profile_sensor_data)
        self.prueba = self.create_publisher(Image, 'prueba', rclpy.qos.qos_profile_sensor_data)

        #colores
        self.img_mask_red = self.create_publisher(Image,'/img_propeties/red/msk', rclpy.qos.qos_profile_sensor_data)
        self.color_percentage_red = self.create_publisher(Float32, '/img_propeties/red/density', rclpy.qos.qos_profile_sensor_data)

        self.img_mask_green = self.create_publisher(Image,'/img_propeties/green/msk', rclpy.qos.qos_profile_sensor_data)
        self.color_percentage_green = self.create_publisher(Float32, '/img_propeties/green/density', rclpy.qos.qos_profile_sensor_data)

        self.img_mask_yellow = self.create_publisher(Image,'/img_propeties/yellow/msk', rclpy.qos.qos_profile_sensor_data)
        self.color_percentage_yellow = self.create_publisher(Float32, '/img_propeties/yellow/density', rclpy.qos.qos_profile_sensor_data)

        # vel final
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_sensor_data)

    

        #Variables
        self.bridge = CvBridge()

        self.minLineLength = 10
        self.maxLineGap = 10
        self.theta = 0
        self.mitad = 300
        self.k = 0.05

        self.img_msg = Image()
        self.img = Image()
        self.img_gray = np.zeros((10,10,3))
        self.img_linea = np.zeros((10,10,3))
        self.img_senal = np.zeros((10,10,3))
        self.img_flip = np.zeros((10,10,3))

        self.stop = False
        self.v_lineal = 0.02
        self.v_ang = 0.0
        self.vel = Twist()


        # variables color
        self.image_message_red = Image()
    
        self.image_message_green = Image()


        self.image_message_yellow = Image()

        self.red_p = Float32()
        self.green_p = Float32()
        self.yellow_p = Float32()
        self.yellow = False
        self.green = False
        self.red = False


        # variables timer
        self.timer_period = Float32()
        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
       # self.get_logger().info('Ready image :D!!')


    def detectar_colores(self, img_camara):
        hsv_img = cv2.cvtColor(img_camara, cv2.COLOR_RGB2HSV)

        # color rojo
        lower1_red = 150
        upper1_red = 255
        lower2_red = 0
        upper2_red = 0
        satu_red   = 125

        # Mask image
        lower_mask = hsv_img[:,:,0] > lower1_red
        upper_mask = hsv_img[:,:,0] < upper1_red
        lower_mask2 = hsv_img[:,:,0] >= lower2_red
        upper_mask2 = hsv_img[:,:,0] < upper2_red
        saturation_mask = hsv_img[:,:,1] > satu_red
        m_1 = (upper_mask & lower_mask)
        m_2 = (upper_mask2 & lower_mask2)
        mask2 = (m_1 | m_2) & saturation_mask

    
        red = img_camara[:,:,0]*mask2
        green = img_camara[:,:,1]*mask2
        blue = img_camara[:,:,2]*mask2

        img_masked_red  = np.dstack((red,green,blue))
        img_gray_r = cv2.cvtColor(img_masked_red, cv2.COLOR_RGB2GRAY)
        ret, th_r = cv2.threshold(img_gray_r, 12, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5,5), np.uint8)
        img_closing_red = cv2.morphologyEx(th_r, cv2.MORPH_CLOSE, kernel)


        # percentage
        total_pixels_r = np.prod(img_gray_r.shape[:2])
        mask_pixels_r = cv2.countNonZero(img_gray_r)
        self.red_p.data = (mask_pixels_r / total_pixels_r)*100

        self.image_message_red = self.bridge.cv2_to_imgmsg(img_closing_red, encoding = 'mono8')
    

        # color verde
        lower1_green = 64
        upper1_green = 102
        lower2_green = 0
        upper2_green = 0
        satu_green   = 127

        # Mask image
        lower_mask = hsv_img[:,:,0] > lower1_green
        upper_mask = hsv_img[:,:,0] < upper1_green
        lower_mask2 = hsv_img[:,:,0] >= lower2_green
        upper_mask2 = hsv_img[:,:,0] < upper2_green
        saturation_mask = hsv_img[:,:,1] > satu_green
        m_1 = (upper_mask & lower_mask)
        m_2 = (upper_mask2 & lower_mask2)
        mask2 = (m_1 | m_2) & saturation_mask


        red = img_camara[:,:,0]*mask2
        green = img_camara[:,:,1]*mask2
        blue = img_camara[:,:,2]*mask2

        img_masked_green = np.dstack((red,green,blue))
        img_gray_g = cv2.cvtColor(img_masked_green, cv2.COLOR_RGB2GRAY)
        ret, th_g = cv2.threshold(img_gray_g, 12, 255, cv2.THRESH_BINARY)

        img_closing_green = cv2.morphologyEx(th_g, cv2.MORPH_CLOSE, kernel)
        # percentage
        total_pixels_g = np.prod(img_gray_g.shape[:2])
        mask_pixels_g = cv2.countNonZero(img_gray_g)
        self.green_p.data = (mask_pixels_g / total_pixels_g)*100

        self.image_message_green = self.bridge.cv2_to_imgmsg(img_closing_green, encoding = 'mono8')

        # color amarillo
        lower1_yellow = 38
        upper1_yellow = 51
        lower2_yellow = 196
        upper2_yellow = 221
        satu_yellow   = 25

        # Mask image
        lower_mask = hsv_img[:,:,0] > lower1_yellow
        upper_mask = hsv_img[:,:,0] < upper1_yellow
        lower_mask2 = hsv_img[:,:,0] >= lower2_yellow
        upper_mask2 = hsv_img[:,:,0] < upper2_yellow
        saturation_mask = hsv_img[:,:,1] < satu_yellow
        mask2 = (upper_mask*lower_mask)+(upper_mask2*lower_mask2)*saturation_mask
    
        red = img_camara[:,:,0]*mask2
        green = img_camara[:,:,1]*mask2
        blue = img_camara[:,:,2]*mask2

        img_masked_yellow = np.dstack((red,green,blue))
        img_gray_y = cv2.cvtColor(img_masked_yellow, cv2.COLOR_RGB2GRAY)
        ret, th_y = cv2.threshold(img_gray_y, 12, 255, cv2.THRESH_BINARY)

        img_closing_yellow = cv2.morphologyEx(th_y, cv2.MORPH_CLOSE, kernel)
        # percentage
        total_pixels_y = np.prod(img_gray_y.shape[:2])
        mask_pixels_y = cv2.countNonZero(img_gray_y)
        self.yellow_p.data = (mask_pixels_y / total_pixels_y)*100 
        #self.get_logger().info('new yellow:D!!')

        self.image_message_yellow = self.bridge.cv2_to_imgmsg(img_closing_yellow, encoding = 'mono8')

    


    def detectar_lineas(self, img_gray):
        max_d = 0
        desired_line = 0

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

        self.img_camara = cv2.resize(edged, (900,600),fx = 0.1,fy = 0.1)

        self.img_msg = self.bridge.cv2_to_imgmsg(edged, encoding = 'mono8')




    def image_callback(self, msg):
        #self.get_logger().info('Received image :D!!')

        img_camara = self.bridge.imgmsg_to_cv2(msg, desired_encoding = "rgb8")
        self.img_flip = cv2.flip(img_camara, -1)

        img_cropped = self.img_flip[600:950,340:940]
        self.img_gray = cv2.cvtColor(img_cropped, cv2.COLOR_RGB2GRAY)



        #Calibración Luz 
    
        total_pixels = np.prod(self.img_gray.shape[:2])
        mask_pixels = cv2.countNonZero(self.img_gray)
        percentage = (mask_pixels / total_pixels)

        if percentage < 0.3:
           self.img_gray = cv2.add(self.img_gray, (np.ones(self.img_gray.shape, np.uint8)) * 50)
        elif percentage > 0.7:
           self.img_gray = cv2.subtract(self.img_gray, (np.ones(self.img_gray.shape, np.uint8)) * 50)
        else:
           self.img_gray = self.img_gray


    def timer_callback(self):
        try:
            self.detectar_lineas(self.img_gray)
            self.detectar_colores(self.img_flip)

            self.vel.linear.x = self.v_lineal
            self.vel.angular.z = self.v_ang


            # Publisher
            self.pub_vel_line.publish(self.vel)
            self.prueba.publish(self.img_msg)

            # colores
            self.img_mask_red.publish(self.image_message_red)
            self.color_percentage_red.publish(self.red_p)

            self.img_mask_green.publish(self.image_message_green)
            self.color_percentage_green.publish(self.green_p)   

            self.img_mask_yellow.publish(self.image_message_yellow)
            self.color_percentage_yellow.publish(self.yellow_p)
        except:
           pass

def main(args=None):
    rclpy.init(args=args)
    ld = LineDetection()
    rclpy.spin(ld)
    ld.destroy_node()
    rclpy.shutdown()

    
if __name__== '__main__':
    main()




