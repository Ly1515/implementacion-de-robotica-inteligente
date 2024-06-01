import rclpy
import rclpy.qos
from rclpy.node import Node
import math
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge, CvBridgeError





class ColorIdentification(Node):
    def __init__(self):
        super().__init__('color_identification_node')

        self.sub_image = self.create_subscription(Image,'/video_source/raw', self.image_callback, rclpy.qos.qos_profile_sensor_data)

        self.img_mask_red = self.create_publisher(Image,'/img_propeties/red/msk', rclpy.qos.qos_profile_sensor_data)
        self.color_percentage_red = self.create_publisher(Float32, '/img_propeties/red/density', rclpy.qos.qos_profile_sensor_data)

        self.img_mask_green = self.create_publisher(Image,'/img_propeties/green/msk', rclpy.qos.qos_profile_sensor_data)
        self.color_percentage_green = self.create_publisher(Float32, '/img_propeties/green/density', rclpy.qos.qos_profile_sensor_data)


        #Variables
        self.bridge = CvBridge()

        self.img = Image()

        self.image_message_red = Image()
        self.percentage_red = Float32()

        self.image_message_green = Image()
        self.percentage_green = Float32()




        # variables timer
        self.timer_period = Float32()
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        #self.get_logger().info('Ready image :D!!')


    def image_callback(self, msg):
        #self.get_logger().info('Received image :D!!')
        img_camara = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'rgb8')
        hsv_img = cv2.cvtColor(img_camara, cv2.COLOR_RGB2HSV)

        # color rojo
        lower1_red = 0
        upper1_red = 10
        lower2_red = 170
        upper2_red = 180
        satu_red   = 100

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
        self.percentage_red.data = (mask_pixels_r / total_pixels_r)*100

        self.image_message_red = self.bridge.cv2_to_imgmsg(img_closing_red, encoding = 'mono8')
    

        # color verde
        lower1_green = 30
        upper1_green = 90
        lower2_green = 0
        upper2_green = 0
        satu_green   = 0

        # Mask image
        lower_mask = hsv_img[:,:,0] > lower1_green
        upper_mask = hsv_img[:,:,0] < upper1_green
        lower_mask2 = hsv_img[:,:,0] >= lower2_green
        upper_mask2 = hsv_img[:,:,0] < upper2_green
        saturation_mask = hsv_img[:,:,1] > satu_green
        mask2 = (upper_mask*lower_mask)+(upper_mask2*lower_mask2)*saturation_mask
    
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
        self.percentage_green.data = (mask_pixels_g / total_pixels_g)*100 + 5

        self.image_message_green = self.bridge.cv2_to_imgmsg(img_closing_green, encoding = 'mono8')




    def timer_callback(self):

        self.img_mask_red.publish(self.image_message_red)
        self.color_percentage_red.publish(self.percentage_red)

        self.img_mask_green.publish(self.image_message_green)
        self.color_percentage_green.publish(self.percentage_green)

    



def main(args=None):
    rclpy.init(args=args)
    ci = ColorIdentification()
    rclpy.spin(ci)
    ci.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main()

