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

        self.sub_image = self.create_subscription(Image,'/video_source/raw', self.image_callback, 10)
        self.img_mask = self.create_publisher(Image,'/img_propeties/red/msk', 10)
        self.color_percentage = self.create_publisher(Float32, '/img_propeties/red/density', 10)
        self.img_centermass = self.create_publisher(Point, '/img_propeties/red/xy', 10)
        



        #Variables
        self.bridge = CvBridge()
        self.image_message = Image()
        self.percentage = Float32()
        self.pos = Point()
        self.color = 1


        # variables timer
        self.timer_period = Float32()
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Ready image :D!!')


    def image_callback(self, msg):
        self.get_logger().info('Received image :D!!')
        img_camara = self.bridge.imgmsg_to_cv2(msg,'rgb8')
        hsv_img = cv2.cvtColor(img_camara, cv2.COLOR_RGB2HSV)

        # Mask image

        if self.color == 1:
         # color rojo
            lower1 = 0
            upper1 = 10
            lower2 = 170
            upper2 = 180
            satu   = 100
            op     = 0

        elif self.color == 2:
         # color verde
            lower1 = 30
            upper1 = 90
            lower2 = 0
            upper2 = 0
            satu   = 0
            op     = 0

        elif self.color == 3:
          # color azul
            lower1 = 100
            upper1 = 140
            lower2 = 0
            upper2 = 0
            satu   = 0
            op     = 0

        elif self.color == 4:
          # color  amarillo
            lower1 = 20
            upper1 = 40
            lower2 = 0
            upper2 = 0
            satu   = 0
            op     = 0

        elif self.color == 5:
          # color  blanco
            lower1 = 0
            upper1 = 0
            lower2 = 0
            upper2 = 0
            satu   = 0.3
            value  = 200
            op     = 1

        else:
            lower1 = 0
            upper1 = 0
            lower2 = 0
            upper2 = 0
            satu   = 0
            value  = 0
            op     = 2


        if op == 0:
            lower_mask = hsv_img[:,:,0] > lower1
            upper_mask = hsv_img[:,:,0] < upper1
            lower_mask2 = hsv_img[:,:,0] >= lower2
            upper_mask2 = hsv_img[:,:,0] < upper2
            saturation_mask = hsv_img[:,:,1] > satu
            mask2 = (upper_mask*lower_mask)+(upper_mask2*lower_mask2)*saturation_mask
        elif op == 1:
            saturation_mask = hsv_img[:,:,1] < satu
            value_mask = hsv_img[:,:,2] > value
            mask2 = saturation_mask*value_mask
        else:
             saturation_mask = 0
             mask2 = 0

        red = img_camara[:,:,0]*mask2
        green = img_camara[:,:,1]*mask2
        blue = img_camara[:,:,2]*mask2

        img_masked = np.dstack((red,green,blue))
        img_gray = cv2.cvtColor(img_masked, cv2.COLOR_RGB2GRAY)
        ret, th = cv2.threshold(img_gray, 12, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5,5), np.uint8)
        img_closing = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel)


        # percentage
        total_pixels = np.prod(img_gray.shape[:2])
        mask_pixels = cv2.countNonZero(img_gray)
        self.percentage.data = (mask_pixels / total_pixels)*100

        # centroid
        contours, hierarchy = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)

        cv2.drawContours(img_closing, contours, -1, (0,255,0), 3)

        for c in contours:
          M = cv2.moments(c)

          if M["m00"] != 0: 
             cX = int(M["m10"] / M["m00"])
             cY = int(M["m01"] / M["m00"])
             cv2.circle(img_closing, (cX, cY), 5, (0, 0, 255), -1)
             cv2.putText(img_closing, "centroid", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
             
             self.pos.x = float(cX)
             self.pos.y = float(cY) 


        self.image_message = self.bridge.cv2_to_imgmsg(img_closing, 'mono8')
    
    def timer_callback(self):

        self.img_mask.publish(self.image_message)
        self.color_percentage.publish(self.percentage)
        self.img_centermass.publish(self.pos)



def main(args=None):
    rclpy.init(args=args)
    ci = ColorIdentification()
    rclpy.spin(ci)
    ci.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main()
