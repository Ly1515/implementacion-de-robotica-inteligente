import rclpy
import rclpy.qos
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import cv2
from cv_bridge import CvBridge, CvBridgeError


class SignalDetection(Node):
    def __init__(self):
        super().__init__('signal_detection_node')

        self.sub_image = self.create_subscription(Image, '/video_source/raw', self.image_callback, rclpy.qos.qos_profile_sensor_data)
        self.pub_señal = self.create_publisher(Int32, 'senal', rclpy.qos.qos_profile_sensor_data)
        self.img_contorno = self.create_publisher(Image, 'senal_detectada', rclpy.qos.qos_profile_sensor_data)

        self.bridge = CvBridge()
        self.img = Image()

        self.nombres_archivos = ["stop", "giveaway", "straigth", "turnaround", "turnleft", "turnright", "workinprogress"]
        self.referencias = []
        self.indice_senal_detectada = Int32()
        
        self.indice_ant = 0
        self.indice_act = 0

        # Crear el objeto SIFT y FLANN una vez
        self.sift = cv2.SIFT_create()
        index_params = dict(algorithm=1, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        # Cargar imágenes de referencia
        self.load_reference_images()

        # Timer setup
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def load_reference_images(self):
        for nombre in self.nombres_archivos:
            ruta_imagen = f"/home/lymc/ros2_ws/src/reto_final/reto_final/{nombre}.jpg"
            ref_img = cv2.imread(ruta_imagen, 0)
            if ref_img is None:
                self.get_logger().error(f"Failed to load image at {ruta_imagen}")
                continue
            ref_img = cv2.normalize(ref_img, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')

            ref_kp, ref_des = self.sift.detectAndCompute(ref_img, None)
            self.referencias.append((ref_kp, ref_des))

    def image_callback(self, msg):
        self.get_logger().info('Received image :D!!')

        img_camara = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        img_camara = cv2.flip(img_camara, -1)
        img_camara = img_camara[450:950,0:1280]
        gris = cv2.cvtColor(img_camara, cv2.COLOR_BGR2GRAY)

        # Detectar características SIFT en la imagen de la cámara
        kp, des = self.sift.detectAndCompute(gris, None)

        if des is None or len(des) == 0:
            self.indice_senal_detectada.data = -1
            self.img = self.bridge.cv2_to_imgmsg(img_camara, encoding='rgb8')
            return

        num_coincidencias = []

        # Encontrar coincidencias entre los keypoints de la cámara y las imágenes de referencia
        for kp_ref, des_ref in self.referencias:
            matches = self.flann.knnMatch(des_ref, des, k=2)
            good_matches = [m for m, n in matches if m.distance < 0.7 * n.distance]
            num_coincidencias.append(len(good_matches))

        # Encontrar el índice de la señal con el mayor número de coincidencias
        if num_coincidencias:
            self.indice_act = int(np.argmax(num_coincidencias))
            
            if self.indice_ant == self.indice_act:
                self.indice_senal_detectada.data = self.indice_act
                self.indice_ant = self.indice_act
            else:
                self.indice_ant = self.indice_act
                
                
        else:
            self.indice_senal_detectada.data = 7  # No signal detected
        

        if self.indice_senal_detectada.data != 7:
            nombres_senales = ["Stop", "Giveaway", "Straight", "Turnaround", "Turn Left", "Turn Right", "Work in Progress"]
            nombre_senal = nombres_senales[self.indice_senal_detectada.data]
            cv2.putText(img_camara, nombre_senal, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        self.img = self.bridge.cv2_to_imgmsg(img_camara, encoding='rgb8')

   
   
    def timer_callback(self):
        self.pub_señal.publish(self.indice_senal_detectada)
        self.img_contorno.publish(self.img)


def main(args=None):
    rclpy.init(args=args)
    sd = SignalDetection()
    rclpy.spin(sd)
    sd.destroy_node()
    rclpy.shutdown()


if __name__ == '_main_':
    main()