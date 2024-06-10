import rclpy
import rclpy.qos
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from collections import Counter
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

        self.gris = np.zeros((10,10,3))
        self.img_camara = np.zeros((10,10,3))

        self.index_sift = 0
        self.indices_detectados = []


        # Crear el objeto SIFT y FLANN una vez
        self.sift = cv2.SIFT_create()
        self.bf = cv2.BFMatcher()

        # Cargar imágenes de referencia
        self.load_reference_images()

        # Timer setup
        self.timer_period = 0.2  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def load_reference_images(self):
        for nombre in self.nombres_archivos:
            ruta_imagen = f"/home/puzzlebot/ros2_ws/src/reto_final/reto_final/{nombre}.jpg"
            ref_img = cv2.imread(ruta_imagen, 0)
            if ref_img is None:
                self.get_logger().error(f"Failed to load image at {ruta_imagen}")
                continue
            ref_img = cv2.normalize(ref_img, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')

            ref_kp, ref_des = self.sift.detectAndCompute(ref_img, None)
            self.referencias.append((ref_kp, ref_des))


    def detectar_sift(self, gris):
        # Detectar características SIFT en la imagen de la cámara
        kp, des = self.sift.detectAndCompute(gris, None)

        if des is None or len(des) == 0:
            self.indice_senal_detectada.data = -1
            #self.img = self.bridge.cv2_to_imgmsg(img_camara, encoding='rgb8')
            return

        num_coincidencias = []

        # Encontrar coincidencias entre los keypoints de la cámara y las imágenes de referencia
        for kp_ref, des_ref in self.referencias:
            matches = self.bf.knnMatch(des_ref, des, k=2)
            good_matches = []
            for m, n in matches:
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)

            num_coincidencias.append(len(good_matches))



        indice_max_coincidencias = int(np.argmax(num_coincidencias))


        # Encontrar el índice de la señal con el mayor número de coincidencias
        umbrales_minimos = [10, 10, 5, 10, 5, 5, 10]  # Umbrales mínimos específicos para cada señal

        if num_coincidencias[indice_max_coincidencias] >= umbrales_minimos[indice_max_coincidencias]:
            mejor_indice = indice_max_coincidencias
        else:
            mejor_indice = 7

        self.get_logger().info('concidencias: {}'.format(mejor_indice))


        return mejor_indice



    def image_callback(self, msg):
        #self.get_logger().info('Received image :D!!')

        self.img_camara = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        self.img_camara = cv2.flip(self.img_camara, -1)
        #img_camara = img_camara[450:950,0:1280]
        self.gris = cv2.cvtColor(self.img_camara, cv2.COLOR_BGR2GRAY)


   
    def timer_callback(self):

        self.index_sift = self.detectar_sift(self.gris)

        self.indices_detectados.append(self.index_sift)

        if len(self.indices_detectados) > 30:
            self.indices_detectados.pop(0)

        contador = Counter(self.indices_detectados)    
        index_final = contador.most_common(1)[0][0]
        #index_final = self.index_sift

        conteo_5 = self.indices_detectados.count(5)

        if conteo_5 > 2:
            index_final = 5

        self.indice_senal_detectada.data = index_final

        self.get_logger().info('indice: {}'.format(index_final))




        if self.indice_senal_detectada.data != 7:
            nombres_senales = ["Stop", "Giveaway", "Straight", "Turnaround", "Turn Left", "Turn Right", "Work in Progress"]
            nombre_senal = nombres_senales[self.indice_senal_detectada.data]
            cv2.putText(self.img_camara, nombre_senal, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        self.img = self.bridge.cv2_to_imgmsg(self.img_camara, encoding='rgb8')

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
