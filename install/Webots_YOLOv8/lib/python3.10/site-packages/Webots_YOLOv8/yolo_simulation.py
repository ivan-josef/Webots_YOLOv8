#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image as ROS_Image
import Webots_YOLOv8.running_inference as ri


class YoloSimulacao(Node):

    def __init__(self):
        super().__init__('teste_yolo_sim')
        self.get_logger().info('>> MODO TESTE VISUAL (YOLO) <<')
        self.window_name = "detection window"

        # 1. Carrega IA
        self.model = ri.model
        
        # 2. Ponte CV <-> ROS
        self.bridge = CvBridge()

        # 3. Subscriber (Entrada do Webots)
        self.camera_subscriber = self.create_subscription(
            ROS_Image, 
            '/camera/image', 
            self.image_callback, 
            10
        )

        # 4. Publisher DE DEBUG 
        self.debug_pub = self.create_publisher(ROS_Image, 'processed_image_topic', 10)

    def image_callback(self, ros_image_msg):
        try:
            # Converte entrada
            frame = self.bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding="bgr8")
            
            # Roda o YOLO
            _, _, _, inference_frame = ri.detect_model(self.model, frame)
            
            # --- VISUALIZAÇÃO ---
            # Converte a imagem desenhada de volta para ROS e publica
            debug_msg = self.bridge.cv2_to_imgmsg(inference_frame, "bgr8")
            self.debug_pub.publish(debug_msg)

            cv2.imshow(self.window_name, inference_frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Erro: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloSimulacao()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
