import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# A partir da análise do arquivo .wbt
HALF_DISTANCE_BETWEEN_WHEELS = 0.06
WHEEL_RADIUS = 0.04

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # Corrigindo os nomes dos motores para corresponderem ao arquivo Webots
        self.__motor_1 = self.__robot.getDevice('MOTOR_1')
        self.__motor_2 = self.__robot.getDevice('MOTOR_2')
        self.__motor_3 = self.__robot.getDevice('MOTOR_3')
        self.__motor_4 = self.__robot.getDevice('MOTOR_4')

        # Configura os motores para controle de velocidade
        self.__motor_1.setPosition(float('inf'))
        self.__motor_2.setPosition(float('inf'))
        self.__motor_3.setPosition(float('inf'))
        self.__motor_4.setPosition(float('inf'))
        
        self.__motor_1.setVelocity(0)
        self.__motor_2.setVelocity(0)
        self.__motor_3.setVelocity(0)
        self.__motor_4.setVelocity(0)

        # Motor da câmera
        self.__camera_motor = self.__robot.getDevice('MOTOR_CAM')
        self.__camera_motor.setPosition(float('inf'))
        self.__camera_motor.setVelocity(0)

        self.__target_twist = Twist()
        self.__target_cam_pan_velocity = 0.0

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(Float64, 'cam_pan_cmd', self.__cam_pan_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def __cam_pan_callback(self, msg):
        self.__target_cam_pan_velocity = msg.data

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # Controle das rodas
        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        # Lógica para 4 rodas (diferencial)
        # Rodas da frente e de trás no mesmo lado devem ter a mesma velocidade
        left_wheel_speed = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        right_wheel_speed = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__motor_1.setVelocity(left_wheel_speed)
        self.__motor_3.setVelocity(left_wheel_speed)
        self.__motor_2.setVelocity(right_wheel_speed)
        self.__motor_4.setVelocity(right_wheel_speed)

        # Controle da câmera
        self.__camera_motor.setVelocity(self.__target_cam_pan_velocity)
