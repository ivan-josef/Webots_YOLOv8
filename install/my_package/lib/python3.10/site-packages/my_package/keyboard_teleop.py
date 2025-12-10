import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64 # Importe a mensagem de float
import sys, select, tty, termios

msg = """
Reading from the keyboard!
---------------------------
Moving the robot:
'w' - forward
's' - backward
'a' - left
'd' - right

Camera control:
'q' - pan left
'e' - pan right

CTRL-C to quit
"""

move_bindings = {
    'w': (0.4, 0.0),   # Forward
    's': (-0.4, 0.0),  # Backward
    'a': (0.0, 0.4),   # Turn left
    'd': (0.0, -0.4),  # Turn right
}

# Defina os valores de velocidade para a câmera
camera_bindings = {
    'q': 0.4,  # Pan left
    'e': -0.4, # Pan right
}

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('keyboard_teleop')
    pub_vel = node.create_publisher(Twist, 'cmd_vel', 10)
    # Adicione um publisher para a câmera
    pub_cam = node.create_publisher(Float64, 'cam_pan_cmd', 10)

    try:
        print(msg)
        while rclpy.ok():
            key = get_key()
            
            twist = Twist()
            cam_msg = Float64()
            
            if key in move_bindings:
                twist.linear.x = move_bindings[key][0]
                twist.angular.z = move_bindings[key][1]
                pub_vel.publish(twist)
                
            elif key in camera_bindings:
                cam_msg.data = camera_bindings[key]
                pub_cam.publish(cam_msg)
            
            elif key == '\x03':  # CTRL-C
                break
            
            else:
                # Se nenhuma tecla de comando for pressionada, envie uma velocidade zero
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                pub_vel.publish(twist)
                
                # Zere a velocidade da câmera também
                cam_msg.data = 0.0
                pub_cam.publish(cam_msg)

            rclpy.spin_once(node, timeout_sec=0)

    except Exception as e:
        print(e)
    finally:
        # Envie mensagens de velocidade zero ao sair
        pub_vel.publish(Twist())
        pub_cam.publish(Float64())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
