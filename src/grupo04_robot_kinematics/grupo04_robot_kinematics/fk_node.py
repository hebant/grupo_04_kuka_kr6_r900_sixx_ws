import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class FKNode(Node):
    def __init__(self):
        super().__init__('fk_node')
        # suscripcion al topico de articulaciones
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.get_logger().info("Nodo FK iniciado. Escuchando /joint_states...")

    def joint_state_callback(self, msg):
        try:
            q = [0.0]*6
            for i, name in enumerate(['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']):
                if name in msg.name:
                    index = msg.name.index(name)
                    q[i] = msg.position[index]
            
            q1, q2, q3, q4, q5, q6 = q

            q1 = -q1

            # ECUACIONES
            px = -0.08*(np.sin(q1)*np.sin(q4) + np.sin(q2 + q3)*np.cos(q1)*np.cos(q4))*np.sin(q5) + (0.035*np.sin(q2 + q3) + 0.455*np.cos(q2) + 0.025)*np.cos(q1) + 0.08*np.cos(q1)*np.cos(q5)*np.cos(q2 + q3) + 0.42*np.cos(q1)*np.cos(q2 + q3)
            py = -0.08*(np.sin(q1)*np.sin(q2 + q3)*np.cos(q4) - np.sin(q4)*np.cos(q1))*np.sin(q5) + (0.035*np.sin(q2 + q3) + 0.455*np.cos(q2) + 0.025)*np.sin(q1) + 0.08*np.sin(q1)*np.cos(q5)*np.cos(q2 + q3) + 0.42*np.sin(q1)*np.cos(q2 + q3)
            pz = -0.455*np.sin(q2) - 0.08*np.sin(q5)*np.cos(q4)*np.cos(q2 + q3) - 0.08*np.sin(q2 + q3)*np.cos(q5) - 0.42*np.sin(q2 + q3) + 0.035*np.cos(q2 + q3) + 0.400

            self.get_logger().info(f"Posicion Cartesian: x={px:.3f}, y={py:.3f}, z={pz:.3f}")

        except Exception as e:
            self.get_logger().error(f"Error procesando fk: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
