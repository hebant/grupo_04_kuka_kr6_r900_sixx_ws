import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        
        # susc y pub
        self.subscription = self.create_subscription(Point, '/target', self.target_callback, 10)
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # config inicial de q0
        self.q_actual = np.zeros(6) 
        
        #limites articulares del kuka en rad
        self.limites_min = np.array([-2.96, -3.31, -2.09, -3.22, -2.09, -6.10])
        self.limites_max = np.array([ 2.96,  0.78,  2.72,  3.22,  2.09,  6.10])
        
        self.get_logger().info("Nodo IK iniciado. Esperando objetivo (x, y, z) en /target...")

    def calcular_fk(self, q):
        q1, q2, q3, q4, q5, q6 = q
        px = -0.08*(np.sin(q1)*np.sin(q4) + np.sin(q2 + q3)*np.cos(q1)*np.cos(q4))*np.sin(q5) + (0.035*np.sin(q2 + q3) + 0.455*np.cos(q2) + 0.025)*np.cos(q1) + 0.08*np.cos(q1)*np.cos(q5)*np.cos(q2 + q3) + 0.42*np.cos(q1)*np.cos(q2 + q3)
        py = -0.08*(np.sin(q1)*np.sin(q2 + q3)*np.cos(q4) - np.sin(q4)*np.cos(q1))*np.sin(q5) + (0.035*np.sin(q2 + q3) + 0.455*np.cos(q2) + 0.025)*np.sin(q1) + 0.08*np.sin(q1)*np.cos(q5)*np.cos(q2 + q3) + 0.42*np.sin(q1)*np.cos(q2 + q3)
        pz = -0.455*np.sin(q2) - 0.08*np.sin(q5)*np.cos(q4)*np.cos(q2 + q3) - 0.08*np.sin(q2 + q3)*np.cos(q5) - 0.42*np.sin(q2 + q3) + 0.035*np.cos(q2 + q3) + 0.400
        return np.array([px, py, pz])

    def calcular_jacobiano(self, q):
        q1, q2, q3, q4, q5, q6 = q
        J = np.zeros((3, 6))
        J[0,0] = 0.08*(np.sin(q1)*np.sin(q2 + q3)*np.cos(q4) - np.sin(q4)*np.cos(q1))*np.sin(q5) - (0.035*np.sin(q2 + q3) + 0.455*np.cos(q2) + 0.025)*np.sin(q1) - 0.08*np.sin(q1)*np.cos(q5)*np.cos(q2 + q3) - 0.42*np.sin(q1)*np.cos(q2 + q3)
        J[0,1] = (-0.455*np.sin(q2) - 0.08*np.sin(q5)*np.cos(q4)*np.cos(q2 + q3) - 0.08*np.sin(q2 + q3)*np.cos(q5) - 0.42*np.sin(q2 + q3) + 0.035*np.cos(q2 + q3))*np.cos(q1)
        J[0,2] = (-0.08*np.sin(q5)*np.cos(q4)*np.cos(q2 + q3) - 0.08*np.sin(q2 + q3)*np.cos(q5) - 0.42*np.sin(q2 + q3) + 0.035*np.cos(q2 + q3))*np.cos(q1)
        J[0,3] = 0.08*(-np.sin(q1)*np.cos(q4) + np.sin(q4)*np.sin(q2 + q3)*np.cos(q1))*np.sin(q5)
        J[0,4] = -0.08*(np.sin(q1)*np.sin(q4) + np.sin(q2 + q3)*np.cos(q1)*np.cos(q4))*np.cos(q5) - 0.08*np.sin(q5)*np.cos(q1)*np.cos(q2 + q3)
        J[0,5] = 0
        J[1,0] = -0.08*(np.sin(q1)*np.sin(q4) + np.sin(q2 + q3)*np.cos(q1)*np.cos(q4))*np.sin(q5) + (0.035*np.sin(q2 + q3) + 0.455*np.cos(q2) + 0.025)*np.cos(q1) + 0.08*np.cos(q1)*np.cos(q5)*np.cos(q2 + q3) + 0.42*np.cos(q1)*np.cos(q2 + q3)
        J[1,1] = (-0.455*np.sin(q2) - 0.08*np.sin(q5)*np.cos(q4)*np.cos(q2 + q3) - 0.08*np.sin(q2 + q3)*np.cos(q5) - 0.42*np.sin(q2 + q3) + 0.035*np.cos(q2 + q3))*np.sin(q1)
        J[1,2] = (-0.08*np.sin(q5)*np.cos(q4)*np.cos(q2 + q3) - 0.08*np.sin(q2 + q3)*np.cos(q5) - 0.42*np.sin(q2 + q3) + 0.035*np.cos(q2 + q3))*np.sin(q1)
        J[1,3] = 0.08*(np.sin(q1)*np.sin(q4)*np.sin(q2 + q3) + np.cos(q1)*np.cos(q4))*np.sin(q5)
        J[1,4] = -0.08*(np.sin(q1)*np.sin(q2 + q3)*np.cos(q4) - np.sin(q4)*np.cos(q1))*np.cos(q5) - 0.08*np.sin(q1)*np.sin(q5)*np.cos(q2 + q3)
        J[1,5] = 0
        J[2,0] = 0
        J[2,1] = 0.08*np.sin(q5)*np.sin(q2 + q3)*np.cos(q4) - 0.035*np.sin(q2 + q3) - 0.455*np.cos(q2) - 0.08*np.cos(q5)*np.cos(q2 + q3) - 0.42*np.cos(q2 + q3)
        J[2,2] = 0.08*np.sin(q5)*np.sin(q2 + q3)*np.cos(q4) - 0.035*np.sin(q2 + q3) - 0.08*np.cos(q5)*np.cos(q2 + q3) - 0.42*np.cos(q2 + q3)
        J[2,3] = 0.08*np.sin(q4)*np.sin(q5)*np.cos(q2 + q3)
        J[2,4] = 0.08*np.sin(q5)*np.sin(q2 + q3) - 0.08*np.cos(q4)*np.cos(q5)*np.cos(q2 + q3)
        J[2,5] = 0
        return J

    def target_callback(self, msg):
        self.get_logger().info(f"Objetivo recibido: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})")
        p_objetivo = np.array([msg.x, msg.y, msg.z])
        
        tolerancia = 0.001
        alpha = 0.1
        max_iter = 2000
        
        self.q_actual = np.array([
            np.arctan2(msg.y, msg.x),
            0.5,
            1.0,
            0.0,
            0.5,
            0.0
        ])
        
        for i in range(max_iter):
            p_actual = self.calcular_fk(self.q_actual)
            e = p_objetivo - p_actual
            error_norm = np.linalg.norm(e)
            
            if error_norm < tolerancia:
                self.get_logger().info(f"Convergencia en {i} iter. Error: {error_norm:.4e} m")
                self.publicar_estado()
                return
                
            J = self.calcular_jacobiano(self.q_actual)
            J_pseudo = np.linalg.pinv(J)
            delta_q = alpha * np.dot(J_pseudo, e)
            self.q_actual = np.clip(self.q_actual + delta_q, self.limites_min, self.limites_max)
            
        self.get_logger().warn(f"Fallo de convergencia. Error final: {error_norm:.4f} m")
        self.publicar_estado()
    
    def publicar_estado(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        msg.position = self.q_actual.tolist()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()