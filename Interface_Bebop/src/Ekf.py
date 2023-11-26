#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from filterpy.kalman import ExtendedKalmanFilter

class FusionNode:
    def __init__(self):
        rospy.init_node('fusion_node')

        # Inicialize o Filtro de Kalman Estendido
        self.ekf = ExtendedKalmanFilter(dim_x=12, dim_z=6)  # Dimensões de estado e medida

        # Subscribers
        rospy.Subscriber('/robot_aruco_pose', PoseStamped, self.aruco_pose_callback)
        rospy.Subscriber('/bebop2/odometry_sensor1/odometry', Odometry, self.odometry_callback)

        # Outro inicialização e configuração do filtro aqui...
        # Exemplo de configuração do EKF (dentro da classe FusionNode)
        self.ekf.F = ...  # Matriz de transição de estado
        self.ekf.H = ...  # Matriz de transformação de medida
        self.ekf.P *= ...  # Matriz de covariância inicial
        self.ekf.Q = ...  # Matriz de covariância do processo
        self.ekf.R = ...  # Matriz de covariância da medida 


    def aruco_pose_callback(self, aruco_pose):
        # Atualize o filtro com informações da pose do aruco
        z = [aruco_pose.pose.position.x, aruco_pose.pose.position.y, aruco_pose.pose.position.z,
            aruco_pose.pose.orientation.x, aruco_pose.pose.orientation.y, aruco_pose.pose.orientation.z]
        self.ekf.update(z)

    def odometry_callback(self, odometry):
        # Atualize o filtro com informações da odometria
        z = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z,
            odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z]
        self.ekf.update(z)

    def run(self):
        rate = rospy.Rate(10)  # Taxa de execução em Hz
        while not rospy.is_shutdown():
            # Obtenha a estimativa do estado do filtro
                x_estimate = self.ekf.x

            # Lógica para publicar a estimativa em tópicos desejados...

            rate.sleep()
if __name__ == '__main__':
    try:
        fusion_node = FusionNode()
        fusion_node.run()
    except rospy.ROSInterruptException:
        pass
