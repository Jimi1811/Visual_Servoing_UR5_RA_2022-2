#!/usr/bin/env python
 
# Obtenido del Laboratorio 6 de Palomino y Rivas en el 2020-1
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
 
from tf.transformations import euler_from_quaternion, quaternion_from_euler


# Para simplificar el codigo
sin = np.sin
cos = np.cos
 
# Aqui va a depender de que nos esten pidiendo como orientacion
global xObj, yObj, zObj, cwObj, cxObj, cyObj, czObj, pitch
 
zObj=0.0
pitch = 0.0

 
def callbackPoseObj(msg):
    # Aqui va a depender de que nos esten pidiendo como orientacion
    global xObj, yObj, zObj, cwObj, cxObj, cyObj, czObj, pitch
    # Recuperamos la posicion
    xObj = msg.pose.position.x
    yObj = msg.pose.position.y
    zObj = msg.pose.position.z
    # Recuperamos la orientacion
    cwObj = msg.pose.orientation.w
    cxObj = msg.pose.orientation.x
    cyObj = msg.pose.orientation.y
    czObj = msg.pose.orientation.z

    orientation_q = msg.pose.orientation
    orientation_list = [cxObj, cyObj, czObj, cwObj]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)


 
if __name__ == '__main__':
 
    # Inicializamos configuraciones de nodo, a donde nos subscribiremos
    rospy.init_node("servo_pos")
 
    # Inicializamos el subscriptor
    rospy.Subscriber("/visp_auto_tracker/object_position",
                     PoseStamped, callbackPoseObj)
 
    # Inicializamos el publicador
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
 

    # 3. Inicializamos una variable llamada twist del tipo Twist() de geometry_msgs. Esto tiene las velocidades lineales y angulares en cada eje
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
 
    # 5. Inicializamos la ganancia Kp y Kd
    Kp = 1
    Kd = 0.01
    #Kproporcional = np.eye(3)*Kp
    #Kderivativo = np.eye(3)*Kd
    epsilon = 1e-3
    e = 0
    e_theta = 0
    # 6. Ponemos el rate del loop en Hz
    rate = rospy.Rate(10)
 
    # Abro un archivo .txt
    #PosicionActual = open("/home/user/PosicionActual.txt", "w")
    #PosicionDeseada = open("/home/user/PosicionDeseada.txt", "w")
 
    # Control Cinematico
    while not rospy.is_shutdown():
 
        # 4. Inicializamos la posicion deseada con respecto al frame inicial.
        # Queremos que se mueva hacia delante, entonces solo ponemos la profundidad
        ZObjd = 0.168
        theta_des = 0.0

 
        # 8. Se actualiza eAnterior y e
        eAnterior = e
        e = zObj-ZObjd
        L = Kp*(e) - Kd*((e-eAnterior)/0.1)

        # 8. Se actualiza eAnterior y e
        e_theta_Anterior = e_theta
        e_theta = theta_des - pitch
        L_theta = Kp*(e_theta) - Kd*((e_theta-e_theta_Anterior)/0.1)


 
        # 9. Detenemos el algoritmo si el error es cercano a 0
        if (np.abs(L) < epsilon):
            # 15. Se actualiza los valores de v y w
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            pub.publish(twist )
        else:

            # 15. Se actualiza los valores de v y w
            if(L > 0.18):
                L=0.18
            elif (L < -0.18):
                L = -0.18

            if(L_theta > 1.82):
                L_theta=1.82
            elif (L_theta < -1.82):
                L_theta = -1.82              

            twist.linear.x = L
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = L_theta

            print(L)
            # 16. Se publica
            pub.publish(twist)
 
        # 17. Guardo valores para ploear
        #PosicionActual.write(str(x) + " " + str(y) + " " + str(theta) + '\n')
        #PosicionDeseada.write(str(xd) + " " + str(yd) +
                             # " " + str(thetad) + '\n')
 
        # 18. Se espera la siguiente iteracion definida por el rate de 10kHz
        rate.sleep()
 
 
#PosicionActual.close()
#PosicionDeseada.close()

