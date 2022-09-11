import rospy
import math
import time
import ast
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
En linea control 9
"""
e = """
EStoy fuera control 9
"""

k0=rospy.get_param("control9/kp")
k1=rospy.get_param("control9/kd")
hz=rospy.get_param("control9/hz")

tiempos=ast.literal_eval(rospy.get_param("/control9/tiempos"))
t1=tiempos[0]
t2=tiempos[1]

delta=1/hz

lx=ast.literal_eval(rospy.get_param("/control9/lx"))
ly=ast.literal_eval(rospy.get_param("/control9/ly"))

lx3=lx[0][2]
ly3=ly[0][2]

lx9=lx[0][8]
ly9=ly[0][8]

V1=0
V2=0.01
w=0.0

x9c1=0
y9c1=0
x9c2=0
y9c2=0
dx9c=0
dy9c=0.01
th9=0

x3c1=0
y3c1=0
x3c2=0
y3c2=0
dx3c=0
dy3c=0.01

L=0.092
t=1
 
def callback9(data):
    global x9c1,y9c1,th9
    x9c1 = data.pose.pose.position.x
    y9c1 = data.pose.pose.position.y
    
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th9 = 2*math.atan2(c1,c2)
    if th9 <0:
        th9 = 2*math.pi + th9
    th9 = th9 + math.pi/2
    
def callback3(data):
    global x3c1,y3c1
    #print(data)
    x3c1 = data.pose.pose.position.x
    y3c1 = data.pose.pose.position.y

def muestras():
    global x9c2,y9c2,x3c2,y3c2
    
    x9c2 = x9c1
    y9c2 = y9c1
    
    x3c2 = x3c1
    y3c2 = y3c1

def desfases():
    global lx3,ly3,lx9,ly9
    if t>t1*hz:
        lx3=lx[1][2]
        ly3=ly[1][2]

        lx9=lx[1][8]
        ly9=ly[1][8]

    if t>t2*hz:
        lx3=lx[2][2]
        ly3=ly[2][2]

        lx9=lx[2][8]
        ly9=ly[2][8]

def control():
    global V1,V2,w
    
    dx9c = (x9c1 - x9c2)/delta
    dy9c = (y9c1 - y9c2)/delta
    
    dx3c = (x3c1 - x3c2)/delta
    dy3c = (y3c1 - y3c2)/delta
    
    x9=x9c1-lx9
    x3=x3c1-lx3
    y9=y9c1-ly9
    y3=y3c1-ly3
    
    r1 = - k0*(x9 - x3) - k1*(dx9c - dx3c) 
    r2 = - k0*(y9 - y3) - k1*(dy9c - dy3c) 
    
    V1 = (r1*math.cos(th9) + r2*math.sin(th9))*delta + V2
    w = -r1*math.sin(th9)/V1 + r2*math.cos(th9)/V1
    if (V1>0.4):
        V1=0.4
    if (V1<-0.4):
        V1=-0.4
    if (w>8.69):
        w=8.68
    if (w<-8.69):
        w=-8.68
    
    V2=V1
    
    
if __name__=="__main__":

    rospy.init_node('Control9',anonymous=True)
    pub = rospy.Publisher('/9/cmd_vel9', Twist, queue_size=5)
    rospy.Subscriber("/3/odom3", Odometry, callback3)
    rospy.Subscriber("/9/odom9", Odometry, callback9)
    rate = rospy.Rate(hz) # Hz
    
    try:
        print (msg)
        print (delta)
        while not rospy.is_shutdown():
            #print("\n--------Control9--------")
            if t > hz*0.1:
                control()
            desfases()
            muestras()

            twist = Twist()
            twist.linear.x = V1; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
            pub.publish(twist)
            rate.sleep()
            t=t+1
            
    except rospy.ROSInterruptException:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        print(e)
        pass

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

