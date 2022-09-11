import rospy
import math
import time
import ast
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
En linea control 5
"""
e = """
EStoy fuera control 5
"""

k0=rospy.get_param("control5/kp")
k1=rospy.get_param("control5/kd")
hz=rospy.get_param("control5/hz")

tiempos=ast.literal_eval(rospy.get_param("/control5/tiempos"))
t1=tiempos[0]
t2=tiempos[1]

delta=1/hz

V1=0
V2=0.01
w=0.0

lx=ast.literal_eval(rospy.get_param("/control5/lx"))
ly=ast.literal_eval(rospy.get_param("/control5/ly"))

lx2=lx[0][1]
ly2=ly[0][1]

lx5=lx[0][4]
ly5=ly[0][4]

x5c1=0
y5c1=0
x5c2=0
y5c2=0
dx5c=0
dy5c=0.01
th5=0

x2c1=0
y2c1=0
x2c2=0
y2c2=0
dx2c=0
dy2c=0.01

L=0.092
t=1

def callback2(data):
    global x2c1,y2c1
    x2c1 = data.pose.pose.position.x 
    y2c1 = data.pose.pose.position.y

def callback5(data):
    global x5c1,y5c1,th5
    #print(data)
    x5c1 = data.pose.pose.position.x
    y5c1 = data.pose.pose.position.y
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th5 = 2*math.atan2(c1,c2)
    if th5 <0:
        th5 = 2*math.pi + th5
    th5 = th5 + math.pi/2

def muestras():
    global x5c2,y5c2,x2c2,y2c2

    x5c2 = x5c1
    y5c2 = y5c1
    
    x2c2 = x2c1
    y2c2 = y2c1
    
def desfases():
    global lx2,ly2,lx5,ly5
    if t>t1*hz:
        lx2=lx[1][1]
        ly2=ly[1][1]

        lx5=lx[1][4]
        ly5=ly[1][4]

    if t>t2*hz:
        lx2=lx[2][1]
        ly2=ly[2][1]

        lx5=lx[2][4]
        ly5=ly[2][4]

def control():
    global V1,V2,w
    dx5c = (x5c1 - x5c2)/delta
    dy5c = (y5c1 - y5c2)/delta
    
    dx2c = (x2c1 - x2c2)/delta
    dy2c = (y2c1 - y2c2)/delta
    
    x5=x5c1-lx5
    x2=x2c1-lx2
    y5=y5c1-ly5
    y2=y2c1-ly2
    
    r1 = - k0*(x5 - x2) - k1*(dx5c - dx2c)
    r2 = - k0*(y5 - y2) - k1*(dy5c - dy2c)
    #print(r2)
    
    V1 = (r1*math.cos(th5) + r2*math.sin(th5))*delta + V2
    w = -r1*math.sin(th5)/V1 + r2*math.cos(th5)/V1
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

    rospy.init_node('Control5',anonymous=True)
    pub = rospy.Publisher('/5/cmd_vel5', Twist, queue_size=5)
    rospy.Subscriber("/5/odom5", Odometry, callback5)
    rospy.Subscriber("/2/odom2", Odometry, callback2)
    rate = rospy.Rate(hz) # Hz
    
    try:
        print (msg)
        print (delta)
        while not rospy.is_shutdown():
            #print("\n--------Control2--------")
            if t > hz*0.5:
                control()
            desfases()
            muestras()
            #print("th2 ",th2*180/math.pi)
            #print("error en X = ", x1c1-xd1)
            #print("error en Y = ", y1c1-yd1)
            #print("\n--------ERROR--------")
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

