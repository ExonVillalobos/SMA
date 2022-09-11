import rospy
import math
import time
import ast
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
En linea control 6
"""
e = """
EStoy fuera control 6
"""

k0=rospy.get_param("control6/kp")
k1=rospy.get_param("control6/kd")
hz=rospy.get_param("control6/hz")

tiempos=ast.literal_eval(rospy.get_param("/control6/tiempos"))
t1=tiempos[0]
t2=tiempos[1]

delta=1/hz

V1=0
V2=0.01
w=0.0


lx=ast.literal_eval(rospy.get_param("/control6/lx"))
ly=ast.literal_eval(rospy.get_param("/control6/ly"))

lx3=lx[0][2]
ly3=ly[0][2]

lx6=lx[0][5]
ly6=ly[0][5]

x6c1=0
y6c1=0
x6c2=0
y6c2=0
dx6c=0
dy6c=0.01
th6=0

x3c1=0
y3c1=0
x3c2=0
y3c2=0
dx3c=0
dy3c=0.01

L=0.092
t=1
 
def callback6(data):
    global x6c1,y6c1,th6
    x6c1 = data.pose.pose.position.x
    y6c1 = data.pose.pose.position.y
    
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th6 = 2*math.atan2(c1,c2)
    if th6 <0:
        th6 = 2*math.pi + th6
    th6 = th6 + math.pi/2
    
def callback3(data):
    global x3c1,y3c1
    #print(data)
    x3c1 = data.pose.pose.position.x
    y3c1 = data.pose.pose.position.y

def muestras():
    global x6c2,y6c2,x3c2,y3c2
    x6c2 = x6c1
    y6c2 = y6c1
    
    x3c2 = x3c1
    y3c2 = y3c1
    
def desfases():
    global lx3,ly3,lx6,ly6
    if t>t1*hz:
        lx3=lx[1][2]
        ly3=ly[1][2]

        lx6=lx[1][5]
        ly6=ly[1][5]

    if t>t2*hz:
        lx3=lx[2][2]
        ly3=ly[2][2]

        lx6=lx[2][5]
        ly6=ly[2][5]

def control():
    global V1,V2,w
    
    dx6c = (x6c1 - x6c2)/delta
    dy6c = (y6c1 - y6c2)/delta
    
    dx3c = (x3c1 - x3c2)/delta
    dy3c = (y3c1 - y3c2)/delta
    
    x6=x6c1-lx6
    x3=x3c1-lx3
    y6=y6c1-ly6
    y3=y3c1-ly3
    
    r1 = - k0*(x6 - x3) - k1*(dx6c - dx3c) 
    r2 = - k0*(y6 - y3) - k1*(dy6c - dy3c) 
    
    V1 = (r1*math.cos(th6) + r2*math.sin(th6))*delta + V2
    w = -r1*math.sin(th6)/V1 + r2*math.cos(th6)/V1
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

    rospy.init_node('Control6',anonymous=True)
    pub = rospy.Publisher('/6/cmd_vel6', Twist, queue_size=5)
    rospy.Subscriber("/3/odom3", Odometry, callback3)
    rospy.Subscriber("/6/odom6", Odometry, callback6)
    rate = rospy.Rate(hz) # Hz
    
    try:
        print (msg)
        print (delta)
        while not rospy.is_shutdown():
            #print("\n--------Control1--------")
            if t > hz*0.5:
                control()
            desfases()
            muestras()
            #print("th3 ",th3*180/math.pi)
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

