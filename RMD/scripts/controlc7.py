import rospy
import math
import time
import ast
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
En linea control 7
"""
e = """
EStoy fuera control 7
"""

k0=rospy.get_param("control7/kp")
k1=rospy.get_param("control7/kd")
hz=rospy.get_param("control7/hz")

tiempos=ast.literal_eval(rospy.get_param("/control7/tiempos"))
t1=tiempos[0]
t2=tiempos[1]

delta=1/hz

lx=ast.literal_eval(rospy.get_param("/control7/lx"))
ly=ast.literal_eval(rospy.get_param("/control7/ly"))

lx7=lx[0][6]
ly7=ly[0][6]

lx4=lx[0][3]
ly4=ly[0][3]

V1=0
V2=0.01
w=0.0

x7c1=0
y7c1=0
x7c2=0
y7c2=0
dx7c=0
dy7c=0.01
th7=0

x4c1=0
y4c1=0
x4c2=0
y4c2=0
dx4c=0
dy4c=0.01

L=0.092
t=1
 
def callback7(data):
    global x7c1,y7c1,th7
    x7c1 = data.pose.pose.position.x
    y7c1 = data.pose.pose.position.y
    
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th7 = 2*math.atan2(c1,c2)
    if th7 <0:
        th7 = 2*math.pi + th7
    th7 = th7 + math.pi/2
    
def callback4(data):
    global x4c1,y4c1
    #print(data)
    x4c1 = data.pose.pose.position.x
    y4c1 = data.pose.pose.position.y

def muestras():
    global x7c2,y7c2,x4c2,y4c2
    x7c2 = x7c1
    y7c2 = y7c1
    
    x4c2 = x4c1
    y4c2 = y4c1
    
def desfases():
    global lx4,ly4,lx7,ly7
    if t>t1*hz:
        lx4=lx[1][3]
        ly4=ly[1][3]

        lx7=lx[1][6]
        ly7=ly[1][6]

    if t>t2*hz:
        lx4=lx[2][3]
        ly4=ly[2][3]

        lx7=lx[2][6]
        ly7=ly[2][6]
  
def control():
    global V1,V2,w
    
    dx7c = (x7c1 - x7c2)/delta
    dy7c = (y7c1 - y7c2)/delta
    
    dx4c = (x4c1 - x4c2)/delta
    dy4c = (y4c1 - y4c2)/delta
    
    x7=x7c1-lx7
    x4=x4c1-lx4
    y7=y7c1-ly7
    y4=y4c1-ly4
    
    r1 = - k0*(x7 - x4) - k1*(dx7c - dx4c) 
    r2 = - k0*(y7 - y4) - k1*(dy7c - dy4c) 
    
    V1 = (r1*math.cos(th7) + r2*math.sin(th7))*delta + V2
    w = -r1*math.sin(th7)/V1 + r2*math.cos(th7)/V1
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

    rospy.init_node('Control7',anonymous=True)
    pub = rospy.Publisher('/7/cmd_vel7', Twist, queue_size=5)
    rospy.Subscriber("/7/odom7", Odometry, callback7)
    rospy.Subscriber("/4/odom4", Odometry, callback4)
    rate = rospy.Rate(hz) # Hz
    
    try:
        print (msg)
        print (delta)
        while not rospy.is_shutdown():
            #print("\n--------Control7--------")
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

