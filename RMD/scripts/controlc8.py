import rospy
import math
import time
import ast
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
En linea control 8
"""
e = """
EStoy fuera control 8
"""

k0=rospy.get_param("control8/kp")
k1=rospy.get_param("control8/kd")
hz=rospy.get_param("control8/hz")

tiempos=ast.literal_eval(rospy.get_param("/control8/tiempos"))
t1=tiempos[0]
t2=tiempos[1]

delta=1/hz

V1=0
V2=0.01
w=0.0

lx=ast.literal_eval(rospy.get_param("/control8/lx"))
ly=ast.literal_eval(rospy.get_param("/control8/ly"))

lx8=lx[0][7]
ly8=ly[0][7]

lx2=lx[0][1]
ly2=ly[0][1]

x8c1=0
y8c1=0
x8c2=0
y8c2=0
th8=0

x2c1=0
y2c1=0
x2c2=0
y2c2=0

L=0.092
t=1

def callback2(data):
    global x2c1,y2c1
    x2c1 = data.pose.pose.position.x 
    y2c1 = data.pose.pose.position.y

def callback8(data):
    global x8c1,y8c1,th8
    #print(data)
    x8c1 = data.pose.pose.position.x
    y8c1 = data.pose.pose.position.y
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th8 = 2*math.atan2(c1,c2)
    if th8 <0:
        th8 = 2*math.pi + th8
    th8 = th8 + math.pi/2

def muestras():
    global x8c2,y8c2,x2c2,y2c2
    
    x8c2 = x8c1
    y8c2 = y8c1
    
    x2c2 = x2c1
    y2c2 = y2c1
    
def desfases():
    global lx2,ly2,lx8,ly8
    if t>t1*hz:
        lx2=lx[1][1]
        ly2=ly[1][1]

        lx8=lx[1][7]
        ly8=ly[1][7]

    if t>t2*hz:
        lx2=lx[2][1]
        ly2=ly[2][1]

        lx8=lx[2][7]
        ly8=ly[2][7]

def control():
    global V1,V2,x8c2,y8c2,x2c2,y2c2,w
    dx8c = (x8c1 - x8c2)/delta
    dy8c = (y8c1 - y8c2)/delta
    
    dx2c = (x2c1 - x2c2)/delta
    dy2c = (y2c1 - y2c2)/delta
    
    
    x8=x8c1-lx8
    x2=x2c1-lx2
    y8=y8c1-ly8
    y2=y2c1-ly2
    
    r1 = - k0*(x8 - x2) - k1*(dx8c - dx2c)
    r2 = - k0*(y8 - y2) - k1*(dy8c - dy2c)
    #print(r2)
    
    V1 = (r1*math.cos(th8) + r2*math.sin(th8))*delta + V2
    w = -r1*math.sin(th8)/V1 + r2*math.cos(th8)/V1
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

    rospy.init_node('Control8',anonymous=True)
    pub = rospy.Publisher('/8/cmd_vel8', Twist, queue_size=5)
    rospy.Subscriber("/8/odom8", Odometry, callback8)
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

