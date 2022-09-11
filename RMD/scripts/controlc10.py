import rospy
import math
import time
import ast
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
En linea control 10
"""
e = """
EStoy fuera control 10
"""

k0=rospy.get_param("control10/kp")
k1=rospy.get_param("control10/kd")
hz=rospy.get_param("control10/hz")

tiempos=ast.literal_eval(rospy.get_param("/control10/tiempos"))
t1=tiempos[0]
t2=tiempos[1]

delta=1/hz
V1=0
V2=0.01
w=0.0

lx=ast.literal_eval(rospy.get_param("/control10/lx"))
ly=ast.literal_eval(rospy.get_param("/control10/ly"))

lx10=lx[0][9]
ly10=ly[0][9]

lx4=lx[0][3]
ly4=ly[0][3]

x10c1=0
y10c1=0
x10c2=0
y10c2=0
dx10c=0
dy10c=0.01
th10=0

x4c1=0
y4c1=0
x4c2=0
y4c2=0
dx4c=0
dy4c=0.01

L=0.092
t=1
 
def callback10(data):
    global x10c1,y10c1,th10
    x10c1 = data.pose.pose.position.x
    y10c1 = data.pose.pose.position.y

    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th10 = 2*math.atan2(c1,c2)
    if th10 <0:
        th10 = 2*math.pi + th10
    th10 = th10 + math.pi/2
    
def callback4(data):
    global x4c1,y4c1
    #print(data)
    x4c1 = data.pose.pose.position.x
    y4c1 = data.pose.pose.position.y

def muestras():
    global x10c2,y10c2,x4c2,y4c2
    
    x10c2 = x10c1
    y10c2 = y10c1
    
    x4c2 = x4c1
    y4c2 = y4c1
    
def desfases():
    global lx4,ly4,lx10,ly10
    if t>t1*hz:
        lx4=lx[1][3]
        ly4=ly[1][3]

        lx10=lx[1][9]
        ly10=ly[1][9]

    if t>t2*hz:
        lx4=lx[2][3]
        ly4=ly[2][3]

        lx10=lx[2][9]
        ly10=ly[2][9]

def control():
    global V1,V2,w
    
    dx10c = (x10c1 - x10c2)/delta
    dy10c = (y10c1 - y10c2)/delta
    
    dx4c = (x4c1 - x4c2)/delta
    dy4c = (y4c1 - y4c2)/delta
    
    x10=x10c1-lx10
    x4=x4c1-lx4
    y10=y10c1-ly10
    y4=y4c1-ly4
    
    r1 = - k0*(x10 - x4) - k1*(dx10c - dx4c) 
    r2 = - k0*(y10 - y4) - k1*(dy10c - dy4c) 
    
    V1 = (r1*math.cos(th10) + r2*math.sin(th10))*delta + V2
    w = -r1*math.sin(th10)/V1 + r2*math.cos(th10)/V1
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

    rospy.init_node('Control10',anonymous=True)
    pub = rospy.Publisher('/10/cmd_vel10', Twist, queue_size=5)
    rospy.Subscriber("/10/odom10", Odometry, callback10)
    rospy.Subscriber("/4/odom4", Odometry, callback4)
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
            #print("th4 ",th4*180/math.pi)
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

