import rospy
import math
import time
import ast
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
En linea control 2
"""
e = """
EStoy fuera control 2
"""

k0=rospy.get_param("control2/kp")
k1=rospy.get_param("control2/kd")
hz=rospy.get_param("control2/hz")

tiempos=ast.literal_eval(rospy.get_param("/control2/tiempos"))
t1=tiempos[0]
t2=tiempos[1]

delta=1/hz

V1=0
V2=0.01
w=0.0


lx=ast.literal_eval(rospy.get_param("/control2/lx"))
ly=ast.literal_eval(rospy.get_param("/control2/ly"))
lx1=lx[0][0]
ly1=ly[0][0]

lx2=lx[0][1]
ly2=ly[0][1]

lx5=lx[0][4]
ly5=ly[0][4]

lx8=lx[0][7]
ly8=ly[0][7]

x1c1=0
y1c1=0
x1c2=0
y1c2=0

x5c1=0
y5c1=0
x5c2=0
y5c2=0

x8c1=0
y8c1=0
x8c2=0
y8c2=0

x2c1=0
y2c1=0
x2c2=0
y2c2=0
dx2c=0
dy2c=0.01
th2=0

L=0.092
t=0

def callback1(data):
    global x1c1,y1c1
    x1c1 = data.pose.pose.position.x 
    y1c1 = data.pose.pose.position.y

def callback5(data):
    global x5c1,y5c1
    x5c1 = data.pose.pose.position.x 
    y5c1 = data.pose.pose.position.y
    
def callback8(data):
    global x8c1,y8c1
    x8c1 = data.pose.pose.position.x 
    y8c1 = data.pose.pose.position.y
    
def callback2(data):
    global x2c1,y2c1,th2
    #print(data)
    x2c1 = data.pose.pose.position.x
    y2c1 = data.pose.pose.position.y
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th2 = 2*math.atan2(c1,c2)
    if th2 <0:
        th2 = 2*math.pi + th2
    th2 = th2 + math.pi/2
    
def muestras():
    global x1c2,y1c2,x2c2,y2c2,x5c2,y5c2,x8c2,y8c2
    x1c2 = x1c1
    y1c2 = y1c1
    
    x2c2 = x2c1
    y2c2 = y2c1
    
    x5c2 = x5c1
    y5c2 = y5c1
    
    x8c2 = x8c1
    y8c2 = y8c1
    
def desfases():
    global lx1,ly1,lx2,ly2,lx5,ly5,lx8,ly8
    if t>t1*hz:
        lx1=lx[1][0]
        ly1=ly[1][0]

        lx2=lx[1][1]
        ly2=ly[1][1]

        lx5=lx[1][4]
        ly5=ly[1][4]

        lx8=lx[1][7]
        ly8=ly[1][7]
    if t>t2*hz:
        lx1=lx[2][0]
        ly1=ly[2][0]

        lx2=lx[2][1]
        ly2=ly[2][1]

        lx5=lx[2][4]
        ly5=ly[2][4]

        lx8=lx[2][7]
        ly8=ly[2][7]

def control():
    global V1,V2,w
    dx1c = (x1c1 - x1c2)/delta
    dy1c = (y1c1 - y1c2)/delta
    
    dx2c = (x2c1 - x2c2)/delta
    dy2c = (y2c1 - y2c2)/delta
    
    dx5c = (x5c1 - x5c2)/delta
    dy5c = (y5c1 - y5c2)/delta
    
    dx8c = (x8c1 - x8c2)/delta
    dy8c = (y8c1 - y8c2)/delta
    
    x1=x1c1-lx1
    x2=x2c1-lx2
    x5=x5c1-lx5
    x8=x8c1-lx8
    y1=y1c1-ly1
    y2=y2c1-ly2
    y5=y5c1-ly5
    y8=y8c1-ly8
    
    r11 = - k0*(x2 - x1) - k1*(dx2c - dx1c) - k0*(x2 - x5) - k1*(dx2c - dx5c)
    r1= r11 - k0*(x2 - x8) - k1*(dx2c - dx8c)
    r22 = - k0*(y2 - y1) - k1*(dy2c - dy1c) - k0*(y2 - y5) - k1*(dy2c - dy5c)
    r2= r22 - k0*(y2 - y8) - k1*(dy2c - dy8c) 
    #print(r2)
    
    V1 = (r1*math.cos(th2) + r2*math.sin(th2))*delta + V2
    w = -r1*math.sin(th2)/V1 + r2*math.cos(th2)/V1
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

    rospy.init_node('Control2',anonymous=True)
    pub = rospy.Publisher('/2/cmd_vel2', Twist, queue_size=5)
    rospy.Subscriber("/1/odom1", Odometry, callback1)
    rospy.Subscriber("/2/odom2", Odometry, callback2)
    rospy.Subscriber("/5/odom5", Odometry, callback5)
    rospy.Subscriber("/8/odom8", Odometry, callback8)
    rate = rospy.Rate(hz) # Hz
    archivo = open("/home/exon/SMA/src/RMD/scripts/plotear/ac258.txt","w") 
    
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
            archivo.write(str((t*delta)) + " " + str((x5c1)) + " " + str((y5c1)) + " ")
            archivo.write(str((x8c1)) + " " + str((y8c1)))
            archivo.write("\n")
            rate.sleep()
            t=t+1
        archivo.close()
            
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

