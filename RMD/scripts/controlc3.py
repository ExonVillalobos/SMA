import rospy
import math
import time
import ast
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
En linea control 3
"""
e = """
EStoy fuera control 3
"""

k0=rospy.get_param("control3/kp")
k1=rospy.get_param("control3/kd")
hz=rospy.get_param("control3/hz")

tiempos=ast.literal_eval(rospy.get_param("/control3/tiempos"))
t1=tiempos[0]
t2=tiempos[1]

delta=1/hz

V1=0
V2=0.01
w=0.0

lx=ast.literal_eval(rospy.get_param("/control3/lx"))
ly=ast.literal_eval(rospy.get_param("/control3/ly"))

lx1=lx[0][0]
ly1=ly[0][0]

lx3=lx[0][2]
ly3=ly[0][2]

lx6=lx[0][5]
ly6=ly[0][5]

lx9=lx[0][8]
ly9=ly[0][8]


x1c1=0
y1c1=0
x1c2=0
y1c2=0

x6c1=0
y6c1=0
x6c2=0
y6c2=0

x9c1=0
y9c1=0
x9c2=0
y9c2=0

x3c1=0
y3c1=0
x3c2=0
y3c2=0
th3=0

L=0.092
t=0
 
def callback3(data):
    global x3c1,y3c1,th3
    x3c1 = data.pose.pose.position.x
    y3c1 = data.pose.pose.position.y
    
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th3 = 2*math.atan2(c1,c2)
    if th3 <0:
        th3 = 2*math.pi + th3
    th3 = th3 + math.pi/2
    
def callback1(data):
    global x1c1,y1c1
    #print(data)
    x1c1 = data.pose.pose.position.x
    y1c1 = data.pose.pose.position.y
    
def callback6(data):
    global x6c1,y6c1
    #print(data)
    x6c1 = data.pose.pose.position.x
    y6c1 = data.pose.pose.position.y
    
def callback9(data):
    global x9c1,y9c1
    #print(data)
    x9c1 = data.pose.pose.position.x
    y9c1 = data.pose.pose.position.y

def muestras():
    global x1c2,y1c2,x3c2,y3c2,x6c2,y6c2,x9c2,y9c2
    # actualización de muestras anteriores 
    x1c2 = x1c1
    y1c2 = y1c1
    
    x6c2 = x6c1
    y6c2 = y6c1
    
    x9c2 = x9c1
    y9c2 = y9c1
       
    x3c2 = x3c1
    y3c2 = y3c1
    
def desfases():
    global lx1,ly1,lx3,ly3,lx6,ly6,lx9,ly9
    if t>t1*hz:
        lx1=lx[1][0]
        ly1=ly[1][0]

        lx3=lx[1][2]
        ly3=ly[1][2]

        lx6=lx[1][5]
        ly6=ly[1][5]

        lx9=lx[1][8]
        ly9=ly[1][8]
    if t>t2*hz:
        lx1=lx[2][0]
        ly1=ly[2][0]

        lx3=lx[2][2]
        ly3=ly[2][2]

        lx6=lx[2][5]
        ly6=ly[2][5]

        lx9=lx[2][8]
        ly9=ly[2][8]
    
def control():
    global V1,V2,w
    
    dx1c = (x1c1 - x1c2)/delta
    dy1c = (y1c1 - y1c2)/delta
    
    dx3c = (x3c1 - x3c2)/delta
    dy3c = (y3c1 - y3c2)/delta
    
    dx6c = (x6c1 - x6c2)/delta
    dy6c = (y6c1 - y6c2)/delta
    
    dx9c = (x9c1 - x9c2)/delta
    dy9c = (y9c1 - y9c2)/delta
    
    x1=x1c1-lx1
    x3=x3c1-lx3
    x6=x6c1-lx6
    x9=x9c1-lx9
    y1=y1c1-ly1
    y3=y3c1-ly3
    y6=y6c1-ly6
    y9=y9c1-ly9
    
    r11 = k0*(x1 - x3) + k1*(dx1c - dx3c) + k0*(x6 - x3) + k1*(dx6c - dx3c) 
    r1 = r11 + k0*(x9 - x3) + k1*(dx9c - dx3c)
    r22 = k0*(y1 - y3) + k1*(dy1c - dy3c) + k0*(y6 - y3) + k1*(dy6c - dy3c)
    r2 = r22 + k0*(y9 - y3) + k1*(dy9c - dy3c)
    
    V1 = (r1*math.cos(th3) + r2*math.sin(th3))*delta + V2
    w = -r1*math.sin(th3)/V1 + r2*math.cos(th3)/V1
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

    rospy.init_node('Control3',anonymous=True)
    pub = rospy.Publisher('/3/cmd_vel3', Twist, queue_size=5)
    rospy.Subscriber("/1/odom1", Odometry, callback1)
    rospy.Subscriber("/3/odom3", Odometry, callback3)
    rospy.Subscriber("/6/odom6", Odometry, callback6)
    rospy.Subscriber("/9/odom9", Odometry, callback9)
    rate = rospy.Rate(hz) # Hz
    archivo = open("/home/exon/SMA/src/RMD/scripts/plotear/ac369.txt","w") 
    
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
            archivo.write(str((t*delta)) + " " + str((x6c1)) + " " + str((y6c1)) + " ")
            archivo.write(str((x9c1)) + " " + str((y9c1)))
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

