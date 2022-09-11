import rospy
import math
import time
import ast
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
En linea control 4
"""
e = """
EStoy fuera control 4
"""

k0=rospy.get_param("control4/kp")
k1=rospy.get_param("control4/kd")
hz=rospy.get_param("control4/hz")

tiempos=ast.literal_eval(rospy.get_param("/control4/tiempos"))
t1=tiempos[0]
t2=tiempos[1]

delta=1/hz

V1=0
V2=0.01
w=0.0

lx=ast.literal_eval(rospy.get_param("/control4/lx"))
ly=ast.literal_eval(rospy.get_param("/control4/ly"))

lx1=lx[0][0]
ly1=ly[0][0]

lx4=lx[0][3]
ly4=ly[0][3]

lx7=lx[0][6]
ly7=ly[0][6]

lx10=lx[0][9]
ly10=ly[0][9]

x1c1=0
y1c1=0
x1c2=0
y1c2=0

x7c1=0
y7c1=0
x7c2=0
y7c2=0

x10c1=0
y10c1=0
x10c2=0
y10c2=0

x4c1=0
y4c1=0
x4c2=0
y4c2=0
th4=0

L=0.092
t=0
 
def callback4(data):
    global x4c1,y4c1,th4
    x4c1 = data.pose.pose.position.x
    y4c1 = data.pose.pose.position.y
    
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th4 = 2*math.atan2(c1,c2)
    if th4 <0:
        th4 = 2*math.pi + th4
    th4 = th4 + math.pi/2
    
def callback1(data):
    global x1c1,y1c1
    #print(data)
    x1c1 = data.pose.pose.position.x
    y1c1 = data.pose.pose.position.y
    
def callback7(data):
    global x7c1,y7c1
    #print(data)
    x7c1 = data.pose.pose.position.x
    y7c1 = data.pose.pose.position.y
    
def callback10(data):
    global x10c1,y10c1
    #print(data)
    x10c1 = data.pose.pose.position.x
    y10c1 = data.pose.pose.position.y

def muestras():
    global x1c2,y1c2,x4c2,y4c2,x7c2,y7c2,x10c2,y10c2
    x1c2 = x1c1
    y1c2 = y1c1
    
    x7c2 = x7c1
    y7c2 = y7c1
    
    x10c2 = x10c1
    y10c2 = y10c1
    
    x4c2 = x4c1
    y4c2 = y4c1

def desfases():
    global lx1,ly1,lx4,ly4,lx7,ly7,lx10,ly10
    if t>t1*hz:
        lx1=lx[1][0]
        ly1=ly[1][0]

        lx4=lx[1][3]
        ly4=ly[1][3]

        lx7=lx[1][6]
        ly7=ly[1][6]

        lx10=lx[1][9]
        ly10=ly[1][9]
        
    if t>t2*hz:
        lx1=lx[2][0]
        ly1=ly[2][0]

        lx4=lx[2][3]
        ly4=ly[2][3]

        lx7=lx[2][6]
        ly7=ly[2][6]

        lx10=lx[2][9]
        ly10=ly[2][9]
    
def control():
    global V1,V2,w
    
    dx1c = (x1c1 - x1c2)/delta
    dy1c = (y1c1 - y1c2)/delta
    
    dx7c = (x7c1 - x7c2)/delta
    dy7c = (y7c1 - y7c2)/delta
    
    dx10c = (x10c1 - x10c2)/delta
    dy10c = (y10c1 - y10c2)/delta
    
    dx4c = (x4c1 - x4c2)/delta
    dy4c = (y4c1 - y4c2)/delta
    
    x1=x1c1-lx1
    x4=x4c1-lx4
    x7=x7c1-lx7
    x10=x10c1-lx10
    y1=y1c1-ly1
    y4=y4c1-ly4
    y7=y7c1-ly7
    y10=y10c1-ly10
    
    r11 = k0*(x1 - x4) + k1*(dx1c - dx4c) + k0*(x7 - x4) + k1*(dx7c - dx4c) 
    r1 = r11 + k0*(x10 - x4) + k1*(dx10c - dx4c)
    r22 = k0*(y1 - y4) + k1*(dy1c - dy4c) + k0*(y7 - y4) + k1*(dy7c - dy4c) 
    r2= r22 + k0*(y10 - y4) + k1*(dy10c - dy4c)
    
    V1 = (r1*math.cos(th4) + r2*math.sin(th4))*delta + V2
    w = -r1*math.sin(th4)/V1 + r2*math.cos(th4)/V1
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

    rospy.init_node('Control4',anonymous=True)
    pub = rospy.Publisher('/4/cmd_vel4', Twist, queue_size=5)
    rospy.Subscriber("/1/odom1", Odometry, callback1)
    rospy.Subscriber("/4/odom4", Odometry, callback4)
    rospy.Subscriber("/7/odom7", Odometry, callback7)
    rospy.Subscriber("/10/odom10", Odometry, callback10)
    rate = rospy.Rate(hz) # Hz
    archivo = open("/home/exon/SMA/src/RMD/scripts/plotear/ac4710.txt","w") 
    
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
            archivo.write(str((t*delta)) + " " + str((x7c1)) + " " + str((y7c1)) + " ")
            archivo.write(str((x10c1)) + " " + str((y10c1)))
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

