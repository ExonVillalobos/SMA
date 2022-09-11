import rospy
import math
import ast
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
Contorl 4 en linea
"""
e = """
control 4 fuera
"""

V=0.0
w=0.0
k1=rospy.get_param("control4/kp")
hz=rospy.get_param("control4/hz")
delta=1/hz

lx=ast.literal_eval(rospy.get_param("/control4/lx"))
ly=ast.literal_eval(rospy.get_param("/control4/ly"))

lx1=lx[0]
ly1=ly[0]

lx4=lx[3]
ly4=ly[3]

lx7=lx[6]
ly7=ly[6]

lx10=lx[9]
ly10=ly[9]

x1c=0
y1c=0
th1=0
x1h=0
y1h=0

x4c=0
y4c=0
th4=0
x4h=0
y4h=0

x7c=0
y7c=0
th7=0
x7h=0
y7h=0

x10c=0
y10c=0
th10=0
x10h=0
y10h=0

h=0.064
L=0.092
t=0
 
def callback1(data):
    global x1c,y1c,y1h,x1h,th1
    #print(data)
    x1c = data.pose.pose.position.x
    y1c = data.pose.pose.position.y
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th1 = 2*math.atan2(c1,c2)
    if th1 <0:
        th1 = 2*math.pi + th1
    th1 = th1 + math.pi/2
    x1h = x1c + h*math.cos(th1) - lx1
    y1h = y1c + h*math.sin(th1) - ly1
    
def callback4(data):
    global x4c,y4c,y4h,x4h,th4
    #print(data)
    x4c = data.pose.pose.position.x
    y4c = data.pose.pose.position.y
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th4 = 2*math.atan2(c1,c2)
    if th4 <0:
        th4=2*math.pi + th4
    th4 = th4 + math.pi/2
    x4h = x4c + h*math.cos(th4) - lx4
    y4h = y4c + h*math.sin(th4) - ly4
    
def callback7(data):
    global x7c,y7c,y7h,x7h,th7
    #print(data)
    x7c = data.pose.pose.position.x
    y7c = data.pose.pose.position.y
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th7 = 2*math.atan2(c1,c2)
    if th7 <0:
        th7=2*math.pi + th7
    th7=th7 + math.pi/2
    x7h = x7c + h*math.cos(th7) - lx7
    y7h = y7c + h*math.sin(th7) - ly7
    
def callback10(data):
    global x10c,y10c,y10h,x10h,th10
    #print(data)
    x10c = data.pose.pose.position.x
    y10c = data.pose.pose.position.y
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th10 = 2*math.atan2(c1,c2)
    if th10 <0:
        th10=2*math.pi + th10
    th10=th10 + math.pi/2
    x10h = x10c + h*math.cos(th10) - lx10
    y10h = y10c + h*math.sin(th10) - ly10
    
def control():
    global V,w
    r1=-k1*(x4h-x1h)-k1*(x4h-x7h)-k1*(x4h-x10h)
    r2=-k1*(y4h-y1h)-k1*(y4h-y7h)-k1*(y4h-y10h)
    V = r1*math.cos(th4) + r2*math.sin(th4)
    w = -r1*math.sin(th4)/h + r2*math.cos(th4)/h
    if (V>0.4):
        V=0.4
    if (V<-0.4):
        V=-0.4
    if (w>8.69):
        w=8.68
    if (w<-8.69):
        w=-8.68
    #print("x4h ",x4h)
    #print("y4h ",y4h)

    
if __name__=="__main__":

    rospy.init_node('Control4',anonymous=True)
    pub = rospy.Publisher('/4/cmd_vel4', Twist, queue_size=5)
    rospy.Subscriber("/4/odom4", Odometry, callback4)
    rospy.Subscriber("/1/odom1", Odometry, callback1)
    rospy.Subscriber("/7/odom7", Odometry, callback7)
    rospy.Subscriber("/10/odom10", Odometry, callback10)
    rate = rospy.Rate(hz)
    archivo = open("/home/exon/SMA/src/RMD/scripts/plotear/ah4710.txt","w") 
    
    try:
        print (msg)
        while not rospy.is_shutdown():
            #print("\n--------Control4--------")
            control()
            #print("th4 ",th4*180/math.pi)
            twist = Twist()
            twist.linear.x = V; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
            pub.publish(twist)
            archivo.write(str((t*delta)) + " " + str((x7h)) + " " + str((y7h)) + " " + str((th7)) + " ")
            archivo.write(str((x10h)) + " " + str((y10h)) + " " + str((th10)))
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
