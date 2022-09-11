import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
control 3 en linea
"""
e = """
control 3 fuera
"""

V=0.0
w=0.0
k1=rospy.get_param("control3/kp")
hz=rospy.get_param("control3/hz")
delta=1/hz

xd=1.0
yd=1.0

lx1=rospy.get_param("/control3/xl1")
ly1=rospy.get_param("/control3/yl1")

lx6=rospy.get_param("/control3/xl6")
ly6=rospy.get_param("/control3/yl6")

lx3=rospy.get_param("/control3/xl3")
ly3=rospy.get_param("/control3/yl3")

lx9=rospy.get_param("/control3/xl9")
ly9=rospy.get_param("/control3/yl9")

x1c=0
y1c=0
th1=0
x1h=0
y1h=0

x3c=0
y3c=0
th3=0
x3h=0
y3h=0

x6c=0
y6c=0
th6=0
x6h=0
y6h=0

x9c=0
y9c=0
th9=0
x9h=0
y9h=0

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
    
def callback3(data):
    global x3c,y3c,y3h,x3h,th3
    #print(data)
    x3c = data.pose.pose.position.x
    y3c = data.pose.pose.position.y
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th3 = 2*math.atan2(c1,c2)
    if th3 <0:
        th3 = 2*math.pi + th3
    th3 = th3 + math.pi/2
    x3h = x3c + h*math.cos(th3) - lx3
    y3h = y3c + h*math.sin(th3) - ly3
    
def callback6(data):
    global x6c,y6c,y6h,x6h,th6
    #print(data)
    x6c = data.pose.pose.position.x
    y6c = data.pose.pose.position.y
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th6 = 2*math.atan2(c1,c2)
    if th6 <0:
        th6=2*math.pi + th6
    th6=th6 + math.pi/2
    x6h = x6c + h*math.cos(th6) - lx6
    y6h = y6c + h*math.sin(th6) - ly6
    
def callback9(data):
    global x9c,y9c,y9h,x9h,th9
    #print(data)
    x9c = data.pose.pose.position.x
    y9c = data.pose.pose.position.y
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th9 = 2*math.atan2(c1,c2)
    if th9 <0:
        th9=2*math.pi + th9
    th9=th9 + math.pi/2
    x9h = x9c + h*math.cos(th9) - lx9
    y9h = y9c + h*math.sin(th9) - ly9
    
    
def control():
    global V,w
    r1=-k1*(x3h-x1h)-k1*(x3h-x6h)-k1*(x3h-x9h)
    r2=-k1*(y3h-y1h)-k1*(y3h-y6h)-k1*(y3h-y9h)
    V = r1*math.cos(th3) + r2*math.sin(th3)
    w = -r1*math.sin(th3)/h + r2*math.cos(th3)/h
    if (V>0.4):
        V=0.4
    if (V<-0.4):
        V=-0.4
    if (w>8.69):
        w=8.68
    if (w<-8.69):
        w=-8.68
    #print("x3h ",x3h)
    #print("y3h ",y3h)
    
if __name__=="__main__":

    rospy.init_node('Control3',anonymous=True)
    pub = rospy.Publisher('/3/cmd_vel3', Twist, queue_size=5)
    rospy.Subscriber("/3/odom3", Odometry, callback3)
    rospy.Subscriber("/1/odom1", Odometry, callback1)
    rospy.Subscriber("/6/odom6", Odometry, callback6)
    rospy.Subscriber("/9/odom9", Odometry, callback9)
    rate = rospy.Rate(hz) # 10hz
    archivo = open("/home/exon/SMA/src/RMD/scripts/plotear/ah369.txt","w") 
    
    try:
        print (msg)
        while not rospy.is_shutdown():
            #print("\n--------Control3--------")
            control()
            #print("th3 ",th3*180/math.pi)
            twist = Twist()
            twist.linear.x = V; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
            pub.publish(twist)
            archivo.write(str((t*delta)) + " " + str((x6h)) + " " + str((y6h)) + " " + str((th6)) + " ")
            archivo.write(str((x9h)) + " " + str((y9h)) + " " + str((th9)))
            archivo.write("\n")
            t=t+1
            rate.sleep()
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

