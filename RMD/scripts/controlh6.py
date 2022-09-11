import rospy
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
En linea control 6
"""
e = """
EStoy fuera control 6
"""

V=0.0
w=0.0
k1=rospy.get_param("control6/kp")
hz=rospy.get_param("control6/hz")
xd=1.0
yd=1.0

lx6=rospy.get_param("/control6/xl6")
ly6=rospy.get_param("/control6/yl6")

lx3=rospy.get_param("/control6/xl3")
ly3=rospy.get_param("/control6/yl3")

x6c=0
y6c=0
th6=0
x6h=0
y6h=0

x3c=0
y3c=0
th3=0
x3h=0
y3h=0

h=0.064
L=0.092
t=0
 
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
    
    
def control():
    global V,w
    r1=-k1*(x6h-x3h)
    r2=-k1*(y6h-y3h)
    V = r1*math.cos(th6) + r2*math.sin(th6)
    w = -r1*math.sin(th6)/h + r2*math.cos(th6)/h
    if (V>0.4):
        V=0.4
    if (V<-0.4):
        V=-0.4
    if (w>8.69):
        w=8.68
    if (w<-8.69):
        w=-8.68
    #print("x6h ",x6h)
    #print("y6h ",y6h)
    

    
if __name__=="__main__":

    rospy.init_node('Control6',anonymous=True)
    pub = rospy.Publisher('/6/cmd_vel6', Twist, queue_size=5)
    rospy.Subscriber("/3/odom3", Odometry, callback3)
    rospy.Subscriber("/6/odom6", Odometry, callback6)
    rate = rospy.Rate(hz) # 10hz
    
    try:
        print (msg)
        while not rospy.is_shutdown():
            #trayectoria()
            #print("\n--------Control3--------")
            control()
            #print("th6 ",th6*180/math.pi)
            twist = Twist()
            twist.linear.x = V; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
            pub.publish(twist)
            rate.sleep()
            
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
