import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
control 7 en linea
"""
e = """
control 7 fuera
"""

V=0.0
w=0.0
k1=rospy.get_param("control7/kp")
hz=rospy.get_param("control7/hz")
xd=1.0
yd=1.0

lx7=rospy.get_param("/control7/xl7")
ly7=rospy.get_param("/control7/yl7")


lx4=rospy.get_param("/control7/xl4")
ly4=rospy.get_param("/control7/yl4")

x7c=0
y7c=0
th7=0
x7h=0
y7h=0

x4c=0
y4c=0
th4=0
x4h=0
y4h=0

h=0.064
L=0.092
t=0
 
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
    
    
def control():
    global V,w
    r1=-k1*(x7h-x4h)
    r2=-k1*(y7h-y4h)
    V = r1*math.cos(th7) + r2*math.sin(th7)
    w = -r1*math.sin(th7)/h + r2*math.cos(th7)/h
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

    rospy.init_node('Control7',anonymous=True)
    pub = rospy.Publisher('/7/cmd_vel7', Twist, queue_size=5)
    rospy.Subscriber("/4/odom4", Odometry, callback4)
    rospy.Subscriber("/7/odom7", Odometry, callback7)
    rate = rospy.Rate(hz) # 10hz
    
    try:
        print (msg)
        while not rospy.is_shutdown():
            #trayectoria()
            #print("\n--------Control4--------")
            control()
            #print("th4 ",th4*180/math.pi)
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
