import rospy
import math
import ast
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
En linea control 5
"""
e = """
EStoy fuera control 5
"""

V=0.0
w=0.0
k1=rospy.get_param("control5/kp")
hz=rospy.get_param("control5/hz")
xd=1.0
yd=1.0

lx=ast.literal_eval(rospy.get_param("/control5/lx"))
ly=ast.literal_eval(rospy.get_param("/control5/ly"))

lx2=lx[1]
ly2=ly[1]

lx5=lx[4]
ly5=ly[4]

x5c=0
y5c=0
th5=0
x5h=0
y5h=0

x2c=0
y2c=0
th2=0
x2h=0
y2h=0

h=0.064
L=0.092
t=0
 
def callback2(data):
    global x2c,y2c,y2h,x2h,th2
    #print(data)
    x2c = data.pose.pose.position.x
    y2c = data.pose.pose.position.y
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th2 = 2*math.atan2(c1,c2)
    if th2 <0:
        th2 = 2*math.pi + th2
    th2 = th2 + math.pi/2
    x2h = x2c + h*math.cos(th2) - lx2
    y2h = y2c + h*math.sin(th2) - ly2
    
def callback5(data):
    global x5c,y5c,y5h,x5h,th5
    #print(data)
    x5c = data.pose.pose.position.x
    y5c = data.pose.pose.position.y
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th5 = 2*math.atan2(c1,c2)
    if th5 <0:
        th5=2*math.pi + th5
    th5=th5 + math.pi/2
    x5h = x5c + h*math.cos(th5) - lx5
    y5h = y5c + h*math.sin(th5) - ly5
    
def control():
    global V,w
    r1=-k1*(x5h-x2h)
    r2=-k1*(y5h-y2h)
    V = r1*math.cos(th5) + r2*math.sin(th5)
    w = -r1*math.sin(th5)/h + r2*math.cos(th5)/h
    if (V>0.4):
        V=0.4
    if (V<-0.4):
        V=-0.4
    if (w>8.69):
        w=8.68
    if (w<-8.69):
        w=-8.68
    #print("x5h ",x5h)
    #print("y5h ",y5h)

    
if __name__=="__main__":

    rospy.init_node('Control5',anonymous=True)
    pub = rospy.Publisher('/5/cmd_vel5', Twist, queue_size=5)
    rospy.Subscriber("/2/odom2", Odometry, callback2)
    rospy.Subscriber("/5/odom5", Odometry, callback5)
    rate = rospy.Rate(hz)
    
    try:
        print (msg)
        while not rospy.is_shutdown():
            #print("\n--------Control5--------")
            control()
            #print("th5 ",th5*180/math.pi)
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
