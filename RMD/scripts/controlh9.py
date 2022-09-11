import rospy
import math
import ast
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
En linea control 9
"""
e = """
EStoy fuera control 9
"""

V=0.0
w=0.0
k1=rospy.get_param("control9/kp")
hz=rospy.get_param("control9/hz")

lx=ast.literal_eval(rospy.get_param("/control9/lx"))
ly=ast.literal_eval(rospy.get_param("/control9/ly"))

lx3=lx[2]
ly3=ly[2]

lx9=lx[8]
ly9=ly[8]

x9c=0
y9c=0
th9=0
x9h=0
y9h=0

x3c=0
y3c=0
th3=0
x3h=0
y3h=0

h=0.064
L=0.092
t=0
 
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
    
    
def control():
    global V,w
    r1=-k1*(x9h-x3h)
    r2=-k1*(y9h-y3h)
    V = r1*math.cos(th9) + r2*math.sin(th9)
    w = -r1*math.sin(th9)/h + r2*math.cos(th9)/h
    if (V>0.4):
        V=0.4
    if (V<-0.4):
        V=-0.4
    if (w>8.69):
        w=8.68
    if (w<-8.69):
        w=-8.68
    #print("x9h ",x9h)
    #print("y9h ",y9h)
    
if __name__=="__main__":

    rospy.init_node('Control9',anonymous=True)
    pub = rospy.Publisher('/9/cmd_vel9', Twist, queue_size=5)
    rospy.Subscriber("/3/odom3", Odometry, callback3)
    rospy.Subscriber("/9/odom9", Odometry, callback9)
    rate = rospy.Rate(hz) 
    
    try:
        print (msg)
        while not rospy.is_shutdown():
            #print("\n--------Control9--------")
            control()
            #print("th9 ",th9*180/math.pi)
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
