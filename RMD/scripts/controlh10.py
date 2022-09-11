import rospy
import math
import ast
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
control 10 en linea
"""
e = """
control 10 fuera
"""

V=0.0
w=0.0
k1=rospy.get_param("control10/kp")
hz=rospy.get_param("control10/hz")
xd=1.0
yd=1.0

lx=ast.literal_eval(rospy.get_param("/control10/lx"))
ly=ast.literal_eval(rospy.get_param("/control10/ly"))

lx10=lx[9]
ly10=ly[9]

lx4=lx[3]
ly4=ly[3]

x10c=0
y10c=0
th10=0
x10h=0
y10h=0

x4c=0
y4c=0
th4=0
x4h=0
y4h=0

h=0.064
L=0.092
t=0
 
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
    
    
def control():
    global V,w
    r1=-k1*(x10h-x4h)
    r2=-k1*(y10h-y4h)
    V = r1*math.cos(th10) + r2*math.sin(th10)
    w = -r1*math.sin(th10)/h + r2*math.cos(th10)/h
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

    rospy.init_node('Control10',anonymous=True)
    pub = rospy.Publisher('/10/cmd_vel10', Twist, queue_size=5)
    rospy.Subscriber("/4/odom4", Odometry, callback4)
    rospy.Subscriber("/10/odom10", Odometry, callback10)
    rate = rospy.Rate(hz) 
    
    try:
        print (msg)
        while not rospy.is_shutdown():
            #print("\n--------Control10--------")
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
