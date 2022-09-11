import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
En linea control 8
"""
e = """
EStoy fuera control 8
"""

V=0.0
w=0.0
k1=rospy.get_param("control8/kp")
hz=rospy.get_param("control8/hz")
xd=1.0
yd=1.0

lx2=rospy.get_param("/control8/xl2")
ly2=rospy.get_param("/control8/yl2")

lx8=rospy.get_param("/control8/xl8")
ly8=rospy.get_param("/control8/yl8")

x8c=0
y8c=0
th8=0
x8h=0
y8h=0

x2c=0
y2c=0
th2=0
x2h=0
y2h=0

h=0.064
L=0.092
t=0
 
def callback8(data):
    global x8c,y8c,y8h,x8h,th8
    #print(data)
    x8c = data.pose.pose.position.x
    y8c = data.pose.pose.position.y
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th8 = 2*math.atan2(c1,c2)
    if th8 <0:
        th8=2*math.pi + th8
    th8=th8 + math.pi/2
    x8h = x8c + h*math.cos(th8) - lx8
    y8h = y8c + h*math.sin(th8) - ly8
    
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
    
    
def control():
    global V,w
    r1=-k1*(x8h-x2h)
    r2=-k1*(y8h-y2h)
    V = r1*math.cos(th8) + r2*math.sin(th8)
    w = -r1*math.sin(th8)/h + r2*math.cos(th8)/h
    if (V>0.4):
        V=0.4
    if (V<-0.4):
        V=-0.4
    if (w>8.69):
        w=8.68
    if (w<-8.69):
        w=-8.68
    #print("x8h ",x8h)
    #print("y8h ",y8h)
    
if __name__=="__main__":

    rospy.init_node('Control8',anonymous=True)
    pub = rospy.Publisher('/8/cmd_vel8', Twist, queue_size=5)
    rospy.Subscriber("/2/odom2", Odometry, callback2)
    rospy.Subscriber("/8/odom8", Odometry, callback8)
    rate = rospy.Rate(hz) # 10hz
    
    try:
        print (msg)
        while not rospy.is_shutdown():
            #trayectoria()
            #print("\n--------Control8--------")
            control()
            #print("th8 ",th8*180/math.pi)
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
