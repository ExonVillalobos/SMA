import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
En linea control 1
"""
e = """
EStoy fuera control 1
"""

k0=rospy.get_param("control1/kp")
k1=rospy.get_param("control1/kd")
hz=rospy.get_param("control1/hz")

delta=1/hz

V1=0.01
V2=0
w=0.0

xd1=0.0
yd1=0.0
xd2=0.0
yd2=0.0
dxd=0.0
dyd=0.0

x1c1=0
y1c1=0
x1c2=0
y1c2=0
dx1c=0
dy1c=0

th1=0

L=0.092
t=100
 
def callback1(data):
    global x1c1,y1c1,th1
    #print(data)
    x1c1 = data.pose.pose.position.x
    y1c1 = data.pose.pose.position.y
    
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th1 = 2*math.atan2(c1,c2)
    if th1 <0:
        th1 = 2*math.pi + th1
    th1 = th1 + math.pi/2
    
def control():
    global V1,V2,x1c2,y1c2,dx1c,dy1c,w
    
    dx1c = (x1c1 - x1c2)/delta
    dy1c = (y1c1 - y1c2)/delta
    
    x1c2 = x1c1
    y1c2 = y1c1
    
    r1 = - k0*(x1c1 - xd1) - k1*(dx1c - dxd)
    r2 = - k0*(y1c1 - yd1) - k1*(dy1c - dyd)
    
    V1 = (r1*math.cos(th1) + r2*math.sin(th1))*delta + V2
    w = -r1*math.sin(th1)/V1 + r2*math.cos(th1)/V1
    
    V2=V1
    
def trayectoria():
    global t,xd1,yd1,dxd,dyd,xd2,yd2
    
    time=t*0.005
    
    xd1=1*math.sin(time)/(1 + math.cos(time)*math.cos(time))
    yd1=1*math.sin(time)*math.cos(time)/(1 + math.cos(time)*math.cos(time))
    
    dxd = (xd1 - xd2)/delta
    dyd = (yd1 - yd2)/delta
    
    xd2 = xd1
    yd2 = yd1
    
    t=t+1
    
if __name__=="__main__":

    rospy.init_node('Control1',anonymous=True)
    pub = rospy.Publisher('/1/cmd_vel1', Twist, queue_size=5)
    rospy.Subscriber("/1/odom1", Odometry, callback1)
    rate = rospy.Rate(hz) # 10hz
    
    try:
        print (msg)
        print (delta)
        while not rospy.is_shutdown():
            #trayectoria()
            #print("\n--------Control1--------")
            control()
            #print("th1 ",th1*180/math.pi)
            print("error en X = ", x1c1-xd1)
            print("error en Y = ", y1c1-yd1)
            print("\n--------ERROR--------")
            twist = Twist()
            twist.linear.x = V1; twist.linear.y = 0; twist.linear.z = 0
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

