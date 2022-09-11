import rospy
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

msg = """
control 2 en linea
"""
e = """
control 2 fuera
"""
k1=rospy.get_param("control2/kp")
hz=rospy.get_param("control2/hz")
delta=1/hz

lx1=rospy.get_param("/control2/xl1")
ly1=rospy.get_param("/control2/yl1")

lx2=rospy.get_param("/control2/xl2")
ly2=rospy.get_param("/control2/yl2")

lx5=rospy.get_param("/control2/xl5")
ly5=rospy.get_param("/control2/yl5")

lx8=rospy.get_param("/control2/xl8")
ly8=rospy.get_param("/control2/yl8")

th1=0
x1h=0
y1h=0

th2=0
x2h=0
y2h=0

th5=0
x5h=0
y5h=0

th8=0
x8h=0
y8h=0

h=0.064
L=0.092
t=0
 
def callback1(data):
    global y1h,x1h,th1
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
    
def callback2(data):
    global y2h,x2h,th2
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
    global y5h,x5h,th5
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
    
def callback8(data):
    global y8h,x8h,th8
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
    
def control():
    global V,w
    r1=-k1*(x2h-x1h)-k1*(x2h-x5h)-k1*(x2h-x8h)
    r2=-k1*(y2h-y1h)-k1*(y2h-y5h)-k1*(y2h-y8h)
    V = r1*math.cos(th2) + r2*math.sin(th2)
    w = -r1*math.sin(th2)/h + r2*math.cos(th2)/h
    if (V>0.4):
        V=0.4
    if (V<-0.4):
        V=-0.4
    if (w>8.69):
        w=8.68
    if (w<-8.69):
        w=-8.68
    
if __name__=="__main__":

    rospy.init_node('Control2',anonymous=True)
    pub = rospy.Publisher('/2/cmd_vel2', Twist, queue_size=5)
    rospy.Subscriber("/2/odom2", Odometry, callback2)
    rospy.Subscriber("/1/odom1", Odometry, callback1)
    rospy.Subscriber("/5/odom5", Odometry, callback5)
    rospy.Subscriber("/8/odom8", Odometry, callback8)
    rate = rospy.Rate(hz) # 10hz
    archivo = open("/home/exon/SMA/src/RMD/scripts/plotear/ah258.txt","w") 
    try:
        print (msg)
        while not rospy.is_shutdown():
            #print("\n--------Control2--------")
            control()
            #print("th2 ",th2*180/math.pi)
            twist = Twist()
            twist.linear.x = V; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
            pub.publish(twist)
            archivo.write(str((t*delta)) + " " + str((x5h)) + " " + str((y5h)) + " " + str((th5)) + " ")
            archivo.write(str((x8h)) + " " + str((y8h)) + " " + str((th8)))
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

