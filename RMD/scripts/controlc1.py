import rospy
import math
import time
import ast
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

tiempos=ast.literal_eval(rospy.get_param("/control1/tiempos"))
t1=tiempos[0]
t2=tiempos[1]

lx=ast.literal_eval(rospy.get_param("/control1/lx"))
ly=ast.literal_eval(rospy.get_param("/control1/ly"))
lx1=lx[0][0]
ly1=ly[0][0]

lx2=lx[0][1]
ly2=ly[0][1]

lx3=lx[0][2]
ly3=ly[0][2]

lx4=lx[0][3]
ly4=ly[0][3]

delta=1/hz
xd=0
xd1=0
yd=0
yd1=0
dxd=0
dxd1=0
dyd=0
dyd1=0
ddxd=0
ddyd=0
xini=1.5
yini=1
xfin=2.5
yfin=2
tini=20
tfin=40

V1=0
V2=0.01
w=0.0

x1c1=0
dx1c=0
y1c1=0
x1c2=0
y1c2=0
th1=0

x2c1=0
y2c1=0
x2c2=0
y2c2=0

x3c1=0
y3c1=0
x3c2=0
y3c2=0

x4c1=0
y4c1=0
x4c2=0
y4c2=0

L=0.092
t=0
 
def callback1(data):
    global x1c1,y1c1,th1
    x1c1 = data.pose.pose.position.x
    y1c1 = data.pose.pose.position.y
    
    c1  = data.pose.pose.orientation.z
    c2  = data.pose.pose.orientation.w
    th1 = 2*math.atan2(c1,c2)
    if th1 <0:
        th1 = 2*math.pi + th1
    th1 = th1 + math.pi/2
    
def callback2(data):
    global x2c1,y2c1
    #print(data)
    x2c1 = data.pose.pose.position.x
    y2c1 = data.pose.pose.position.y
    
def callback3(data):
    global x3c1,y3c1
    #print(data)
    x3c1 = data.pose.pose.position.x
    y3c1 = data.pose.pose.position.y
    
def callback4(data):
    global x4c1,y4c1
    #print(data)
    x4c1 = data.pose.pose.position.x
    y4c1 = data.pose.pose.position.y

def muestras():
    global x1c2,y1c2,x2c2,y2c2,x3c2,y3c2,x4c2,y4c2  
    # actualización de muestras anteriores 
    x1c2 = x1c1
    y1c2 = y1c1
    
    x2c2 = x2c1
    y2c2 = y2c1
    
    x3c2 = x3c1
    y3c2 = y3c1
    
    x4c2 = x4c1
    y4c2 = y4c1
    
def desfases():
    global lx1,ly1,lx2,ly2,lx3,ly3,lx4,ly4
    if t>t1*hz:
        lx1=lx[1][0]
        ly1=ly[1][0]

        lx2=lx[1][1]
        ly2=ly[1][1]

        lx3=lx[1][2]
        ly3=ly[1][2]

        lx4=lx[1][3]
        ly4=ly[1][3]
    if t>t2*hz:
        lx1=lx[2][0]
        ly1=ly[2][0]

        lx2=lx[2][1]
        ly2=ly[2][1]

        lx3=lx[2][2]
        ly3=ly[2][2]

        lx4=lx[2][3]
        ly4=ly[2][3]
    
def control():
    global V1,V2,w
    # calculo de velocidades 
    dx1c = (x1c1 - x1c2)/delta
    dy1c = (y1c1 - y1c2)/delta
    
    dx2c = (x2c1 - x2c2)/delta
    dy2c = (y2c1 - y2c2)/delta
    
    dx3c = (x3c1 - x3c2)/delta
    dy3c = (y3c1 - y3c2)/delta
    
    dx4c = (x4c1 - x4c2)/delta
    dy4c = (y4c1 - y4c2)/delta
    # calculo de desfases 
    x1=x1c1-lx1
    x2=x2c1-lx2
    x3=x3c1-lx3
    x4=x4c1-lx4
    y1=y1c1-ly1
    y2=y2c1-ly2
    y3=y3c1-ly3
    y4=y4c1-ly4
    # calculo de control auxiliar 
    r11 =ddxd - k0*(x1-xd) - k1*(dx1c-dxd) - k0*(x1 - x2) - k1*(dx1c - dx2c) 
    r1 = r11 - k0*(x1 - x3) - k1*(dx1c - dx3c) - k0*(x1 - x4) - k1*(dx1c - dx4c)
    # calculo de control auxiliar 
    r22 =ddyd - k0*(y1-yd) - k1*(dy1c-dyd) - k0*(y1 - y2) - k1*(dy1c - dy2c) 
    r2 = r22 - k0*(y1 - y3) - k1*(dy1c - dy3c) - k0*(y1 - y4) - k1*(dy1c - dy4c)
    # calculo de velocidades 
    
    V1 = (r1*math.cos(th1) + r2*math.sin(th1))*delta + V2
    w = (-r1*math.sin(th1) + r2*math.cos(th1))/V1
    # actualizar valor anterior de velocidad lineal 
    if (V1>0.4):
        V1=0.4
    if (V1<-0.4):
        V1=-0.4
    if (w>8.69):
        w=8.68
    if (w<-8.69):
        w=-8.68
    V2=V1
    
def trayectoria2():
    global t,xd,yd,dxd,dyd,ddxd,ddyd,xd1,dxd1
    time=t*delta
    tau= (time - tini)/(tfin-tini)
    poli=pow(tau,5)*(126 - 420*tau + 540*pow(tau,2) - 315*pow(tau,3) + 70*pow(tau,4))
    if time < tini:
        xd=xini
        xd1=xini
    if time > tfin:
        xd=xfin
    if (time >= tini)&(time <= tfin):
        xd= xini + poli*(xfin-xini) 
    dxd=(xd-xd1)/delta
    ddxd=(dxd-dxd1)/delta
    yd = (yfin-yini)/(xfin-xini)*(xd - xini) + yini
    dyd= (yfin-yini)/(xfin-xini)*dxd
    ddyd=(yfin-yini)/(xfin-xini)*ddxd
    xd1=xd
    dxd1=dxd
    print("#####################################")
    print("time= ", time)
    #print(dxd)
    #print(dyd)
    t=t+1
    
def trayectoria():
    global t,xd,yd,dxd,dyd,ddxd,ddyd,xd1,dxd1,yd1,dyd1
    time2=t*delta
    time=0.05*t*delta
    xd=4*math.sin(time)/(1 + math.cos(time)*math.cos(time))
    yd=4*math.sin(time)*math.cos(time)/(1 + math.cos(time)*math.cos(time))
    dxd=(xd-xd1)/delta
    ddxd=(dxd-dxd1)/delta
    dyd=(yd-yd1)/delta
    ddyd=(dyd-dyd1)/delta
    xd1=xd
    dxd1=dxd
    yd1=yd
    dyd1=dyd
    print("#####################################")
    print("time= ", time2)
    #print(dxd)
    #print(dyd)
    t=t+1
    
if __name__=="__main__":

    rospy.init_node('Control1',anonymous=True)
    pub = rospy.Publisher('/1/cmd_vel1', Twist, queue_size=5)
    rospy.Subscriber("/1/odom1", Odometry, callback1)
    rospy.Subscriber("/2/odom2", Odometry, callback2)
    rospy.Subscriber("/3/odom3", Odometry, callback3)
    rospy.Subscriber("/4/odom4", Odometry, callback4)
    rate = rospy.Rate(hz) # Hz
    archivo = open("/home/exon/SMA/src/RMD/scripts/plotear/ac1234.txt","w") 
    
    
    try:
        print (msg)
        print (delta)
        while not rospy.is_shutdown():
            trayectoria()
            #print("\n--------Control1--------")
            if t > hz*0.5:
                control()
            desfases()
            muestras()
            #print("th1 ",th1*180/math.pi)
            print("error en X = ", x1c1-lx1-xd)
            print("error en Y = ", y1c1-ly1-yd)
            print("xd = ", xd)
            print("yd = ", yd)
            #print("\n--------ERROR--------")
            twist = Twist()
            twist.linear.x = V1; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
            pub.publish(twist)
            archivo.write(str((t*delta)) + " " + str((x1c1)) + " " + str((y1c1))  + " ")
            archivo.write(str((x2c1)) + " " + str((y2c1)) + " ")
            archivo.write(str((x3c1)) + " " + str((y3c1))  + " ")
            archivo.write(str((x4c1)) + " " + str((y4c1)))
            archivo.write("\n")
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


