import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#Mensajes de inicialización de nodo
msg = """
En linea control 1
"""
e = """
EStoy fuera control 1
"""
# Lectura de ganancias para el control
k1=rospy.get_param("control1/kp")
hz=rospy.get_param("control1/hz")
delta=1/hz
# Condiciones iniciales de leyes de control
V=0.0
w=0.0
# Posiciones de referencia
xd=0
xd1=0
yd=0
dxd=0
dyd=0
xini=1.5
yini=1
xfin=2.5
yfin=2
tini=20
tfin=40
#Lectura de desfases entre agentes
lx1=rospy.get_param("/control1/xl1")
ly1=rospy.get_param("/control1/yl1")

lx2=rospy.get_param("/control1/xl2")
ly2=rospy.get_param("/control1/yl2")

lx3=rospy.get_param("/control1/xl3")
ly3=rospy.get_param("/control1/yl3")

lx4=rospy.get_param("/control1/xl4")
ly4=rospy.get_param("/control1/yl4")
#Inicialización de variables para cada agente
x1c=0
y1c=0
th1=0
x1h=0
y1h=0

x2c=0
y2c=0
th2=0
x2h=0
y2h=0

x3c=0
y3c=0
th3=0
x3h=0
y3h=0

x4c=0
y4c=0
th4=0
x4h=0
y4h=0

#Parametros de robot móvil
h=0.064
L=0.092

#Parametros de trayectoria deseada
t=0
 
# Funciones para obtener posicion y ángulo de agentes
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
    
# Funcion de control cinematico
def control():
    global V,w
    r1=dxd-k1*(x1h-xd)-k1*(x1h-x2h)-k1*(x1h-x3h)-k1*(x1h-x4h)
    r2=dyd-k1*(y1h-yd)-k1*(y1h-y2h)-k1*(y1h-y3h)-k1*(y1h-y4h)
    #print(r1)
    #print(r2)
    V = r1*math.cos(th1) + r2*math.sin(th1)
    w = -r1*math.sin(th1)/h + r2*math.cos(th1)/h
    
    if (V>0.4):
        V=0.4
    if (V<-0.4):
        V=-0.4
    if (w>8.69):
        w=8.68
    if (w<-8.69):
        w=-8.68
    #print("x1h ",x1h)
    #print("y1h ",y1h)

# Funcion de trayectoria deseada
def trayectoria():
    global t,xd,yd,dxd,dyd,xd1
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
    yd = (yfin-yini)/(xfin-xini)*(xd - xini) + yini
    dyd= (yfin-yini)/(xfin-xini)*dxd 
    xd1=xd
    print("#####################################")
    print("time= ", time)
    print("error x= ", xd-x1h)
    print("error y= ", yd-y1h)
    
    #xd=1*math.sin(time)/(1 + math.cos(time)*math.cos(time))
    #yd=1*math.sin(time)*math.cos(time)/(1 + math.cos(time)*math.cos(time))
    t=t+1
    
if __name__=="__main__":
    rospy.init_node('Control1',anonymous=True)
    pub = rospy.Publisher('/1/cmd_vel1', Twist, queue_size=5)
    rospy.Subscriber("/1/odom1", Odometry, callback1)
    rospy.Subscriber("/2/odom2", Odometry, callback2)
    rospy.Subscriber("/3/odom3", Odometry, callback3)
    rospy.Subscriber("/4/odom4", Odometry, callback4)
    rate = rospy.Rate(hz) # 10hz
    archivo = open("/home/exon/SMA/src/RMD/scripts/plotear/ah1234.txt","w") 
    # bloque de detección de errores
    try:
        print (msg)
        while not rospy.is_shutdown():
            trayectoria()
            control()
            #print(th1)
            twist = Twist()
            twist.linear.x = V; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
            pub.publish(twist)
            archivo.write(str((t*delta)) + " " + str((x1h)) + " " + str((y1h)) + " " + str((th1)) + " ")
            archivo.write(str((x2h)) + " " + str((y2h)) + " " + str((th2)) + " ")
            archivo.write(str((x3h)) + " " + str((y3h)) + " " + str((th3)) + " ")
            archivo.write(str((x4h)) + " " + str((y4h)) + " " + str((th4)))
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

