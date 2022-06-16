#!/usr/bin/env python
# license
##############################################################################################
#  Script:        ceresDesicionMaking
#  Version:       1.0
#  Authors:       Robinsson Deantonio
#  Organization:  Universidad Nacional de Colombia, Universidad Militar Nueva Grenada
#  Proyecto:      Investigador
#  Goal:          Read a CERES Path from a .path file and publish ROS Pose messages.
#  Date:          06-06-2022
###############################################################################################
# Importations:

import numpy as np
import math
import rospy
import std_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from math import sqrt, cos, sin
from ceres.msg import CeresRC
import threadingJohann as threading
import tf
import utm
import os.path
import sys
import os
import time
###############################################################################################
# Functions Definitions:
# callbackRC: Acquire RC data, to check RC Emergency Stop state.
def callbackRC(data):
	global AU
	if data.emergency<50:
		if AU==True:
			rospy.logwarn("Emergency Stop Desactivated !")
		AU=False
	else:
		if AU==False:
			rospy.logwarn("Emergency Stop Activated !")
		AU=True
###############################################################################################
# callbackGPS: Acquire Robot GPS Initial Position
def callbackGPS(msg):
	global initGPS
	initGPS=[msg.latitude, msg.longitude]
	gpsSub.unregister() #Once first message is received, disconnect the subscriber.
###############################################################################################
# callbackGPS: Acquire Robot Initial UTM Position/Orientation
def callbackImu(msg):
	global initOrientation, initPose, imuSub
	# Convert from Quaternions to Euler Angles
	quaternion = (
	msg.pose.pose.orientation.x,
	msg.pose.pose.orientation.y,
	msg.pose.pose.orientation.z,
	msg.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	
	initPose[0]=msg.pose.pose.position.x
	initPose[1]=msg.pose.pose.position.y
	initOrientation=euler[2]
	imuSub.unregister()	#Once first message is received, disconnect the subscriber.


###############################################################################################
# Functions Definitions:
# SensorKalman: creacion de filtro para controlar actuadores X Y Z.
class SensorKalman:
    def __init__(self):
        self.Q_distance=1
        self.R_measure=1
        self.distance=0
        self.P=0
    def getDistance(self,newDistance,dt):
        self.P+=self.Q_distance*dt
        S=self.P+self.R_measure
        K=self.P/S
        y=newDistance-self.distance
        self.distance+=K*y
        self.P*=(1-K)
        return self.distance
    def setDistance(self,newDistance):
        self.distance=newDistance
    def getQdistance(self):
        return self.Q_distance
    def setQdistance(self,newQ_distance):
        self.Q_distance=newQ_distance
    def getRmeasure(self):
        return self.R_measure
    def setRmeasure(self,newR_measure):
        self.R_measure=newR_measure

XX=SensorKalman()
YY=SensorKalman()#SensorZ
XXX=SensorKalman()#SensorY
YYY=SensorKalman()
XXXX=SensorKalman()#SensorX
YYYY=SensorKalman()

###############################################################################################
# Controlador: Funcion para establecer las velocidades de los actuadores

class Controlador:
    def __init__(self, G, H,K,Ke,Xek,Yk):
        self._timer = None
        self.G= G
        self.H = H
        self.K = K
        self.Ke = Ke
        self.Xek = Xek
        self.Xek1 = Xek
        self.Uk = 0
        self.Uk1 = 0
        self.Yk = Yk
        self.Yk1 = Yk
        self.C=np.array([[1]])
    def control(self,y,e):
        #Esta evaluado sin U, por que da cero, si ubiera abria que ponerla
        self.Yk=y-e
        Xek = np.matmul(self.G, self.Xek1) + (self.H*self.Uk1) + \
              np.matmul(self.Ke, (self.Yk1 - np.matmul(self.C, self.Xek1)))
        print(Xek)
        print(self.Uk)
        print(self.Yk)
        #print(self.K)
        #self.Uk = -1*(np.matmul(self.K, np.array([[self.Yk]])))
        self.Uk = -1 * (np.matmul(self.K, Xek))
        self.Xek1=Xek
        self.Uk1=self.Uk
        self.Yk1=self.Yk
    def setXek(self,xestimado):
        self.Xek1=xestimado
        self.Xek=xestimado

ControlZ=Controlador(np.array([[1]]),np.array([[-0.001]]),np.array([[-15]]),np.array([[1]])
                     ,np.array([[0]]),0)#Con K=90 funciona muy bien -939.4591
ControlX=Controlador(np.array([[1]]),np.array([[-0.001]]),np.array([[-15]]),np.array([[1]])
                     ,np.array([[0]]),0)#Con K=90 funciona muy bien -939.4591
ControlY=Controlador(np.array([[1]]),np.array([[-0.001]]),np.array([[-15]]),np.array([[1]])
                     ,np.array([[0]]),0)#Con K=90 funciona muy bien -939.4591

###############################################################################################
# Envio señales a Arduinos

def ACTUADORX(paquete): #Se inicia el nodo ACTUADORESPY(Actuadores python)
    rate = rospy.Rate(1000) #10 Hz
    if not rospy.is_shutdown():
        hello_str = float(paquete)
        rospy.loginfo(hello_str)
        ACTUADORXP.publish(hello_str)
        rate.sleep()

def ACTUADORY(paquete):
    rate = rospy.Rate(1000) #10 Hz
    if not rospy.is_shutdown():
        hello_str = float(paquete)
        rospy.loginfo(hello_str)
        ACTUADORYP.publish(hello_str)
        rate.sleep()

def ACTUADORZ(paquete):
    rate = rospy.Rate(1000) #10 Hz
    if not rospy.is_shutdown():
        hello_str = float(paquete)
        rospy.loginfo(hello_str)
        ACTUADORZP.publish(hello_str)
        rate.sleep()

###############################################################################################
# Datos recibidos de la camara
def callback(data):
    rospy.loginfo(data.data)
    global DisG
    DisG=data.data

def callback2(data):
    rospy.loginfo(data.data)
    global DisO
    DisO=data.data

def UObjeto():
    rate = rospy.Rate(1000)  # 100 Hz
    rate.sleep()

def UGripper():
    rate = rospy.Rate(1000)  # 100 Hz
    rate.sleep()

###############################################################################################
# Main Program
global gpsSub, initGPS, initOrientation, initPose, imuSub, AU
AU=True

# Check that the PathFile was added as argument when the Python Script was launched
if len(sys.argv)<2:
	raise IOError("Error: The algorithm takes exactly 1 argument!\nceresPathReader.py source.path")
# Check that the PathFile specified exists.
elif not os.path.isfile(sys.argv[1]):
	raise IOError("Error: Specified file doesn't exists!")
# Open the PathFile
else:
	file=open(sys.argv[1], "r")

###############################################################################################
# Decrypt the PathFile
l=file.readline()

# Initialize parameters
origins=[0.0,0.0,0.0] # Origin: Lat, Long, Psi
originsFlag=[False,False,False]
frequency=-1

# Variable that will contains all points data.
points=[] # [[X, Y, Psi, U, W], [...]]

# define if a number is float or not
def isfloat(num):
    try:
        float(num)
        return True
    except ValueError:
        return False

# Decrypt .path file
while(l!=""):
	# Split each line in space-separated elements
	elements=l.replace("\n", "").replace("\r","").split(" ")
	# Decode first keyword:
	if elements[0]=="POSITION":
		# Check if Origin Position is eather relative or GPS-coordinates defined.
		if elements[1]!="RELATIVE":
			originsFlag[0]=True
			originsFlag[1]=True
			origins[0]=float(elements[1])
			origins[1]=float(elements[2])

	elif elements[0]=="ORIENTATION":
		# Check if Orientation is eather relative or defined as an angle from East.
		if elements[1]!="RELATIVE":
			originsFlag[2]=True
			origins[2]=float(elements[1])/180.*3.14
	
	elif elements[0]=="FREQUENCY":
		# Read Path frequency (i.e. The time between each point)
		frequency=float(elements[1])

	elif isfloat(elements[0]):
		# if the first keyword is numeric, read the point data defined in the line.
		points.append([float(elements[0]), float(elements[1]), float(elements[2])/180.*3.14, float(elements[3]), float(elements[4])])	

	l=file.readline()

###############################################################################################
# Variables Globales:
global Inicio

global guardar

global Contador

global DisG
global DisO
global X
global Y
global Z
global stop_threads

Inicio=0
guardar=0
Contador = 1
stop_threads = False  ## variable para empezar el modo automatico
Dis=0
DisG=[0,0,0]
DisO=[0,0,0]
X=0
Y=0
Z=0

# Initialize Robot Initial position variables
initGPS=[-1000.0, -1000.0]		#GPS
initOrientation=-1000.0			#IMU
initPose=[0, 0]                	#IMU

# Initialize ROS Node and Topics 
rospy.init_node('ceresDecisionMaking')
gpsSub = rospy.Subscriber('/advanced_navigation_driver/nav_sat_fix', NavSatFix, callbackGPS)
imuSub = rospy.Subscriber('/advanced_navigation_driver/odom', Odometry, callbackImu)
rospy.Subscriber("DistanciaObjeto", Float32MultiArray, callback2, queue_size=1, buff_size=268435456)
rospy.Subscriber("DistanciaGripper", Float32MultiArray, callback, queue_size=1, buff_size=268435456)

rospy.Subscriber("/ceres/RC", CeresRC, callbackRC)
pub=rospy.Publisher('/ceres/cmd_pose',Odometry,queue_size=25)
ACTUADORXP = rospy.Publisher("ACTUADORX", Float32, queue_size=1)
ACTUADORYP = rospy.Publisher("ACTUADORY", Float32, queue_size=1)
ACTUADORZP = rospy.Publisher("ACTUADORZ", Float32, queue_size=1)


def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath(".")

    return os.path.join(base_path, relative_path)



# Wait for receving IMU data.
rospy.logwarn("[Pose-Controller]Waiting IMU Initial Position...")
while(initPose[0]==0 and initPose[1]==0):
	pass
rospy.loginfo("[Pose-Controller]IMU Initial Position Received")
rospy.loginfo("[Pose-Controller]     - X: "+str(initPose[0]))
rospy.loginfo("[Pose-Controller]     - Y: "+str(initPose[1]))

rospy.loginfo("[Pose-Controller]Waiting IMU Initial Orientation...")
rospy.loginfo("[Pose-Controller]IMU Initial Orientation Received")
rospy.loginfo("[Pose-Controller]     - Orientation: "+str(initOrientation))

# Wait for receving GPS data.
if (originsFlag[0]==True or originsFlag[1]==True):
	rospy.logwarn("[Pose-Controller] Waiting GPS Initial Position...")
	while(initGPS[0]==-1000.0 and initGPS[1]==-1000.0):
		pass
	rospy.loginfo("[Pose-Controller]GPS Initial Position Received")
	rospy.loginfo("[Pose-Controller]     - Latitude: "+str(initGPS[0]))
	rospy.loginfo("[Pose-Controller]     - Longitude: "+str(initGPS[1]))

# Check that frequency has been speficied.
if(frequency<=0):
	raise ValueError("Error: Frenquency wasn't defined!")
timer=rospy.Rate(frequency)

# Initialize Angle Offset
if originsFlag[2]==True:
	# If Absolute Angle, Offset is specified from East Axis.
	dAngle = origins[2]
else:
	# If relative Angle, Offset is specified from Initial Robot X Axis.
	dAngle = initOrientation

# Initialize Position Offset
if (originsFlag[0]==True or originsFlag[1]==True):
	# If Absolute Position, convert Robot Initial GPS and Path Origin GPS Position to UTM (Easting, Northing) Positions
	robotInitPose = utm.from_latlon(initGPS[0], initGPS[1])
	csvInitPose = utm.from_latlon(origins[0], origins[1])

	# Calculate the distance around each axis (Easting, Northing) of the Path Origin from the Robot Initial Position.
	deltaX = csvInitPose[0]-robotInitPose[0] 
	deltaY = csvInitPose[1]-robotInitPose[1] 
	
	# Check if The Path Origin is around the robot initial position (Check the norm)
	distance = sqrt(pow(deltaX,2)+pow(deltaY,2))
	if(distance>10):
		# If The Path Origin is too far, raise error.
		raise ValueError("ERROR: Robot is too far from the CSV Origin Position ("+str(distance)+" m)")
	
	# Compute the Position offset, respectively to the IMU Origin (0,0)
	dX = deltaX + initPose[0]
	dY = deltaY + initPose[1]

else:
	# If Relative Position, Offset is the robot initial position, respectively to the IMU Origin (0,0)
	dX = initPose[0]
	dY = initPose[1]

# Wait for Emergency Stop Activation before initiating Path.
rospy.logwarn("[Pose-Controller] Waitting for Emergency Stop Activation...")
while AU!=True:
	pass

rospy.loginfo("[Pose-Controller]Sequence loaded... "+str(len(points))+" points detected.")

# Wait for Emergency Stop Desactivation to initiate the Path.
rospy.logwarn("[Pose-Controller]Waitting for Emergency Stop Desactivation to EXECUTE the sequence.")
while AU!=False:
	pass


###############################################################################################
# Controlador: Funcion para establecer las velocidades de los actuadores

class Controlador:
    def __init__(self, G, H,K,Ke,Xek,Yk):
        self._timer = None
        self.G= G
        self.H = H
        self.K = K
        self.Ke = Ke
        self.Xek = Xek
        self.Xek1 = Xek
        self.Uk = 0
        self.Uk1 = 0
        self.Yk = Yk
        self.Yk1 = Yk
        self.C=np.array([[1]])
    def control(self,y,e):
        #Esta evaluado sin U, por que da cero, si ubiera abria que ponerla
        self.Yk=y-e
        Xek = np.matmul(self.G, self.Xek1) + (self.H*self.Uk1) + \
              np.matmul(self.Ke, (self.Yk1 - np.matmul(self.C, self.Xek1)))
        print(Xek)
        print(self.Uk)
        print(self.Yk)
        #print(self.K)
        #self.Uk = -1*(np.matmul(self.K, np.array([[self.Yk]])))
        self.Uk = -1 * (np.matmul(self.K, Xek))
        self.Xek1=Xek
        self.Uk1=self.Uk
        self.Yk1=self.Yk
    def setXek(self,xestimado):
        self.Xek1=xestimado
        self.Xek=xestimado

ControlZ=Controlador(np.array([[1]]),np.array([[-0.001]]),np.array([[-15]]),np.array([[1]])
                     ,np.array([[0]]),0)#Con K=90 funciona muy bien -939.4591
ControlX=Controlador(np.array([[1]]),np.array([[-0.001]]),np.array([[-15]]),np.array([[1]])
                     ,np.array([[0]]),0)#Con K=90 funciona muy bien -939.4591
ControlY=Controlador(np.array([[1]]),np.array([[-0.001]]),np.array([[-15]]),np.array([[1]])
                     ,np.array([[0]]),0)#Con K=90 funciona muy bien -939.4591


###############################################################################################
# Automatico: secuencia automatica para recorrido de actuadores en "U"

def Automatico ():
    global stop_threads, DisG, DisO, DisO2, DisSave,GPSSave,situacion

    DisO2 = [[479, 117, 1219], [1500, 180, 300], [1500, 180, 300], [1500, 180, 300]]  # Coordenadas de origen , y 3 trayectorias para recorrido en U. coordenadas en pixeles
    DisSave = []  ##variable para guardar backup de coordenadas
    estado = []   ## variable para guardar estado de la planta por mdeio de la jetson
    ##situacion= jetson   ## señal de deteccion de la jetson

    ###############################################################################################
    # Exportacion posiciones a excel

    #Workbook = xlrd.open_workbook("Resultados.xls")
    #wb = copy(Workbook)
    #sheet = wb.get_sheet(0)
    ###############################################################################################
    # inicializacion variables

    Contador=1
    guardar=0
    Tinicio=time.time()
    TiControl=Tinicio
    TiControlY = Tinicio
    TiControlX = Tinicio
    Angulo=math.radians(35)##42 # Angulo de posicionamiento de la camara primesense en el ceres

    ###############################################################################################
    # ciclo de deteccion en U
    while True:
        #time.sleep(0.001)
        if stop_threads:
            if contador<2000:
                DisO2 = [500, 117, 1000]
            elif contador<4000:
                DisO2 = [427, 304, 1883]
            elif contador < 6000:
                DisO2 = [210, 302, 1873]
            elif contador < 8000:
                DisO2 = [130, 118, 100]
            elif contador == 10000:
                contador=0
     ###############################################################################################
    # deteccion de enfermedad en jetson
            #if (mala):
            #   DisSave.append(DisG)
            #    GPSSave.append(GPSActual)
            #    estado.append(situacion)

            #    if situacion == "a":
            #        print("matar maleza")
            #    elif situacion == "b":
            #        print("regar planta")
            #    elif situacion == "c":
            #        print("fumigar planta")

    ###############################################################################################
    # calculo actuadores xyz

            O = DisO[1]-240
            G = DisG[1]-240
            OKA = XX.getDistance(O , 0.005)
            GKA = YY.getDistance(G , 0.005)
            OKAZ=((OKA*DisO[2]/520))
            GKAZ=((GKA*DisG[2]/520))
            OKA=(OKAZ*math.cos(Angulo))-(DisO[2]*math.sin(Angulo))
            OKA=DisO2[1]
            GKA = (GKAZ * math.cos(Angulo)) - (DisG[2] * math.sin(Angulo))
            print(OKA)
            if(guardar!=1):
                ControlZ.setXek(np.array([[GKA]]))
            #E = (-10 * 0.66913061 * ((OKA - GKA)))
            TtControl=time.time()
            if (-TiControl+TtControl)>0.02:
                ControlZ.control(GKA, round(OKA/100)*100+200)
                Ez = ControlZ.Uk[0,0]
                TiControl = TtControl
            #print(-TiControl+TtControl)
            if -40<(Ez)<40:
                ACTUADORZ(0)  #
            elif Ez>9000 or Ez<-9000:
                ACTUADORZ(np.sign(Ez)*9000)
            else:
                ACTUADORZ(Ez)  #
            #sheet.write(Contador, 0, OKA)
            #sheet.write(Contador, 1, Ez)
            #E=XX.getDistance(E,0.005)
            #print(E)
            #sheet.write(Contador, 2, GKA)
            #sheet.write(Contador, 9, (time.time()-Tinicio))
            Contador=Contador+1
            #print((O - G))


            O = DisO[0]-320
            G = DisG[0]-320
            #print(-(O - G))
            OKA = XXX.getDistance(O, 0.005)
            GKA = YYY.getDistance(G, 0.005)
            OKA=DisO2[0]

            #OKAZ = ((OKA * DisO[0] / 520)) -----------------
            #GKAZ = ((GKA * DisG[0] / 520))

            #sheet.write(Contador, 3, OKA)
            #E=(-4 * (OKA - GKA))
            TtControl = time.time()
            if (-TiControlY + TtControl) > 0.02:
                ControlY.control(GKA, round(OKA/100)*100)
                Ey = ControlY.Uk[0, 0]
                TiControlY = TtControl
            if -40 < (Ey) < 40:
                ACTUADORY(0)  #
            elif Ey > 9000 or Ey < -9000:
                ACTUADORY(np.sign(Ey) * 9000)
            else:
                ACTUADORY(Ey)  #
            #sheet.write(Contador, 4, Ey)
            #sheet.write(Contador, 5, GKA)


            O = DisO[2]
            G = DisG[2]
            #print(-(O - G))
            OKA = XXXX.getDistance(O, 0.005)
            GKA = YYYY.getDistance(G, 0.005)
            OKA=OKA*math.cos(Angulo)+OKAZ*math.sin(Angulo)
            OKA=DisO2[2]
            GKA=GKA*math.cos(Angulo)+GKAZ*math.sin(Angulo)
            print(OKA)
            print(GKA)
            #sheet.write(Contador, 6, OKA)
            #E=(-10 * 0.74314482*(OKA - GKA))#1
            TtControl = time.time()
            if (-TiControlX + TtControl) > 0.02:
                ControlX.control(GKA, round(OKA/100)*100)
                Ex = ControlX.Uk[0, 0]
                TiControlX = TtControl
            #sheet.write(Contador, 7, Ex)
            #sheet.write(Contador, 8, GKA)
            if -40 < (Ex) < 40:
                ACTUADORX(0)  #
            elif Ex > 9000 or Ex < -9000:
                ACTUADORX(np.sign(Ex) * 9000)
            else:
                ACTUADORX(Ex)  #
            guardar = 1
        else:
            try:
                if guardar==1:
                    guardar=0
                    #wb.save('example2.xls')
            except:
                pass



# Publish the Path Points, at the specified Frequency.
for i in range(len(points)):
	pubMsg = Odometry()
	pubMsg.twist.twist.linear.x = points[i][3]
	pubMsg.twist.twist.angular.z = points[i][4]

	# Convert from the Path Referential to the Inertial Referential (UTM)
	if originsFlag[2]==False:
		pubMsg.pose.pose.position.x = points[i][0]*cos(dAngle) - points[i][1]*sin(dAngle) + dX
		pubMsg.pose.pose.position.y = points[i][1]*cos(dAngle) + points[i][0]*sin(dAngle) + dY
	else:
		pubMsg.pose.pose.position.x = points[i][0] + dX
		pubMsg.pose.pose.position.y = points[i][1] + dY

	quaternions = tf.transformations.quaternion_from_euler(0.,0.,points[i][2]+dAngle)
	pubMsg.pose.pose.orientation.x = quaternions[0]
	pubMsg.pose.pose.orientation.y = quaternions[1]
	pubMsg.pose.pose.orientation.z = quaternions[2]
	pubMsg.pose.pose.orientation.w = quaternions[3]

	# Publish the Odom Message to /ceres/cmd_pose topic
	pub.publish(pubMsg)

	timer.sleep()


t1 = threading.Thread(target=Automatico)
t1.start()
 
rospy.loginfo("[Pose-Controller]Sequence Finished, Node shutting down.")
