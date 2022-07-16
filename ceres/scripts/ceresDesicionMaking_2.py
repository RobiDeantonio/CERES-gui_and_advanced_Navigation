#!/usr/bin/env python
# license

##############################################################################################
#  Script:        ceresPathGenerator
#  Version:       4.0
#  Authors:       Adrien Legrand
#  Organization:  Universidad Nacional de Colombia, Universidad Militar Nueva Grenada
#  Proyecto:      Trabajo de Grado
#  Goal:          Generate Speed Command to Control the CERES Agrobot and publish them to /ceres/cmd_vel topic
#  Date:          06-09-2018
###############################################################################################
# Importations:
import rospy
import datetime
import time
from geometry_msgs.msg import Twist
from ceres.msg import CeresRC
from math import sqrt,cos,sin
import sys

###############################################################################################
# Functions Definitions:
# adquire: Initialize ROS Node and all topics subscribers/publishers.
def adquire():
    rospy.init_node('decisionMaking', anonymous=True)
    rospy.Subscriber("/ceres/RC", CeresRC, callbackRC)
    rospy.Subscriber("DistanciaGripper", Float32MultiArray, callback, queue_size=1, buff_size=268435456)
###############################################################################################
# callbackRC: receive RC Emergency Stop State to initiate/stop the sequence.
def callbackRC(data):
	global AU
	if data.emergency<50:
		if AU==True:
			rospy.logwarn("[PathGenerator]Emergency Stop Desactivated !")
		AU=False
	else:
		if AU==False:
			rospy.logwarn("[PathGenerator]Emergency Stop Activated !")
		AU=True
###############################################################################################
# generatePath: generate a path according to the selection
def generatePath(mode, length, Ax, Vx):
	global timeInit,pub

	rate=rospy.Rate(40)	# Frequency of the Reference Publication
	flag=True
	
	if mode==0:
		# Linear Path: Calculate Lenght
		Dx = length
	elif mode==1:
		# Circular Path: Calculate Lenght (Perimeter)
		Dx = length*3.14
	else:
		Dx = 0
	
	Ta = Vx / Ax # Acceleration Time
	Tx = Dx / Vx + Ta # Total Time
	
	while (not rospy.is_shutdown() and flag and not AU):
		# Initialize Message
		msg=Twist()

		# Generate a trapezoidal speed (Acceleration Phase, Constant Speed Phase, Decceleration Phase).
		
		if 2.0*Ta > Tx: # Case Maximal Velocity can't be reached (No Constant Speed Phase)
			Tx = 2*sqrt(Dx / Ax)
			# Update Path Total Time
			if time.time()-timeInit > Tx:
				# If the path is finished, stop the motors.
				flag = False
				msg.linear.x = 0.0
			
			elif time.time()-timeInit < Tx/2:
				# If Acceleration Phase, integrate the acceleration and publish the calculated speed.
				msg.linear.x = Ax * (time.time()-timeInit)
					
			else:
				# If Desacceleration Phase, integrate the acceleration and publish the calculated speed.
				msg.linear.x = Ax * (Tx/2) - Ax * (time.time() - timeInit - Tx/2)
					
					
		else: # Case Maximal Velocity is reached
			if time.time() - timeInit > Tx:
				# If the path is finished, stop the motors.
				flag = False
				msg.linear.x = 0.0
				
			elif time.time() - timeInit < Ta:
				# If Acceleration Phase, integrate the acceleration and publish the calculated speed.
					msg.linear.x=Ax*(time.time()-timeInit)
					
			elif time.time() - timeInit >= Ta and time.time() - timeInit < Tx - Ta:
				# If Constant Speed Phase, publish the Maximum speed.
				msg.linear.x = Ax * Ta
					
			else:
				# If Desacceleration Phase, integrate the acceleration and publish the calculated speed.
				msg.linear.x = Ax * Ta - Ax * (time.time() - timeInit - (Tx - Ta))

		if mode==1:
			# If Circular Path, calculate angular speed to respect the specified radius.
			msg.angular.z = 2.0*msg.linear.x / length
		else:
			# If Linear Path, angular speed is null.
			msg.angular.z = 0.0				

		# Publish the ROS message in the /ceres/cmd_vel topic.
		if not rospy.is_shutdown():
			pub.publish(msg)

		rate.sleep()


###############################################################################################
###############################################################################################
###############################################################################################
# Importations:
import numpy as np
import math

#ROS y Actuadores librerias
import threadingJohann as threading
from threadingJohann import Timer,Thread,Event
import os
import time
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

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
global Ez

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

Ez = 0

###############################################################################################
# publicacion Topicos ROS:
ACTUADORXP = rospy.Publisher("ACTUADORX", Float32, queue_size=1)
ACTUADORYP = rospy.Publisher("ACTUADORY", Float32, queue_size=1)
ACTUADORZP = rospy.Publisher("ACTUADORZ", Float32, queue_size=1)
from os import remove
def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath(".")

    return os.path.join(base_path, relative_path)

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
    global stop_threads, DisG, DisO, DisO2, DisSave, GPSSave, situacion ,Ez

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
            if Contador<2000:
                DisO2 = [500, 117, 1000]
            elif Contador<4000:
                DisO2 = [427, 304, 1883]
            elif Contador < 6000:
                DisO2 = [210, 302, 1873]
            elif Contador < 8000:
                DisO2 = [130, 118, 100]
            elif Contador == 10000:
                Contador=0
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
            print('PRINT OKA',OKA)
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


###############################################################################################
# Envio señales a Arduinos

def ACTUADORX(paquete): #Se inicia el nodo ACTUADORESPY(Actuadores python)
    rate = rospy.Rate(10) #10 Hz
    if not rospy.is_shutdown():
        hello_str = float(paquete)
        #rospy.loginfo(hello_str)
        ACTUADORXP.publish(hello_str)
        rate.sleep()

def ACTUADORY(paquete):
    rate = rospy.Rate(10) #10 Hz
    if not rospy.is_shutdown():
        hello_str = float(paquete)
        #rospy.loginfo(hello_str)
        ACTUADORYP.publish(hello_str)
        rate.sleep()

def ACTUADORZ(paquete):
    rate = rospy.Rate(10) #10 Hz
    if not rospy.is_shutdown():
        hello_str = float(paquete)
        #rospy.loginfo(hello_str)
        ACTUADORZP.publish(hello_str)
        rate.sleep()

###############################################################################################
# Datos recibidos de la camara
def callback(data):
    #rospy.loginfo(data.data)
    global DisG
    DisG=data.data

def UObjeto():
    rate = rospy.Rate(1000)  # 100 Hz
    rate.sleep()

def UGripper():
    rate = rospy.Rate(1000)  # 100 Hz
    rate.sleep()


###############################################################################################
# Main Program

if __name__=='__main__':
  # Read the arguments that were entered when the script was executed.
  # # # If no argument: Linear 5 meters test.
  if len(sys.argv)==1:
    mode=0
    length = 1
	
  elif (len(sys.argv) != 3) or (not (str(sys.argv[1]).isnumeric())) or int(sys.argv[1])<0  or int(sys.argv[1])>2 or float(sys.argv[2])<0:
    raise Exception("Usage error: wrong parameters!")
  else:
    mode = int(sys.argv[1])
    length = 1
    
    #These Values can be adjusted:
    Vx = 0.5 #Maximal Speed
    Ax = 0.25 #Maximal Acceleration	

	#Do not adjust these maximum limits
	# Over-sized values Security Checks
  if(Ax>1):
    Ax=1
    rospy.logwarn("[PathGenerator]Linear Acceleration too high, setted to 1 [m.s-2]")
    
  if(Vx>1):
    Vx=1
    rospy.logwarn("[PathGenerator]Linear Speed too high, setted to 2 [m.s-1]")
  
  adquire()
  pub=rospy.Publisher('/ceres/cmd_vel',Twist,queue_size=25)
  AU=0

  if(AU==0):
    rospy.logwarn("[PathGenerator]Waitting Emergency Stop Activation...")
    while AU==0:
        pass
			
  rospy.logwarn("[PathGenerator]Waitting Emergency Stop Desactivation to initiate the test...")
  if mode==0:
    rospy.logwarn("[PathGenerator]THE ROBOT WILL PERFORM A "+str(length)+" METERS FORWARD MOVEMENT")
  elif mode==1:
    rospy.logwarn("[PathGenerator]THE ROBOT WILL PERFORM A CIRCULAR MOVEMENT WITH A DIAMETER OF "+str(length)+" METERS")

	
  while AU==1:
    pass

  rospy.loginfo("[PathGenerator]Running test...")
  timeInit=time.time()
  
  generatePath(mode, length, Ax, Vx)
  rospy.loginfo("[PathGenerator]Linear mov is finished ")
  
  stop_threads = True
  t1 = threading.Thread(target=Automatico)
  t1.start()
  #stop_threads = False
  rospy.loginfo("[PathGenerator] ALL is finished ")


