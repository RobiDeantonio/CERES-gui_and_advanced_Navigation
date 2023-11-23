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
#from turtle import delay
from signal import pause
from std_msgs.msg import String
import rospy
import datetime
import time
from geometry_msgs.msg import Twist
from ceres.msg import CeresRC
from math import sqrt,cos,sin
import sys
global tratamiento
tratamiento=False
###############################################################################################
# Functions Definitions:
# adquire: Initialize ROS Node and all topics subscribers/publishers.
def adquire():
    rospy.init_node('decisionMaking', anonymous=True)
    rospy.Subscriber("/ceres/RC", CeresRC, callbackRC)
    rospy.Subscriber("DistanciaGripper", Float32MultiArray, callback, queue_size=1, buff_size=268435456)
    rospy.Subscriber("DistanciaObjeto", Float32MultiArray, callback2, queue_size=1, buff_size=268435456)
    rospy.Subscriber("Jetson_TX1",String,callbackJet, queue_size=10)



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
# callbackJet: receive RC Emergency Stop State to initiate/stop the sequence.
def callbackJet(data):
	global Jet
	Jet=data.data

def callback2(data):
    #rospy.loginfo(data.data)
    global DisO
    DisO=data.data
###############################################################################################
# generatePath: generate a path according to the selection
def STOPX(self):
	global X
	X = 0
	ACTUADORX(X)
def STOPY(self):
	global Y
	Y = 0
	ACTUADORY(Y)

def STOPZ(self):
	global Z
	Z = 0
	ACTUADORZ(Z)



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

global contador

global DisG
global DisO
global X
global Y
global Z
global stop_threads
global Ez

Inicio=0
guardar=0
contador = 1
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



class EvacionDeObstaculos:
    #obstacle es un vector de cordenadas [x,y,z,size], siendo size el tamaño del obstaculo
    obstacle = []

    def __init__(self, name):
        self.name = name

    def addObstacle(self, obstacle):
        self.obstacle.append(obstacle)

    def clearObstacle(self):
        self.obstacle.clear()

    def calcObstacle(self, x, y, z):
        if self.obstacle==None:
            print("There are not obstacles")
        else:
            sumretro=[0, 0, 0]
            for o in self.obstacle:
                try:
                    sq = math.sqrt((o[0] - x) ** 2 + (o[1] - y) ** 2 + (o[2] - z) ** 2)
                    print(sq)
                    retro = (o[3] ** (200 - sq))
                    # if((o[3] * (200 - sq))>0):
                    #     retro = -(o[3] * (100 - sq))
                    # else:
                    #     retro=0
                    
                    sumretro = [np.sign(o[0] - x)*retro ,np.sign(o[1] - y)*retro ,np.sign(o[2] - z)*retro ]+sumretro # este depende del actuador y de la funcion de transferencia
                except:
                    sq = math.sqrt((o[0] - x) ** 2 + (o[1] - y) ** 2 + (o[2] - z) ** 2)
                    print(sq)
                    retro = (2 ** (200 - sq))
                    # if((100 * (200 - sq))>0):
                    #     retro = -(100 * (100 - sq))
                    # else:
                    #     retro=0
                    sumretro = [np.sign(o[0] - x)*retro ,np.sign(o[1] - y)*retro ,np.sign(o[2] - z)*retro ]+sumretro # este depende del actuador y de la funcion de transferencia
                return sumretro



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
        #print(Xek)
        #print(self.Uk)
        #print(self.Yk)
        #print(self.K)
        #self.Uk = -1*(np.matmul(self.K, np.array([[self.Yk]])))
        self.Uk = -1 * (np.matmul(self.K, Xek))
        self.Xek1=Xek
        self.Uk1=self.Uk
        self.Yk1=self.Yk
    def setXek(self,xestimado):
        self.Xek1=xestimado
        self.Xek=xestimado


ControlZ=Controlador(np.array([[1]]),np.array([[-0.001]]),np.array([[-2]]),np.array([[1]])
                     ,np.array([[0]]),0)#Con K=90 funciona muy bien -939.4591
ControlX=Controlador(np.array([[1]]),np.array([[-0.001]]),np.array([[-2]]),np.array([[1]])
                     ,np.array([[0]]),0)#Con K=90 funciona muy bien -939.4591
ControlY=Controlador(np.array([[1]]),np.array([[-0.001]]),np.array([[-2]]),np.array([[1]])
                     ,np.array([[0]]),0)#Con K=90 funciona muy bien -939.4591

###############################################################################################
# Automatico: secuencia automatica para recorrido de actuadores en "U"

def Automatico ():
    global stop_threads, DisG, DisO, DisO2, DisSave, GPSSave, situacion ,Ez, X,Y,Z,tratamiento

    DisO2 = [[-250, -708, 600], [250, -685, 600], [270, -573, 1630], [-301, -604, 1630]]  # Coordenadas de origen , y 3 trayectorias para recorrido en U. coordenadas en pixeles
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

    contador=1
    cont=0
    guardar=0
    Tinicio=time.time()
    TiControl=Tinicio
    TiControlY = Tinicio
    TiControlX = Tinicio
    Angulo=math.radians(35)##42 # Angulo de posicionamiento de la camara primesense en el ceres
    X=1
    Y=1
    Z=1
    ACTUADORX(X)
    ACTUADORZ(Z)
    ACTUADORY(Y)
    ###############################################################################################
    # ciclo de deteccion en U
    d =EvacionDeObstaculos("arbol")
    evadir=[0,0,0]
    while (stop_threads):
        Jet="hola"
        if stop_threads:
            if Jet=="tizon":
                tratamiento = True
                while(tratamiento):
                    print("en tratamiento")
                    X=0
                    Y=0
                    Z=0
                    ACTUADORX(X)
                    ACTUADORZ(Z)
                    ACTUADORY(Y)
                    subprocess.Popen(['ceres/scripts/smc_linux/SmcCmd','--resume', '--speed', '1000'])
                    time.sleep(20)
                    subprocess.Popen(['ceres/scripts/smc_linux/SmcCmd','--stop'])
                    print("continuar recorrido")
                    tratamiento = False

            #(y,z,x)   
            cont=cont+1          
            
            if cont<80:
                DisO2 = [-250, -708, 600]# Coordenada 1 parte superior izquierda(-300, -708, 600)     (y,z,x)  
            elif cont==80:
                X=0
                Y=0
                Z=0
                ACTUADORX(X)
                ACTUADORZ(Z)
                ACTUADORY(Y)

            elif cont<140:                
                DisO2 = [250, -685, 600]# Coordenada 2 parte superior izquierda(250, -685, 600)     
            elif cont ==140:
                X=0
                Y=0
                Z=0
                ACTUADORX(X)
                ACTUADORZ(Z)
                ACTUADORY(Y)            
            elif cont<300:
                 DisO2 = [270, -573, 1400]# Coordenada 1 parte superior izquierda(270, -573, 1630) 
            elif cont ==300:
                X=0
                Y=0
                Z=0
                ACTUADORX(X)
                ACTUADORZ(Z)
                ACTUADORY(Y)  
           
            elif cont<400:
                 DisO2 = [-250, -604, 1400]# Coordenada 1 parte superior izquierda(-301, -604, 1630) 
            elif cont<400:
                rospy.loginfo("[PathGenerator] ALL is finished ")
                
                print("end process XYZ")
                
                X=0
                Y=0
                Z=0
                ACTUADORX(X)
                ACTUADORZ(Z)
                ACTUADORY(Y)
                stop_threads = False                
                


        # if stop_threads:
        #     if Jet=="tizon":
        #         tratamiento = True
        #         while(tratamiento):
        #             print("en tratamiento")
        #             X=0
        #             Y=0
        #             Z=0
        #             ACTUADORX(X)
        #             ACTUADORZ(Z)
        #             ACTUADORY(Y)
        #             time.sleep(20)
        #             print("continuar recorrido")
        #             tratamiento = False
                

        #     #(y,z,x)   
        #     cont=cont+1          
            
        #     if cont<80:
        #         DisO2 = [-250, -708, 600]# Coordenada 1 parte superior izquierda(-300, -708, 600)       
        #     elif cont==80:
        #         X=0
        #         Y=0
        #         Z=0
        #         ACTUADORX(X)
        #         ACTUADORZ(Z)
        #         ACTUADORY(Y)

        #     elif cont<140:                
        #         DisO2 = [250, -685, 600]# Coordenada 2 parte superior izquierda(250, -685, 600)     
        #     elif cont ==140:
        #         X=0
        #         Y=0
        #         Z=0
        #         ACTUADORX(X)
        #         ACTUADORZ(Z)
        #         ACTUADORY(Y)            
        #     elif cont<210:
        #          DisO2 = [270, -573, 1630]# Coordenada 1 parte superior izquierda(270, -573, 1630) 
        #     elif cont ==210:
        #         X=0
        #         Y=0
        #         Z=0
        #         ACTUADORX(X)
        #         ACTUADORZ(Z)
        #         ACTUADORY(Y)  
           
        #     elif cont<280:
        #          DisO2 = [-250, -604, 1630]# Coordenada 1 parte superior izquierda(-301, -604, 1630) 
        #     elif cont<350:
        #         rospy.loginfo("[PathGenerator] ALL is finished ")
                
        #         print("end process XYZ")
                
        #         X=0
        #         Y=0
        #         Z=0
        #         ACTUADORX(X)
        #         ACTUADORZ(Z)
        #         ACTUADORY(Y)
        #         stop_threads = False                
                



    ###############################################################################################
    # calculo actuadores xyz

            

            O = DisO[1]-240
            G = DisG[1]-240
            OKA = XX.getDistance(O , 0.005)
            GKA = YY.getDistance(G , 0.005)
            OKAZ=((OKA*DisO[2]/520))
            GKAZ=((GKA*DisG[2]/520))
            OKA=(OKAZ*math.cos(Angulo))-(DisO[2]*math.sin(Angulo))
            ObjetoZ=OKA
            OKA=DisO2[1]#Se omite ya que se esta usando directamente la distancia en xyz y no se debe encontrar en la camara
            GKA = (GKAZ * math.cos(Angulo)) - (DisG[2] * math.sin(Angulo))
            GriperZ=GKA
            #print(OKA)
            #######################################################################evacion de obstaculos
            
            #d.addObstacle([0, -690, 590, 1.3])
            

            

            if(guardar!=1):
                ControlZ.setXek(np.array([[GKA]]))
            #E = (-10 * 0.66913061 * ((OKA - GKA)))
            TtControl=time.time()
            if (-TiControl+TtControl)>0.02:
                ControlZ.control(GKA, round(OKA/100)*100+200)
                Ez = ControlZ.Uk[0,0]
                TiControl = TtControl
            #print(-TiControl+TtControl)
            print(evadir)
            if -40<(Ez+evadir[1])<40:
                ACTUADORZ(0)  #
            elif (Ez+evadir[1])>3000 or (Ez+evadir[1])<-3000:
                ACTUADORZ(np.sign(Ez+evadir[1])*3000)
            else:
                ACTUADORZ(Ez+evadir[1])  #
            #sheet.write(Contador, 0, OKA)
            #sheet.write(Contador, 1, Ez)
            #E=XX.getDistance(E,0.005)
            #print(E)
            #sheet.write(Contador, 2, GKA)
            #sheet.write(Contador, 9, (time.time()-Tinicio))
            contador=contador+1
            #print((O - G))

            #O es para objeto y G para gripper
            #Actuador Y
            O = DisO[0]-320
            G = DisG[0]-320
            #print(-(O - G))
            #Filtro distancia
            OKA = XXX.getDistance(O, 0.005)
            GKA = YYY.getDistance(G, 0.005)
            OKA = ((OKA * DisO[2] / 520))
            ObjetoY=OKA
            OKA=DisO2[0]


            #OKAZ = ((OKA * DisO[0] / 520)) -----------------
            
            GKA = ((GKA * DisG[2] / 520))
            
            GriperY=GKA
            
            #sheet.write(Contador, 3, OKA)
            #E=(-4 * (OKA - GKA))
            TtControl = time.time()
            if (-TiControlY + TtControl) > 0.02:
                ControlY.control(GKA, round(OKA/100)*100)
                Ey = ControlY.Uk[0, 0]
                TiControlY = TtControl
            if -40 < (Ey+evadir[0]) < 40:
                ACTUADORY(0)  #
            elif (Ey+evadir[0]) > 3000 or (Ey+evadir[0]) < -3000:
                ACTUADORY(np.sign(Ey+evadir[0]) * 3000)
            else:
                ACTUADORY(Ey+evadir[0])  #
            #sheet.write(Contador, 4, Ey)
            #sheet.write(Contador, 5, GKA)

            #Actuador X
            O = DisO[2]
            G = DisG[2]
            #print(-(O - G))
            OKA = XXXX.getDistance(O, 0.005)
            GKA = YYYY.getDistance(G, 0.005)
            OKA=OKA*math.cos(Angulo)+OKAZ*math.sin(Angulo)
            ObjetoX=OKA
            OKA=DisO2[2]
            GKA=GKA*math.cos(Angulo)+GKAZ*math.sin(Angulo)
            GriperX=GKA
####################################################################################################33
            d.addObstacle([ObjetoY, ObjetoZ, ObjetoX, 1.05])
            #print(OKA)
            #print(GKA)
            #sheet.write(Contador, 6, OKA)
            #E=(-10 * 0.74314482*(OKA - GKA))#1
            TtControl = time.time()
           

            
            if (-TiControlX + TtControl) > 0.02:
                ControlX.control(GKA, round(OKA/100)*100)
                Ex = ControlX.Uk[0, 0]
                TiControlX = TtControl
            #sheet.write(Contador, 7, Ex)
            #sheet.write(Contador, 8, GKA)
            if -40 < (Ex+evadir[2]) < 40:
                ACTUADORX(0)  #
            elif (Ex+evadir[2]) > 3000 or (Ex+evadir[2]) < -3000:
                ACTUADORX(np.sign(Ex+evadir[2]) * 3000)
            else:
                ACTUADORX(Ex+evadir[2])  #
            guardar = 1

            evadir=d.calcObstacle(GriperY, GriperZ, GriperX)
            print("evadir y,z,x",evadir)
            print("griper y,z,x ",GriperY,",", GriperZ,",", GriperX)
            print("Obstaculo y,z,x ",ObjetoY,",", ObjetoZ,",", ObjetoX)
            d.clearObstacle()

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
  #delay(5000)
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
  t1 = threading.Thread(Automatico())
  #Automatico()
  t1.start()
  stop_threads = False
    
  rospy.loginfo("[PathGenerator] ALL is finished asdasd")



