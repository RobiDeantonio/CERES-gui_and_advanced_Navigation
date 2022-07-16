#!/home/gidam/catkin_ws/src/CERES-ServoVisual_control/tests/ActuadoresVision-env/bin/python3

#Vision CamaraPrimesense
# license
##############################################################################################
#  Script:        ceresXYZDesicionMaking
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
# Importations:
#ROS y Actuadores librerias
import threadingJohann as threading
from threadingJohann import Timer,Thread,Event
from xlrd import open_workbook
from xlutils.copy import copy
import xlrd
import xlwt
import os
import sys
import time
from PyQt5 import uic
from PyQt5.QtWidgets import QMainWindow, QApplication
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

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
###############################################################################################
# publicacion Topicos ROS:
ACTUADORXP = rospy.Publisher("ACTUADORX", Float32, queue_size=1)
ACTUADORYP = rospy.Publisher("ACTUADORY", Float32, queue_size=1)
ACTUADORZP = rospy.Publisher("ACTUADORZ", Float32, queue_size=1)
from os import remove
rospy.init_node("ACTUADORESPY",anonymous=True)
def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath(".")

    return os.path.join(base_path, relative_path)

###############################################################################################
# Parte del codigo comentariado de la interfaz grafica
# Boton salir activa la variable stop_threads para iniciar el proceso:



#class ActuadorGUI(QMainWindow):
#    def __init__(self):
#        super().__init__()
#        ActuadoresGUI = resource_path("ActuadoresGUI.ui")
#        uic.loadUi(ActuadoresGUI, self)
#      self.Salir.clicked.connect(self.Salido)


 #   def Salido (self):
 #       global stop_threads
 #       stop_threads=True





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

    DisO2 = [[1500, 180, 300], [1500, 180, 300], [1500, 180, 300], [1500, 180, 300]]  # Coordenadas de origen , y 3 trayectorias para recorrido en U. coordenadas en pixeles
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
            '''if contador<2000:
                DisO2 = [1500, 180, 300]
            elif contador<4000:
                DisO2 = [1500, 180, 300]
            elif contador < 6000:
                DisO2 = [1500, 180, 300]
            elif contador < 8000:
                DisO2 = [1500, 180, 300]
            elif contador == 10000:
                contador=0
                '''
            if contador < 4000: ### dirigir el manipulador al punto de origen en 4000 puntos
                DisO2 = [1500, 180, 300]
                Cor=0

            elif Cor==1 or DisG[1] in range(DisO2[1][1]-100, DisO2[1][1]+100) and DisG[2] in range(DisO2[1][2]-100, DisO2[1][2]+100) and DisG[3] in range(DisO2[1][3]-100, DisO2[1][3]+100 ) : ### si Cor es 1   o   el griper se encuentra cerca a las cordenadas origen

                global X, Y, Z
                X = 0
                y = 0
                z = 0
                Cor = 1
                ACTUADORX(X)
                ACTUADORY(Y)
                ACTUADORZ(Z)

                if DisG[1] in range(DisO2[2][1]-100, DisO2[2][1]+100) and DisG[2] in range(DisO2[2][2]-100, DisO2[2][2]+100) and DisG[3] in range(DisO2[2][3]-100, DisO2[2][3]+100) :### si el griper se encuentra cerca a las cordenadas 2
                    Cor=2

            elif Cor==2 or DisG[1] in range(DisO2[2][1]-100, DisO2[2][1]+100) and DisG[2] in range(DisO2[2][2]-100, DisO2[2][2]+100) and DisG[3] in range(DisO2[2][3]-100, DisO2[2][3]+100) :### si Cor es 2   o   el griper se encuentra cerca a las cordenadas 2
                global X, Y, Z
                X = 0
                y = 0
                z = 0
                Cor = 2
                ACTUADORX(X)
                ACTUADORY(Y)
                ACTUADORZ(Z)

                if DisG[1] in range(DisO2[3][1]-100, DisO2[3][1]+100) and DisG[2] in range(DisO2[3][2]-100, DisO2[3][2]+100) and DisG[3] in range(DisO2[3][3]-100, DisO2[3][3]+100) :### si el griper se encuentra cerca a las cordenadas 3
                    Cor=3


            elif Cor==3 or DisG[1] in range(DisO2[3][1]-100, DisO2[3][1]+100) and DisG[2] in range(DisO2[3][2]-100, DisO2[3][2]+100) and DisG[3] in range(DisO2[3][3]-100, DisO2[3][3]+100) :### si Cor es 3   o   el griper se encuentra cerca a las cordenadas 3
                global X, Y, Z
                X = 0
                y = 0
                z = 0
                Cor = 3
                ACTUADORX(X)
                ACTUADORY(Y)
                ACTUADORZ(Z)

                if DisG[1] in range(DisO2[1][1]-100, DisO2[1][1]+100) and DisG[2] in range(DisO2[1][2]-100, DisO2[1][2]+100) and DisG[3] in range(DisO2[1][3]-100, DisO2[1][3]+100):### si el griper se encuentra cerca a las cordenadas 4
                    Cor=4

            elif Cor==4 or DisG[1] in range(DisO2[4][1]-100, DisO2[4][1]+100) and DisG[2] in range(DisO2[4][2]-100, DisO2[4][2]+100) and DisG[3] in range(DisO2[4][3]-100, DisO2[4][3]+100):### si Cor es 4   o   el griper se encuentra cerca a las cordenadas 4
                global X, Y, Z
                X = 0
                y = 0
                z = 0
                Cor = 4
                ACTUADORX(X)
                ACTUADORY(Y)
                ACTUADORZ(Z)

                if DisG[1] in range(DisO2[1][1]-100, DisO2[1][1]+100) and DisG[2] in range(DisO2[1][2]-100, DisO2[1][2]+100) and DisG[3] in range(DisO2[1][3]-100, DisO2[1][3]+100 ) : ### si el griper se encuentra cerca a las cordenadas origen
                    contador =0



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


if __name__ == '__main__':

    rospy.Subscriber("DistanciaObjeto", Float32MultiArray, callback2, queue_size=1, buff_size=268435456)
    rospy.Subscriber("DistanciaGripper", Float32MultiArray, callback, queue_size=1, buff_size=268435456)
    t1 = threading.Thread(target=Automatico)
    t1.start()
    sys.exit(app.exec_())

