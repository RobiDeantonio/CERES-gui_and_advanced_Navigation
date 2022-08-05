#!/home/gidam/catkin_ws/src/CERES-ServoVisual_control/tests/ActuadoresVision-env/bin/python3

#Vision CamaraPrimesense
import numpy as np
#import cv2
#from primesense import openni2#, nite2
#from primesense import _openni2 as c_api
import math


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

global Inicio
Inicio=0
global guardar
guardar=0
global Contador
Contador = 1
#ROS y Actuadores librerias
import threadingJohann as threading
from threadingJohann import Timer,Thread,Event
import os
import sys
import time
from PyQt5 import uic
from PyQt5.QtWidgets import QMainWindow, QApplication
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
global DisG
global DisO
global X
global Y
global Z
global stop_threads
ACTUADORXP = rospy.Publisher("ACTUADORX", Float32, queue_size=1)
ACTUADORYP = rospy.Publisher("ACTUADORY", Float32, queue_size=1)
ACTUADORZP = rospy.Publisher("ACTUADORZ", Float32, queue_size=1)
from os import remove
stop_threads = False
Dis=0
DisG=[0,0,0]
DisO=[0,0,0]
X=0
Y=0
Z=0
rospy.init_node("ACTUADORESPY",anonymous=True)
def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath(".")

    return os.path.join(base_path, relative_path)

class ActuadorGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        ActuadoresGUI = resource_path("/home/gidam/catkin_ws/src/CERES-gui_and_advanced_Navigation/ceres/scripts/ActuadoresGUI.ui")
        uic.loadUi(ActuadoresGUI, self)
        self.Salir.clicked.connect(self.Salido)
        self.RETROCEDERX.clicked.connect(self.RETROCEDERXX)
        self.RETROCEDERY.clicked.connect(self.RETROCEDERYY)
        self.RETROCEDERZ.clicked.connect(self.RETROCEDERZZ)
        self.AVANZARX.clicked.connect(self.AVANZARXX)
        self.AVANZARY.clicked.connect(self.AVANZARYY)
        self.AVANZARZ.clicked.connect(self.AVANZARZZ)
        self.PARADAX.clicked.connect(self.STOPX)
        self.PARADAY.clicked.connect(self.STOPY)
        self.PARADAZ.clicked.connect(self.STOPZ)

    def Salido (self):
        global stop_threads
        # global Tinicio
        # Tinicio = time.time()
        stop_threads=True
        # global DisG
        # global DisO
        # UGripper()
        # UObjeto()
        # O=DisO[1]
        # G=DisG[1]
        # print(O-G)
        # ACTUADORZ(O-G)

    def AVANZARXX(self):
        global X
        X=X+1000
        ACTUADORX(X)
    def AVANZARYY(self):
        global Y
        Y = Y + 1000
        ACTUADORY(Y)
    def AVANZARZZ(self):
        global Z
        Z = Z + 1000
        ACTUADORZ(Z)
    def STOPX(self):
        global stop_threads
        stop_threads = False
        global X
        X = 0
        ACTUADORX(X)

    def STOPY(self):
        global stop_threads
        stop_threads = False
        global Y
        Y = 0
        ACTUADORY(Y)

    def STOPZ(self):
        global stop_threads
        stop_threads = False
        global Z
        Z = 0
        ACTUADORZ(Z)


    def RETROCEDERXX(self):
        global X
        X = X - 1000
        ACTUADORX(X)
    def RETROCEDERYY(self):
        global Y
        Y = Y - 1000
        ACTUADORY(Y)
    def RETROCEDERZZ(self):
        global Z
        Z = Z - 1000
        ACTUADORZ(Z)

#para usar muestras se usa
class perpetualTimer():

   def __init__(self,t,hFunction):
      self.t=t
      self.hFunction = hFunction
      self.thread = Timer(self.t,self.handle_function)

   def handle_function(self):
      self.hFunction()
      self.thread = Timer(self.t,self.handle_function)
      self.thread.start()

   def start(self):
      self.thread.start()

   def cancel(self):
      self.thread.cancel()

class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer = None
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.is_running = False
        self.start()
    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)
    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True
    def stop(self):
        self._timer.cancel()
        self.is_running = False

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

#ControlZ=Controlador(np.array([[1,0.0188],[0,0.8823]]),np.array([[-0.0099],[-0.9687]]),
#                     np.array([[-284.051,14.2561]]),np.array([[1.8823],[41.412]]),np.array([[0],[0]]),0)
ControlZ=Controlador(np.array([[1]]),np.array([[-0.001]]),np.array([[-9]]),np.array([[1]])
                     ,np.array([[0]]),0)#Con K=90 funciona muy bien -939.4591
ControlX=Controlador(np.array([[1]]),np.array([[-0.001]]),np.array([[-9]]),np.array([[1]])
                     ,np.array([[0]]),0)#Con K=90 funciona muy bien -939.4591
ControlY=Controlador(np.array([[1]]),np.array([[-0.001]]),np.array([[-9]]),np.array([[1]])
                     ,np.array([[0]]),0)#Con K=90 funciona muy bien -939.4591
#Controlador
def Automatico ():
    global stop_threads, DisG, DisO, DisO2, DisSave,GPSSave,situacion

    DisO2 = [[479, 117, 1219], [1500, 180, 300], [1500, 180, 300], [1500, 180, 300]]
    DisSave = []
    estado = []
    ##situacion= jetson


    #Workbook = xlrd.open_workbook("Resultados.xls")
    #wb = copy(Workbook)
    #sheet = wb.get_sheet(0)
    contador=1
    guardar=0
    Tinicio=time.time()
    TiControl=Tinicio
    TiControlY = Tinicio
    TiControlX = Tinicio
    Angulo=math.radians(35)##42 en la maquina


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
                DisO2 = [130, 118, 1000]
            elif contador < 10000:
                x=0
                y=0
                z=0
            elif contador == 10000:
                contador=0

            #if (mala):
            #    DisSave.append(DisG)
            #    GPSSave.append(GPSActual)
            #    estado.append(situacion)

#                if situacion == "a":
 #                   print("matar maleza")
  #              elif situacion == "b":
   #                 print("regar planta")
    #            elif situacion == "c":
     #               print("fumigar planta")


            # t = threading.Thread(target=listener)
            # t.start()
            #UGripper()
            #UObjeto()
            #print(DisO)
            #print(DisG)

            O = DisO2[1]-240
            G = DisG[1]-240
            OKA = XX.getDistance(O , 0.005)
            GKA = YY.getDistance(G , 0.005)
            OKAZ=((OKA*DisO[2]/520))
            GKAZ=((GKA*DisG[2]/520))
            OKA=(OKAZ*math.cos(Angulo))-(DisO[2]*math.sin(Angulo))
            #OKA=DisO2[1]
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
            contador=contador+1
            #print((O - G))


            O = DisO2[0]-320
            G = DisG[0]-320
            #print(-(O - G))
            OKA = XXX.getDistance(O, 0.005)
            GKA = YYY.getDistance(G, 0.005)
            #OKA=DisO2[0]

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


            O = DisO2[2]
            G = DisG[2]
            #print(-(O - G))
            OKA = XXXX.getDistance(O, 0.005)
            GKA = YYYY.getDistance(G, 0.005)
            OKA=OKA*math.cos(Angulo)+OKAZ*math.sin(Angulo)
            #OKA=DisO2[2]
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

# def Automatico ():
#     global stop_threads, DisG, DisO,guardar,Contador
#     Workbook = xlrd.open_workbook("Resultados.xls")
#     wb = copy(Workbook)
#     sheet = wb.get_sheet(0)
#     #time.sleep(0.001)
#     if stop_threads:
#         guardar=1
#         # t = threading.Thread(target=listener)
#         # t.start()
#         UGripper()
#         UObjeto()
#         print(DisO)
#         print(DisG)
#         O = DisO[1]
#         G = DisG[1]
#         E=-17*(O - G)
#         sheet.write(Contador, 0, O)
#         sheet.write(Contador, 1, E)
#         sheet.write(Contador, 2, G)
#         sheet.write(Contador, 3, (time.time()-Tinicio))
#         Contador=Contador+1
#         print((O - G))
#         ACTUADORZ(-17*(O - G)) #-17
#         O = DisO[0]
#         G = DisG[0]
#         print(-(O - G))
#         ACTUADORY(-4 * (O - G))#-4
#         O = DisO[2]
#         G = DisG[2]
#         print(-(O - G))
#         ACTUADORX(-1 * (O - G))
#         try:
#             remove('/home/johann/catkin_ws/src/tests/ejemplo.xls')
#             wb.save('ejemplo.xls')
#         except:
#             pass
#     else:
#         try:
#             if guardar==1:
#                 guardar=0
#
#         except:
#             pass

#Enviar al arduino
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

#Para recibir de la camara
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
    #rospy.spin()
def UGripper():
    rate = rospy.Rate(1000)  # 100 Hz
    rate.sleep()
    #rospy.spin()
'''
def talker(paquete):
    pub = rospy.Publisher("ACTUADORZ", Float32, queue_size=10)
    rospy.init_node("talker",anonymous=True)
    rate = rospy.Rate(10) #10 Hz
    if not rospy.is_shutdown():
        hello_str = float(paquete)
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
'''

if __name__ == '__main__':
    app = QApplication(sys.argv)
    GUI = ActuadorGUI()
    rospy.Subscriber("DistanciaObjeto", Float32MultiArray, callback2, queue_size=1, buff_size=268435456)
    rospy.Subscriber("DistanciaGripper", Float32MultiArray, callback, queue_size=1, buff_size=268435456)
    t1 = threading.Thread(target=Automatico)
    #t1 = perpetualTimer(0.001,Automatico)
    t1.start()
    # t1=RepeatedTimer(0.001, Automatico)
    GUI.show()
    sys.exit(app.exec_())

