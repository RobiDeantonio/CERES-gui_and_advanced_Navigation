#!/usr/bin/env python
# license
import sys
import os
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import QObject, QTimer, Qt
from PyQt5.QtGui import QPixmap, QImage, QTransform, QPixmap, QTextCursor
from PyQt5.QtWidgets import QFileDialog
from mainwindow import Ui_MainWindow
import subprocess32 as subprocess
import time
import tf
import rospy
import qOSM 
qOSM.use("PyQt5")
from qOSM.common import QOSM
sys.path.append("/root/catkin_ws/src")
from ceres.msg import CeresArduinoLogging
from ceres.msg import CeresRC
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from rosgraph_msgs.msg import Log
from math import cos, sin, sqrt
from serial.tools import list_ports
import pyqtgraph
import utm

class CeresApplication(QtWidgets.QMainWindow):
	def __init__(self, parent=None):
		super (CeresApplication, self).__init__(parent)
		self.createWidgets()
		self.path=""

	def createWidgets(self):
		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)
  
		self.ui.IMU_connect.clicked.connect(self.connectIMU)
		self.ui.MainArduino_connect.clicked.connect(self.connectMainArduino)
		self.ui.RC_connect.clicked.connect(self.connectRCArduino)
		self.ui.Act_X_connect.clicked.connect(self.connectArduinoAct_X)
		self.ui.Act_Y_connect.clicked.connect(self.connectArduinoAct_Y)
		self.ui.Act_Z_connect.clicked.connect(self.connectArduinoAct_Z)
		self.ui.Start_Sequences_Button.clicked.connect(self.startPathGenerator)
		self.ui.Stop_Sequences_Button.clicked.connect(self.stopPathGenerator)
		self.ui.Run_Camera_Primesense_Button.clicked.connect(self.connectCameraPrime)
		
		self.ui.Water_Pump_Button.clicked.connect(self.runWaterPump)



		self.ui.IMU_autodetect.clicked.connect(self.detectIMU)
		self.ui.RC_autodetect.clicked.connect(self.detectArduinoRC)
		self.ui.MainArduino_autodetect.clicked.connect(self.detectMainArduino)
		self.ui.Act_X_autodetect.clicked.connect(self.detectArduinoAct_X)
		self.ui.Act_Y_autodetect.clicked.connect(self.detectArduinoAct_Y)
		self.ui.Act_Z_autodetect.clicked.connect(self.detectArduinoAct_Z)

		self.ui.Open_Trajectory_File_Button.clicked.connect(self.loadPath)
		
		self.ui.verticalSlider.valueChanged.connect(refreshTrajectory)
		self.ui.Distance_SpinBox.valueChanged.connect(refreshTrajectory)
		self.ui.Diameter_SpinBox.valueChanged.connect(refreshTrajectory)

		## Actuators
		global X
		global Y
		global Z
		X=0
		Y=0
		Z=0

		self.ui.Leave.clicked.connect(self.Salido)
		self.ui.Go_Back_X.clicked.connect(self.RETROCEDERXX)
		self.ui.Go_Back_Y.clicked.connect(self.RETROCEDERYY)
		self.ui.Go_Back_Z.clicked.connect(self.RETROCEDERZZ)
		self.ui.Go_Forward_X.clicked.connect(self.AVANZARXX)
		self.ui.Go_Forward_Y.clicked.connect(self.AVANZARYY)
		self.ui.Go_Forward_Z.clicked.connect(self.AVANZARZZ)
		self.ui.Stop_X.clicked.connect(self.STOPX)
		self.ui.Stop_Y.clicked.connect(self.STOPY)
		self.ui.Stop_Z.clicked.connect(self.STOPZ)


	def loadPath(self):
		self.path = QFileDialog.getOpenFileName(self, 'Open Path File', '',"CERES Path Files (*.path);;All Files (*)")
		refreshTrajectory()
	
	def closeEvent(self, event):
		global procs
		while(len(procs)>0):
			procs[0][1].kill()
			procs.pop(0)
			rospy.logwarn("[GUI] Closed Properly.")
	
	def connectIMU(self):
		global procs
		self.ui.IMU_connect.setText("Disconnect")
		self.ui.IMU_connect.clicked.disconnect(self.connectIMU)
		self.ui.IMU_autodetect.setEnabled(False)
		self.ui.IMU_connect.clicked.connect(self.disconnectIMU)
		imu = subprocess.Popen(['rosrun', 'advanced_navigation_driver', 'advanced_navigation_driver', '_baud:='+self.ui.IMU_vel_serial_list.currentText(), '_port:='+self.ui.IMU_port_list.currentText(), '__name:=ceres_IMU']) 
		procs.append(["imu", imu])
		self.ui.IMU_label_4.setText("<font color='#00AA00'>Connected</font>")
		self.ui.IMU_label_2.setText("<font color='#00AA00'>Connected</font>")

	def disconnectIMU(self):
		global procs
		self.ui.IMU_label_4.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.IMU_label_2.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.IMU_autodetect.setEnabled(True)
		self.ui.IMU_connect.setText("Connect")
		self.ui.IMU_connect.clicked.disconnect(self.disconnectIMU)
		self.ui.IMU_connect.clicked.connect(self.connectIMU)
		for i in procs:
			if i[0]=="imu":
				i[1].kill()
				procs.pop(procs.index(i))
				break

	def connectMainArduino(self):
		global procs
		self.ui.MainArduino_connect.setText("Disconnect")
		self.ui.MainArduino_connect.clicked.disconnect(self.connectMainArduino)
		self.ui.MainArduino_connect.clicked.connect(self.disconnectMainArduino)
		self.ui.MainArduino_autodetect.setEnabled(False)
		mainArduino = subprocess.Popen(['rosrun', 'rosserial_python', 'serial_node.py', '_port:='+self.ui.MainArduino_port_list.currentText(), '__name:=ceres_MainArduino']) 
		procs.append(["mainArduino", mainArduino])
		self.ui.MainArduino_label_4.setText("<font color='#00AA00'>Connected</font>")
		self.ui.MainArduino_label_2.setText("<font color='#00AA00'>Connected</font>")
		
	def disconnectMainArduino(self):
		global procs
		self.ui.MainArduino_label_4.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.MainArduino_label_2.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.MainArduino_autodetect.setEnabled(True)
		self.ui.MainArduino_connect.setText("Connect")
		self.ui.MainArduino_connect.clicked.disconnect(self.disconnectMainArduino)
		self.ui.MainArduino_connect.clicked.connect(self.connectMainArduino)
		for i in procs:
			if i[0]=="mainArduino":
				i[1].kill()
				procs.pop(procs.index(i))
				break
				
	def connectRCArduino(self):
		global procs
		self.ui.RC_connect.setText("Disconnect")
		self.ui.RC_connect.clicked.disconnect(self.connectRCArduino)
		self.ui.RC_connect.clicked.connect(self.disconnectRCArduino)
		self.ui.RC_autodetect.setEnabled(False)
		RCArduino = subprocess.Popen(['rosrun', 'rosserial_python', 'serial_node.py', '_port:='+self.ui.RC_port_list.currentText(), '__name:=ceres_ArduinoRC']) 
		procs.append(["RCArduino", RCArduino])
		self.ui.RC_label_4.setText("<font color='#00AA00'>Connected</font>")
		self.ui.RC_label_2.setText("<font color='#00AA00'>Connected</font>")
		
	def disconnectRCArduino(self):
		global procs
		self.ui.RC_label_4.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.RC_label_2.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.RC_autodetect.setEnabled(True)
		self.ui.RC_connect.setText("Connect")
		self.ui.RC_connect.clicked.disconnect(self.disconnectRCArduino)
		self.ui.RC_connect.clicked.connect(self.connectRCArduino)
		for i in procs:
			if i[0]=="RCArduino":
				i[1].kill()
				procs.pop(procs.index(i))
				break
	

	def connectCameraPrime(self):
		global procs
		self.ui.Run_Camera_Primesense_Button.setText("Stop Camera Primesense")
		self.ui.Run_Camera_Primesense_Button.setStyleSheet('color: red;')
		self.ui.Run_Camera_Primesense_Button.clicked.disconnect(self.connectCameraPrime)
		self.ui.Run_Camera_Primesense_Button.clicked.connect(self.disconnectCameraPrime)
		PrimeSense = subprocess.Popen(['rosrun', 'ceres', 'CamaraPrimesense_cut.py', '__name:=ceres_PrimeSense']) 
		procs.append(["PrimeSense", PrimeSense])
		
	def disconnectCameraPrime(self):
		global procs
		self.ui.Run_Camera_Primesense_Button.setText("Run Camera Primesense")
		self.ui.Run_Camera_Primesense_Button.setStyleSheet('color: green;')
		self.ui.Run_Camera_Primesense_Button.clicked.disconnect(self.disconnectCameraPrime)
		self.ui.Run_Camera_Primesense_Button.clicked.connect(self.connectCameraPrime)
		for i in procs:
			if i[0]=="PrimeSense":
				i[1].kill()
				procs.pop(procs.index(i))
				break

	def runWaterPump(self):
		global procs
		self.ui.Water_Pump_Button.setText("Stop Water Pump")
		self.ui.Water_Pump_Button.setStyleSheet('color: red;')
		self.ui.Water_Pump_Button.clicked.disconnect(self.runWaterPump)
		self.ui.Water_Pump_Button.clicked.connect(self.stopWaterPump)
		subprocess.Popen(['ceres/scripts/smc_linux/SmcCmd','--resume', '--speed', '1000']) 
		self.ui.WaterPump_label.setText("<font color='#00AA00'>ON</font>")
		
	def stopWaterPump(self):
		global procs
		self.ui.WaterPump_label.setText("<font color=''#FF0000'>OFF</font>")
		self.ui.Water_Pump_Button.setText("Run Water Pump")
		self.ui.Water_Pump_Button.setStyleSheet('color: green;')
		self.ui.Water_Pump_Button.clicked.disconnect(self.stopWaterPump)
		self.ui.Water_Pump_Button.clicked.connect(self.runWaterPump)
		subprocess.Popen(['ceres/scripts/smc_linux/SmcCmd','--stop'])

	def connectArduinoAct_X(self):
		global procs
		self.ui.Act_X_connect.setText("Disconnect")
		self.ui.Act_X_connect.clicked.disconnect(self.connectArduinoAct_X)
		self.ui.Act_X_connect.clicked.connect(self.disconnectArduinoAct_X)
		self.ui.Act_X_autodetect.setEnabled(False)
		ArduinoAct_X = subprocess.Popen(['rosrun', 'rosserial_python', 'serial_node.py', '_port:='+self.ui.Act_X_port_list.currentText(), '__name:=ceres_ArduinoAct_X']) 
		procs.append(["ArduinoAct_X", ArduinoAct_X])
		self.ui.Act_X_label_4.setText("<font color='#00AA00'>Connected</font>")
		self.ui.Act_X_label_2.setText("<font color='#00AA00'>Connected</font>")
		
	def disconnectArduinoAct_X(self):
		global procs
		self.ui.Act_X_label_4.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.Act_X_label_2.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.Act_X_autodetect.setEnabled(True)
		self.ui.Act_X_connect.setText("Connect")
		self.ui.Act_X_connect.clicked.disconnect(self.disconnectArduinoAct_X)
		self.ui.Act_X_connect.clicked.connect(self.connectArduinoAct_X)
		for i in procs:
			if i[0]=="ArduinoAct_X":
				i[1].kill()
				procs.pop(procs.index(i))
				break			
	
	def connectArduinoAct_Y(self):
		global procs
		self.ui.Act_Y_connect.setText("Disconnect")
		self.ui.Act_Y_connect.clicked.disconnect(self.connectArduinoAct_Y)
		self.ui.Act_Y_connect.clicked.connect(self.disconnectArduinoAct_Y)
		self.ui.Act_Y_autodetect.setEnabled(False)
		ArduinoAct_Y = subprocess.Popen(['rosrun', 'rosserial_python', 'serial_node.py', '_port:='+self.ui.Act_Y_port_list.currentText(), '__name:=ceres_ArduinoAct_Y']) 
		procs.append(["ArduinoAct_Y", ArduinoAct_Y])
		self.ui.Act_Y_label_4.setText("<font color='#00AA00'>Connected</font>")
		self.ui.Act_Y_label_2.setText("<font color='#00AA00'>Connected</font>")
		
	def disconnectArduinoAct_Y(self):
		global procs
		self.ui.Act_Y_label_4.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.Act_Y_label_2.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.Act_Y_autodetect.setEnabled(True)
		self.ui.Act_Y_connect.setText("Connect")
		self.ui.Act_Y_connect.clicked.disconnect(self.disconnectArduinoAct_Y)
		self.ui.Act_Y_connect.clicked.connect(self.connectArduinoAct_Y)
		for i in procs:
			if i[0]=="ArduinoAct_Y":
				i[1].kill()
				procs.pop(procs.index(i))
				break
	
	def connectArduinoAct_Z(self):
		global procs
		self.ui.Act_Z_connect.setText("Disconnect")
		self.ui.Act_Z_connect.clicked.disconnect(self.connectArduinoAct_Z)
		self.ui.Act_Z_connect.clicked.connect(self.disconnectArduinoAct_Z)
		self.ui.Act_Z_autodetect.setEnabled(False)
		ArduinoAct_Z = subprocess.Popen(['rosrun', 'rosserial_python', 'serial_node.py', '_port:='+self.ui.Act_Z_port_list.currentText(), '__name:=ceres_ArduinoAct_Z']) 
		procs.append(["ArduinoAct_Z", ArduinoAct_Z])
		self.ui.Act_Z_label_4.setText("<font color='#00AA00'>Connected</font>")
		self.ui.Act_Z_label_2.setText("<font color='#00AA00'>Connected</font>")
		
	def disconnectArduinoAct_Z(self):
		global procs
		self.ui.Act_Z_label_4.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.Act_Z_label_2.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.Act_Z_autodetect.setEnabled(True)
		self.ui.Act_Z_connect.setText("Connect")
		self.ui.Act_Z_connect.clicked.disconnect(self.disconnectArduinoAct_Z)
		self.ui.Act_Z_connect.clicked.connect(self.connectArduinoAct_Z)
		for i in procs:
			if i[0]=="ArduinoAct_Z":
				i[1].kill()
				procs.pop(procs.index(i))
				break

### Actuators Move
	def Salido (self):
		print(1)
		sys.exit(app.exec_())
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

	def RETROCEDERXX(self):
		global X
		X = X - 100
		ACTUADORX(X)
	def RETROCEDERYY(self):
		global Y
		Y = Y - 1000
		ACTUADORY(Y)
	def RETROCEDERZZ(self):
		global Z
		Z = Z - 1000
		ACTUADORZ(Z)


	def startPathGenerator(self):
		global myapp, procs
		flag = True
		for i in procs:
			if i[0]=="PathGenerator" or i[0]=="bagLogger" or i[0]=="csvLogger" or i[0]=="poseController" or i[0]=="PathReader":
				flag=False
				break
		if flag:
			if myapp.ui.CSV_Slider.value()==1:
				csvLogger= subprocess.Popen(['rosrun', 'ceres', 'ceresLog.py', '__name:=ceres_Logger']) 
				procs.append(["csvLogger", csvLogger])
			
			if myapp.ui.BAG_Slider.value()==1:
				bagLogger= subprocess.Popen(['rosbag', 'record', '-a', '__name:=ceres_bag']) 
				procs.append(["bagLogger", bagLogger])
		
			if myapp.ui.verticalSlider.value()==1:				
				pathGenerator = subprocess.Popen(['rosrun', 'ceres', 'ceresPathGenerator.py', '1', str(myapp.ui.Diameter_SpinBox.value())]) 
				procs.append(["PathGenerator", pathGenerator])
			elif myapp.ui.verticalSlider.value()==2:
				pathGenerator = subprocess.Popen(['rosrun', 'ceres', 'ceresPathGenerator.py', '0', str(myapp.ui.Distance_SpinBox.value())]) 
				procs.append(["PathGenerator", pathGenerator])
			elif myapp.ui.verticalSlider.value()==3:
				poseController = subprocess.Popen(['rosrun', 'ceres', 'ceresController.py', '__name:=ceres_PoseController']) 
				procs.append(["poseController", poseController])
				pathReader = subprocess.Popen(['rosrun', 'ceres', 'ceresPathReader.py', myapp.path[0], '__name:=ceres_PathReader']) 
				procs.append(["PathReader", pathReader])
			elif myapp.ui.verticalSlider.value()==4:
				#pass
				#poseController = subprocess.Popen(['rosrun', 'ceres', 'ceresController.py', '__name:=ceres_PoseController']) 
				#procs.append(["poseController", poseController])
				DesiciMaking = subprocess.Popen(['rosrun', 'ceres', 'ceresDesicionMaking_2.py', '1', str(myapp.ui.Diameter_SpinBox.value())]) 
				procs.append(["DesiciMaking", DesiciMaking])
		else:
			rospy.logerr("[GUI] A test sequence is already started!")
			
	def stopPathGenerator(self):
		global myapp, procs
		flag=True
		while(flag):
			flag=False
			for i in procs:
				if i[0]=="poseController":
					i[1].kill()
					procs.pop(procs.index(i))
					rospy.logwarn("[GUI] Pose Controller Closed.")
					flag=True
					break
				if i[0]=="PathReader":
					i[1].kill()
					procs.pop(procs.index(i))
					rospy.logwarn("[GUI] Test aborded.")
					flag=True
					break
				if i[0]=="PathGenerator":
					i[1].kill()
					procs.pop(procs.index(i))
					rospy.logwarn("[GUI] Test aborded.")
					flag=True
					break
				if i[0]=="csvLogger":
					i[1].kill()
					procs.pop(procs.index(i))
					rospy.logwarn("[GUI]  CSV data logging finished.")
					flag=True
					break
				if i[0]=="bagLogger":
					temp = subprocess.Popen(['rosnode', 'kill', '/ceres_bag'])
					procs.pop(procs.index(i))
					rospy.logwarn("[GUI]  BAG data logging finished.")
					flag=True
					break
					
	def detectIMU(self):
		global myapp
		
		myapp.ui.IMU_port_list.clear()
		ports = list_ports.comports()
		n=-1
		for i in range(len(ports)):
			if ports[i].description == "USB-RS232 Cable":
				n=i
				break
		L = []
		if n>-1:
			L.append(ports[n].device)
		for i in range(len(ports)):
			if i!=n:
				L.append(ports[i].device)
		myapp.ui.IMU_port_list.addItems(L)

	def detectArduinoRC(self):
		global myapp
		
		myapp.ui.RC_port_list.clear()
		ports = list_ports.comports()
		n=-1
		for i in range(len(ports)):
			if ports[i].description == "USB2.0-Serial":
				n=i
				break
		#L=QStringList()
		L = []
		if n>-1:
			L.append(ports[n].device)
		for i in range(len(ports)):
			if i!=n:
				L.append(ports[i].device)
		myapp.ui.RC_port_list.addItems(L)
		
	def detectMainArduino(self):
		global myapp
		
		myapp.ui.MainArduino_port_list.clear()
		ports = list_ports.comports()
		n=-1
		for i in range(len(ports)):
			if ports[i].description == "Arduino Mega":
				n=i
				break
		#L=QStringList()
		L = []
		if n>-1:
			L.append(ports[n].device)
		for i in range(len(ports)):
			if i!=n:
				L.append(ports[i].device)
		myapp.ui.MainArduino_port_list.addItems(L)


	def detectArduinoAct_X(self):
		global myapp
		
		myapp.ui.Act_X_port_list.clear()
		ports = list_ports.comports()
		n=-1
		for i in range(len(ports)):
			if ports[i].description == "Arduino UNO R3":
				n=i
				break
		#L=QStringList()
		L = []
		if n>-1:
			L.append(ports[n].device)
		for i in range(len(ports)):
			if i!=n:
				L.append(ports[i].device)
		myapp.ui.Act_X_port_list.addItems(L)


	def detectArduinoAct_Y(self):
		global myapp
		
		myapp.ui.Act_Y_port_list.clear()
		ports = list_ports.comports()
		n=-1
		for i in range(len(ports)):
			if ports[i].description == "Arduino UNO R3":
				n=i
				break
		#L=QStringList()
		L = []
		if n>-1:
			L.append(ports[n].device)
		for i in range(len(ports)):
			if i!=n:
				L.append(ports[i].device)
		myapp.ui.Act_Y_port_list.addItems(L)

	def detectArduinoAct_Z(self):
		global myapp
		
		myapp.ui.Act_Z_port_list.clear()
		ports = list_ports.comports()
		n=-1
		for i in range(len(ports)):
			if ports[i].description == "Arduino UNO R3":
				n=i
				break
		#L=QStringList()
		L = []
		if n>-1:
			L.append(ports[n].device)
		for i in range(len(ports)):
			if i!=n:
				L.append(ports[i].device)
		myapp.ui.Act_Z_port_list.addItems(L)

def refreshProcs():
	global procs, myapp
	updateLog()
	for i in procs:
		if i[1].poll() is not None:
			if  i[0] == "imu":
				procs.pop(procs.index(i))
				myapp.ui.IMU_label_4.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.IMU_label_2.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.IMU_connect.setText("Connect")
				myapp.ui.IMU_connect.clicked.disconnect(myapp.disconnectIMU)
				myapp.ui.IMU_connect.clicked.connect(myapp.connectIMU)
				myapp.ui.IMU_autodetect.setEnabled(True)
				break
				
			elif  i[0] == "mainArduino":
				procs.pop(procs.index(i))
				myapp.ui.MainArduino_label_4.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.MainArduino_label_2.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.MainArduino_connect.setText("Connect")
				myapp.ui.MainArduino_connect.clicked.disconnect(myapp.disconnectMainArduino)
				myapp.ui.MainArduino_connect.clicked.connect(myapp.connectMainArduino)
				myapp.ui.MainArduino_autodetect.setEnabled(True)
				break
				
			elif  i[0] == "RCArduino":
				procs.pop(procs.index(i))
				myapp.ui.RC_label_4.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.RC_label_2.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.RC_connect.setText("Connect")
				myapp.ui.RC_connect.clicked.disconnect(myapp.disconnectRCArduino)
				myapp.ui.RC_connect.clicked.connect(myapp.connectRCArduino)
				myapp.ui.RC_autodetect.setEnabled(True)
				break

			elif  i[0] == "PrimeSense":
				procs.pop(procs.index(i))
				myapp.ui.Run_Camera_Primesense_Button.setText("Run Camera Primesense")
				myapp.ui.Run_Camera_Primesense_Button.setStyleSheet('color: green;')
				myapp.ui.Run_Camera_Primesense_Button.clicked.disconnect(myapp.disconnectCameraPrime)
				myapp.ui.Run_Camera_Primesense_Button.clicked.connect(myapp.connectCameraPrime)
				break

			elif  i[0] == "ArduinoAct_X":
				procs.pop(procs.index(i))
				myapp.ui.Act_X_label_4.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.Act_X_label_2.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.Act_X_connect.setText("Connect")
				myapp.ui.Act_X_connect.clicked.disconnect(myapp.disconnectArduinoAct_X)
				myapp.ui.Act_X_connect.clicked.connect(myapp.connectArduinoAct_X)
				myapp.ui.Act_X_autodetect.setEnabled(True)
				break

			elif  i[0] == "ArduinoAct_Y":
				procs.pop(procs.index(i))
				myapp.ui.Act_Y_label_4.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.Act_Y_label_2.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.Act_Y_connect.setText("Connect")
				myapp.ui.Act_Y_connect.clicked.disconnect(myapp.disconnectArduinoAct_Y)
				myapp.ui.Act_Y_connect.clicked.connect(myapp.connectArduinoAct_Y)
				myapp.ui.Act_Y_autodetect.setEnabled(True)
				break

			elif  i[0] == "ArduinoAct_Z":
				procs.pop(procs.index(i))
				myapp.ui.Act_Z_label_4.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.Act_Z_label_2.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.Act_Z_connect.setText("Connect")
				myapp.ui.Act_Z_connect.clicked.disconnect(myapp.disconnectArduinoAct_Z)
				myapp.ui.Act_Z_connect.clicked.connect(myapp.connectArduinoAct_Z)
				myapp.ui.Act_Z_autodetect.setEnabled(True)
				break
				
			elif  i[0] == "PathGenerator" or i[0]=="PathReader":
				procs.pop(procs.index(i))
				flag=True
				while(flag):
					flag=False
					for j in procs:
						if j[0]=="poseController":
							j[1].kill()
							procs.pop(procs.index(j))
							rospy.logwarn("[GUI] Pose Controller Closed.")
							flag=True
							break
						if j[0]=="csvLogger":
							j[1].kill()
							procs.pop(procs.index(j))
							rospy.logwarn("[GUI]  CSV data logging finished.")
							flag=True
							break
						if j[0]=="bagLogger":
							temp = subprocess.Popen(['rosnode', 'kill', '/ceres_bag'])
							procs.pop(procs.index(j))
							rospy.logwarn("[GUI]  BAG data logging finished.")
							flag=True
							break
				break				

# callbackRC: called each time a message is received from the RC Arduino.
def callbackRC(data):
	global var
	var[10][3]=data.CH1
	var[11][3]=data.CH2
	var[12][3]=data.emergency
	var[13][3]=data.AUX

# callbackTwist: called each time a message is received from the cmd_vel topic.
def callbackTwist(data):
	global var
	var[8][3]=data.linear.x
	var[9][3]=data.angular.z

# callbackArduino: called each time a message is received from the Arduino.
def callbackArduino(data):
	global var
	var[6][3]=data.Ul
	var[7][3]=data.Ur

 #callBackIMU: called each time the IMU return an Odometry message.
def callbackIMU(data):
	global var
	quaternion = (
	data.pose.pose.orientation.x,
	data.pose.pose.orientation.y,
	data.pose.pose.orientation.z,
	data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	var[3][3] = euler[2] 
	var[18][3] = euler[0] 
	var[19][3] = euler[1] 
	
	var[1][3]=data.pose.pose.position.x
	var[2][3]=data.pose.pose.position.y

	var[4][3]=data.twist.twist.linear.x
	var[17][3]=data.twist.twist.linear.y
	var[5][3]=data.twist.twist.angular.z


 #callBackGPS: called each time the IMU return a GPS Message.
def callbackGPS(data):
	global var
	var[14][3]=data.latitude
	var[15][3]=data.longitude
	var[16][3]=data.altitude

 #callBackLog: called each time ROS returns a message.
def callbackLog(data):
	global logs
	logs.append([data.msg, data.level])

def updateLog():
	global myapp, logs
	while(len(logs)>0):
		if logs[0][1] == 1:
			myapp.ui.Terminal_1.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#55aa00;\" >"+logs[0][0] +"</span><br/><br/>")
			myapp.ui.Terminal_2.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#55aa00;\" >"+logs[0][0] +"</span><br/><br/>")
			myapp.ui.statusBar.showMessage("MainArduino:  "+logs[0][0], 5000)

		elif logs[0][1]  == 2:
			myapp.ui.Terminal_1.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#0000FF;\" >"+logs[0][0]+"</span><br/><br/>")
			myapp.ui.Terminal_2.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#0000FF;\" >"+logs[0][0]+"</span><br/><br/>")
			myapp.ui.statusBar.showMessage("MainArduino:  "+logs[0][0], 5000)
		
		elif logs[0][1]  == 4:
			myapp.ui.Terminal_1.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#ffaa00;\" >"+logs[0][0]+"</span><br/><br/>")
			myapp.ui.Terminal_2.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#ffaa00;\" >"+logs[0][0]+"</span><br/><br/>")
			myapp.ui.statusBar.showMessage("MainArduino:  "+logs[0][0], 5000)

		elif logs[0][1]  == 8:
			myapp.ui.Terminal_1.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#FF0000;\" >"+logs[0][0]+"</span><br/><br/>")
			myapp.ui.Terminal_2.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#FF0000;\" >"+logs[0][0]+"</span><br/><br/>")
			myapp.ui.statusBar.showMessage("MainArduino:  "+logs[0][0], 5000)
			
		elif logs[0][1]  == 16:
			myapp.ui.Terminal_1.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#aa0000;\" >"+logs[0][0]+"</span><br/><br/>")
			myapp.ui.Terminal_2.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#aa0000;\" >"+logs[0][0]+"</span><br/><br/>")
			myapp.ui.statusBar.showMessage("MainArduino:  "+logs[0][0], 5000)
		logs=logs[1:]
		myapp.ui.Terminal_1.moveCursor(QTextCursor.End)
		myapp.ui.Terminal_2.moveCursor(QTextCursor.End)
		
# adquire: Initialize all the subscribers/publishers.
def adquire():
	rospy.init_node('Ceres_GUI', anonymous=True)
	rospy.Subscriber("/ceres/arduLog", CeresArduinoLogging, callbackArduino)
	rospy.Subscriber("/ceres/RC", CeresRC, callbackRC)
	rospy.Subscriber("/ceres/cmd_vel", Twist, callbackTwist)
	rospy.Subscriber("/advanced_navigation_driver/odom", Odometry, callbackIMU)
	rospy.Subscriber("/advanced_navigation_driver/nav_sat_fix", NavSatFix, callbackGPS)
	rospy.Subscriber("/rosout", Log, callbackLog)

def refreshGraph():
	global myapp, var, eastingList, northingList, headingList, velocityXList, velocityYList, angularVelocityList, timeList, graphRate
	
	timeList.pop(0)
	timeList.append(timeList[-1]+1.0/graphRate)
	eastingList.pop(0)
	eastingList.append(var[1][3])
	myapp.ui.Easting_Position_view.clear()
	myapp.ui.Easting_Position_view.plot(timeList,eastingList, pen=pyqtgraph.mkPen('k', width=3))
	myapp.ui.Easting_Position_view.setXRange(timeList[0], timeList[-1], padding=0)
	myapp.ui.Easting_Position_view.showGrid(x=True, y=True)

	northingList.pop(0)
	northingList.append(var[2][3])
	myapp.ui.Northing_Position_view.clear()
	myapp.ui.Northing_Position_view.plot(timeList,northingList, pen=pyqtgraph.mkPen('k', width=3))
	myapp.ui.Northing_Position_view.setXRange(timeList[0], timeList[-1], padding=0)
	myapp.ui.Northing_Position_view.showGrid(x=True, y=True)

	headingList.pop(0)
	headingList.append(360*var[3][3]/(2*3.14))
	myapp.ui.Heading_Orientation_view.clear()
	myapp.ui.Heading_Orientation_view.plot(timeList,headingList, pen=pyqtgraph.mkPen('k', width=3))
	myapp.ui.Heading_Orientation_view.setXRange(timeList[0], timeList[-1], padding=0)
	myapp.ui.Heading_Orientation_view.showGrid(x=True, y=True)

	velocityXList.pop(0)
	velocityXList.append(var[4][3])
	myapp.ui.Robot_X_Velocity_view.clear()
	myapp.ui.Robot_X_Velocity_view.plot(timeList,velocityXList, pen=pyqtgraph.mkPen('k', width=3))
	myapp.ui.Robot_X_Velocity_view.setXRange(timeList[0], timeList[-1], padding=0)
	myapp.ui.Robot_X_Velocity_view.showGrid(x=True, y=True)

	velocityYList.pop(0)
	velocityYList.append(var[17][3])
	myapp.ui.Robot_Y_Velocity_view.clear()
	myapp.ui.Robot_Y_Velocity_view.plot(timeList,velocityYList, pen=pyqtgraph.mkPen('k', width=3))
	myapp.ui.Robot_Y_Velocity_view.setXRange(timeList[0], timeList[-1], padding=0)
	myapp.ui.Robot_Y_Velocity_view.showGrid(x=True, y=True)

	angularVelocityList.pop(0)
	angularVelocityList.append(var[5][3])
	myapp.ui.Robot_Ang_Velocity_view.clear()
	myapp.ui.Robot_Ang_Velocity_view.plot(timeList,angularVelocityList, pen=pyqtgraph.mkPen('k', width=3))
	myapp.ui.Robot_Ang_Velocity_view.setXRange(timeList[0], timeList[-1], padding=0)
	myapp.ui.Robot_Ang_Velocity_view.showGrid(x=True, y=True)
	

def refreshData():
	global myapp, var, i
	myapp.ui.Latitude_label_.setText(str(round(var[14][3], 8))) #Refresh Latitude
	myapp.ui.Longitude_label_.setText(str(round(var[15][3], 8))) #Refresh Longitude
	myapp.ui.Altitude_label_.setText(str(round(var[16][3], 0))) #Refresh Altitude
	myapp.ui.Easting_label_.setText(str(round(var[1][3], 2))) #Refresh Easting
	myapp.ui.Northing_label_.setText(str(round(var[2][3], 2))) #Refresh Northing

	myapp.ui.progressBarThrottle.setValue(var[10][3]) #Refresh CH1
	myapp.ui.progressBarYaw.setValue(var[11][3]) #Refresh CH2
	myapp.ui.progressBarAUX.setValue(var[13][3]) #Refresh AUX
	myapp.ui.progressBarAU.setValue(var[12][3]) #Refresh AU
	
	myapp.ui.left_driver_vol_indicator.setValue(int(1000*var[6][3])) #Refresh Voltage
	myapp.ui.right_driver_vol_indicator.setValue(int(1000*var[7][3])) #Refresh Voltage
	
	if var[12][3] > 50:
		myapp.ui.status_mode_2.setText("<font color='#FF0000'>Emergency Stop</font>")
	elif var[13][3] > 50:
		myapp.ui.status_mode_2.setText("<font color='#00AA00'>Automatic Mode</font>")
	else:
		myapp.ui.status_mode_2.setText("<font color='#ffaa00'>Manual Mode</font>")
	
	if(var[14][3]!=0.0 or var[15][3] != 0.0):
		coords = var[14][3], var[15][3]
		myapp.ui.Map_Position_fullview.addMarker("Position Time: "+str(var[0][3]), *coords, **dict(
			icon="http://maps.gstatic.com/mapfiles/ridefinder-images/mm_20_red.png",
			draggable=False,
			title="Position Time: "+str(var[0][3])
		))
		myapp.ui.Map_Position_fullview.centerAt(var[14][3], var[15][3])

		coords = var[14][3], var[15][3]
		myapp.ui.graphicsView.addMarker("Position Time: "+str(var[0][3]), *coords, **dict(
			icon="http://maps.gstatic.com/mapfiles/ridefinder-images/mm_20_red.png",
			draggable=False,
			title="Position Time: "+str(var[0][3])
		))
		myapp.ui.Map_Position_view.centerAt(var[14][3], var[15][3])

	img = QImage()
	path = os.path.dirname(os.path.abspath(__file__))
	img.load(os.path.join(path, 'heading.png'))
	pixmap = QPixmap(img)
	transform = QTransform().rotate(-(var[3][3]*180.0/3.14)+90.0)
	pixmap = pixmap.transformed(transform, Qt.SmoothTransformation)
	myapp.ui.label_72.setPixmap(pixmap)
	
	img = QImage()
	img.load(os.path.join(path, 'roll.png'))
	pixmap = QPixmap(img)
	transform = QTransform().rotate((var[18][3]*180/3.14+180))
	pixmap = pixmap.transformed(transform, Qt.SmoothTransformation)
	myapp.ui.label_73.setPixmap(pixmap)
	
	img = QImage()
	img.load(os.path.join(path, 'pitch.png'))
	pixmap = QPixmap(img)
	transform = QTransform().rotate((var[19][3]*180.0/3.14))
	pixmap = pixmap.transformed(transform, Qt.SmoothTransformation)
	myapp.ui.label_74.setPixmap(pixmap)
	
	img = QImage()
	img.load(os.path.join(path, 'xyz.png'))
	pixmap = QPixmap(img)
	transform = QTransform().rotate((var[19][3]*180.0/3.14))
	pixmap = pixmap.transformed(transform, Qt.SmoothTransformation)
	myapp.ui.XYZ_img_label.setPixmap(pixmap)

	myapp.ui.Heading_label_.setText(str(round(var[3][3]*180.0/3.14,1))+" deg")
	myapp.ui.Roll_label_.setText(str((round(var[18][3]*180.0/3.14,1)+180.0)%360)+" deg")
	myapp.ui.Pitch_label_.setText(str(round(-var[19][3]*180.0/3.14,1))+" deg")

		
		
def refreshTrajectory():
	global myapp
	if  myapp.ui.verticalSlider.value()==1:
		myapp.ui.Start_Sequences_Button.setEnabled(True)
		points = calculateCirclePoints(myapp.ui.Diameter_SpinBox.value())
	elif myapp.ui.verticalSlider.value()==2:
		myapp.ui.Start_Sequences_Button.setEnabled(True)
		points=calculateLinePoints(myapp.ui.Distance_SpinBox.value())
	elif myapp.ui.verticalSlider.value()==3:
		if(myapp.path ==""):
			myapp.ui.Start_Sequences_Button.setEnabled(False)
			rospy.logwarn("[GUI]  Please load a Path File.")
			points=[[0],[0]]
		else:
			myapp.ui.Start_Sequences_Button.setEnabled(True)
			points=readPath()
	elif myapp.ui.verticalSlider.value()==4:
		myapp.ui.Start_Sequences_Button.setEnabled(True)
		points=calculateLinePoints(myapp.ui.Distance_SpinBox.value())
	else:
		points=[[0.0],[0.0]]
	myapp.ui.Movement_Previews_view.clear()
	if myapp.ui.verticalSlider.value()==3:
		myapp.ui.Movement_Previews_view.plot(points[0], points[1], pen=pyqtgraph.mkPen('r', width=3), symbol='d')
	else:
		myapp.ui.Movement_Previews_view.plot(points[0], points[1], pen=pyqtgraph.mkPen('r', width=3))

	path = os.path.dirname(os.path.abspath(__file__))
	img = QtWidgets.QGraphicsPixmapItem(QtGui.QPixmap(os.path.join(path, 'top.png')))
	img.scale(0.04,-0.04)
	img.translate(-2.02/0.04,-0.82/0.04)
	myapp.ui.Movement_Previews_view.addItem(img)
	lims = myapp.ui.Movement_Previews_view.getViewBox().childrenBounds()
	mini = min(lims[0][0], lims[1][0])
	maxi = max(lims[0][1], lims[1][1])
	myapp.ui.Movement_Previews_view.setXRange(mini-0.5, maxi+0.5)
	myapp.ui.Movement_Previews_view.setYRange(mini-0.5, maxi+0.5)

	myapp.ui.Movement_Previews_view.setLabel('left', "Robot Front Axis")
	myapp.ui.Movement_Previews_view.setLabel('bottom', "Robot Right Axis")

def readPath():
	global myapp, var
	file=open(myapp.path[0], "r")
	l=file.readline()
	origins=[0.0, 0.0, 0.0] # Origin: Lat, Long, Psi
	originsFlag=[False,False,False]
	points=[] # [[X, Y, Psi]]

	# Decrypt .path file
	while(l!=""):
		elements=l.replace("\n", "").replace("\r","").split(" ")
		if elements[0]=="POSITION":
			if elements[1]!="RELATIVE":
				originsFlag[0]=True
				originsFlag[1]=True
				origins[0]=float(elements[1])
				origins[1]=float(elements[2])
		elif elements[0]=="ORIENTATION":
			if elements[1]!="RELATIVE":
				originsFlag[2]=True
				origins[2]=float(elements[1])/180.*3.14

		elif elements[0][0]=="X":
			points.append([float(elements[0][1:]), float(elements[1][1:])])

		l=file.readline()
	output=[[],[]]
	global dX,dY,dAngle
	if originsFlag[0]==True:
		dX = 0
		dY = 0
	else:
		robotPose = utm.from_latlon(var[14][3], var[15][3])
		pathPose = utm.from_latlon(origins[0], origins[1])
		dXi = (pathPose[0]-robotPose[0])
		dYi = (pathPose[1]-robotPose[1])
		dX = dXi*cos(-var[3][3]) - dYi*sin(-var[3][3])
		dY = dYi*cos(-var[3][3]) + dXi*sin(-var[3][3])
		if sqrt(pow(dX,2)+pow(dY,2))>10:
			rospy.logwarn("[GUI] The robot is far from the Path Origin, PathReader may refuse Path file!")

	if originsFlag[2]==True:
		dAngle = 0
	else:
		dAngle = (origins[2]-var[3][3])# Calcul IMU Angle
		
	for i in points:
		X = i[0]
		Y = i[1]
		output[0].append(-(Y*cos(dAngle)+X*sin(dAngle)+dY)) # Xr
		output[1].append(X*cos(dAngle)-Y*sin(dAngle)+dX) # Yr
	return output

def calculateLinePoints(length):
	X=[]
	Y=[]
	for i in range(1000):
		X.append(0)
		Y.append(length*i/1000)
	return [X,Y]
def calculateCirclePoints(diameter):
	X=[]
	Y=[]
	for i in range(1000):
		X.append(diameter/2.0*cos(i*2.0*3.14/1000.0)-diameter/2)
		Y.append(diameter/2.0*sin(i*2.0*3.14/1000.0))
	return [X,Y]

def ACTUADORX(paquete):
    pub = rospy.Publisher("ACTUADORX", Float32, queue_size=10)
    rospy.init_node("Ceres_GUI",anonymous=True)
    rate = rospy.Rate(10) #10 Hz
    if not rospy.is_shutdown():
        hello_str = float(paquete)
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

def ACTUADORY(paquete):
    pub = rospy.Publisher("ACTUADORY", Float32, queue_size=10)
    rospy.init_node("Ceres_GUI",anonymous=True)
    rate = rospy.Rate(10) #10 Hz
    if not rospy.is_shutdown():
        hello_str = float(paquete)
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

def ACTUADORZ(paquete):
    pub = rospy.Publisher("ACTUADORZ", Float32, queue_size=10)
    rospy.init_node("Ceres_GUI",anonymous=True)
    rate = rospy.Rate(10) #10 Hz
    if not rospy.is_shutdown():
        hello_str = float(paquete)
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
		
		
if __name__ == "__main__":
	global var, myapp, i
	global eastingList, northingList, headingList, velocityXList, velocityYList, angularVelocityList, timeList, graphRate
	global procs

	pyqtgraph.setConfigOption('background', [247,246,246,255])
	pyqtgraph.setConfigOption('foreground', 'k')

	procs=[]
	i=0
	rate = 20 # Recording Frequency
	graphRate = 1 # Graph Refreshing Rate
	procsRate = 50 # External processes refreshing rate for reading the terminal messages

	logs = []
	
	var=[]
	var.append(["time","Time", "[s]", 0, False])				#0
	var.append(["Easting", "Robot Position East Axis", "[m]", 0, False]) 	#1
	var.append(["Northing", "Robot Position North Axis", "[m]", 0, False])	#2
	var.append(["heading", "Robot Heading to North", "[rad]", 0.0, False])	#3
	var.append(["v", "Robot Forward Speed", "[m/s]", 0, False])		#4
	var.append(["w", "Robot Angular Speed", "[rad/s]",  0, False])		#5
	var.append(["vl", "Left Driver Input Voltage", "[V]", 0, False])	#6
	var.append(["vr", "Right Driver Input Voltage", "[V]", 0, False])	#7
	var.append(["vd", "Reference Linear Speed", "[m/s]", 0, False])		#8
	var.append(["wd", "Reference Angular Speed", "[rad/s]", 0, False])	#9
	var.append(["CH1", "RC Channel 1 Value", "[%]", 0, False])		#10
	var.append(["CH2", "RC Channel 2 Value", "[%]", 0, False])		#11
	var.append(["AU", "RC Emergency Channel Value", "[%]", 0, False])	#12
	var.append(["AUX", "RC AUX Channel 1 Value", "[%]", 0, False])		#13
	var.append(["lat", "Robot Latitude", "[rad]", 0, False])		#14
	var.append(["lon", "Robot Longitude", "[rad]", 0, False])		#15
	var.append(["alt", "Robot Altitude", "[m]", 0, False])			#16
	var.append(["vy", "Robot Lateral Velocity", "[m/s]", 0, False])		#17
	var.append(["Roll", "Robot Roll", "[rad]", 3.14, False])			#18
	var.append(["Pitch", "Robot Pitch", "[rad]", 0.0, False])			#19


	timeList=[i*(1.0/graphRate) for i in range(60)]
	eastingList=[0 for i in range(60)]
	northingList=[0 for i in range(60)]
	headingList=[0 for i in range(60)]
	velocityXList=[0 for i in range(60)]
	velocityYList=[0 for i in range(60)]
	angularVelocityList=[0 for i in range(60)]

	app = QtWidgets.QApplication(sys.argv)
	myapp = CeresApplication()
	
	timer = QTimer()

	graphTimer = QTimer()
	procsTimer = QTimer()

	timeInit=time.time()
	
	adquire()
	
	timer.timeout.connect(refreshData)
	timer.start(int(1000/rate))

	graphTimer.timeout.connect(refreshGraph)
	graphTimer.start(int(1000/graphRate))

	procsTimer.timeout.connect(refreshProcs)
	procsTimer.start(int(1000/procsRate))
	
	myapp.ui.Map_Position_view.waitUntilReady()
	myapp.ui.Map_Position_view.centerAt(4.638, -74.08523)
	myapp.ui.Map_Position_view.setZoom(15)
	
	myapp.ui.Map_Position_fullview.waitUntilReady()
	myapp.ui.Map_Position_fullview.centerAt(4.638, -74.08523)
	myapp.ui.Map_Position_fullview.setZoom(16)

	myapp.ui.Movement_Previews_view.setLabel('left', "Robot Front Axis")
	myapp.ui.Movement_Previews_view.setLabel('bottom', "Robot Right Axis")
	myapp.ui.Movement_Previews_view.showGrid(x=True, y=True)
	path = os.path.dirname(os.path.abspath(__file__))
	img = QPixmap('top.png')
	#img.scaled(Qt.IgnoreAspectRatio)
	pixItem = QtWidgets.QGraphicsPixmapItem(img)
	# img = pyqtgraph.QtWidgets.QGraphicsPixmapItem(pyqtgraph.QtWidgets.QPixmap(os.path.join(path, 'top.png')))
	#img.scale(0.04,-0.04)
	#img.translate(-2.02/0.04,-0.82/0.04)
	myapp.ui.Movement_Previews_view.addItem(pixItem)
	
	myapp.show()
	sys.exit(app.exec_())

