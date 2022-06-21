#!/home/regulus/catkin_ws/src/CERES-gui_and_advanced_Navigation/ceres/CamaraDistancia/bin/python3

'''
Created on 19Jun2015
Stream rgb and depth video side-by-side using openni2 opencv-python (cv2).
RGB is overlayed on top on readable-depth. In addition, streams are aligned, mirror-corrected, and synchronized.

Requires the following libraries:
    1. OpenNI-Linux-<Platform>-2.2 <Library and driver>
    2. primesense-2.2.0.30 <python bindings>
    3. Python 2.7+
    4. OpenCV 2.4.X

Current features:
    1. Convert primensense oni -> numpy
    2. Stream and display rgb || depth || rgbd overlayed
    3. Keyboard commands
        press esc to exit
        press s to save current screen and distancemap
    4. Sync and registered depth & rgb streams
    5. Print distance to center pixel
    6. Masks and overlays rgb stream on the depth stream

NOTE:
    1. On device streams:  IR and RGB streams do not work together
       Depth & IR  = OK
       Depth & RGB = OK
       RGB & IR    = NOT OK
@author: Carlos Torres <carlitos408@gmail.com>
'''

import numpy as np
import cv2
from primesense import openni2  # , nite2
from primesense import _openni2 as c_api
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
global enviocont
enviocont=0
tracker = cv2.TrackerCSRT_create()
## Path of the OpenNI redistribution OpenNI2.so or OpenNI2.dll
# Windows
#dist = 'D:\programation datation\Drivers\Camara primesense\Redist'
# OMAP
# dist = '/home/carlos/Install/kinect/OpenNI2-Linux-ARM-2.2/Redist/'
# Linux
dist ='/home/regulus/Downloads/OpenNI/Redist'


def DistanciaGripper(paquete):
    pub = rospy.Publisher("DistanciaGripper", Float32MultiArray, queue_size=1)
    rospy.init_node("CamaraPrimeSense",anonymous=True)
    rate = rospy.Rate(1000) #100 Hz
    if not rospy.is_shutdown():
        hello_str = Float32MultiArray()
        hello_str.data=paquete
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

## inicializar openni y chequear funcionamiento
openni2.initialize(
    dist)  # 'C:\Program Files\OpenNI2\Redist\OpenNI2.dll') # accepts the path of the OpenNI redistribution
if (openni2.is_initialized()):
    print("openNI2 iniciado")
else:
    print("openNI2 no iniciado")

## registrar el dispositivo
dev = openni2.Device.open_any()
## crear el streams stream
rgb_stream = dev.create_color_stream()
depth_stream = dev.create_depth_stream()

ResX=640
RESY=480
#ResX=320
#RESY=240
##configure the depth_stream
# print 'Get b4 video mode', depth_stream.get_video_mode()
depth_stream.set_video_mode(
    c_api.OniVideoMode(pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM, resolutionX=ResX, resolutionY=RESY,
                       fps=20))
rgb_stream.set_video_mode(c_api.OniVideoMode(pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888, resolutionX=ResX,
 resolutionY=RESY, fps=20))

## Check and configure the mirroring -- default is True
# print 'Mirroring info1', depth_stream.get_mirroring_enabled()
depth_stream.set_mirroring_enabled(False)
rgb_stream.set_mirroring_enabled(False)

## start the stream
rgb_stream.start()
depth_stream.start()

## synchronize the streams
dev.set_depth_color_sync_enabled(True)  # synchronize the streams

## IMPORTANT: ALIGN DEPTH2RGB (depth wrapped to match rgb stream)
dev.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)


##help(dev.set_image_registration_mode)


def get_rgb():
    """
    Returns numpy 3L ndarray to represent the rgb image.
    """
    bgr = np.frombuffer(rgb_stream.read_frame().get_buffer_as_uint8(), dtype=np.uint8).reshape(RESY,ResX,3)
    #bgr=cv2.resize(bgr,(640,480),3)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return rgb, hsv

def get_depth():
    """
    Returns numpy ndarrays representing the raw and ranged depth images.
    Outputs:
        dmap:= distancemap in mm, 1L ndarray, dtype=uint16, min=0, max=2**12-1
        d4d := depth for dislay, 3L ndarray, dtype=uint8, min=0, max=255
    Note1:
        fromstring is faster than asarray or frombuffer
    Note2:
        .reshape(120,160) #smaller image for faster response
                OMAP/ARM default video configuration
        .reshape(240,320) # Used to MATCH RGB Image (OMAP/ARM)
                Requires .set_video_mode
    """
    dmap = np.frombuffer(depth_stream.read_frame().get_buffer_as_uint16(), dtype=np.uint16).reshape(RESY,
                                                                                                    ResX)  # Works & It's FAST
    d4d = np.uint8(dmap.astype(float) * 255 / 2 ** 12 - 1)  # Correct the range. Depth images are 12bits
    d4d = 255 - cv2.cvtColor(d4d, cv2.COLOR_GRAY2RGB)
    return dmap, d4d

def mask_rgbd(d4d, rgb, th=0):
    """
    Overlays images and uses some blur to slightly smooth the mask
    (3L ndarray, 3L ndarray) -> 3L ndarray
    th:= threshold
    """
    mask = d4d.copy()
    # mask = cv2.GaussianBlur(mask, (5,5),0)
    idx = (mask > th)
    mask[idx] = rgb[idx]
    return mask

def getContours(img,imgContour):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        areaMin = 300
        if area > areaMin:
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 7)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            #print(len(approx))
            x , y , w, h = cv2.boundingRect(approx)
            cv2.rectangle(imgContour, (x , y ), (x + w , y + h ), (0, 0, 255), 5)
            bbox=[x,y,w,h]
            cv2.putText(imgContour, "Points: " + str(len(approx)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7,
                        (0, 255, 0), 2)
            cv2.putText(imgContour, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                        (0, 255, 0), 2)
            return bbox

## main loop
s = 0
done = False
kernelC=np.ones((5,5),np.uint8)

FramesContador=1000
def drawBox(img,bbox):
    global enviocont
    x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
    cv2.rectangle(img, (x, y), ((x + w), (y + h)), (255, 0, 255), 3, 3 )
    cv2.putText(img, "Tracking", (100, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    enviocont=enviocont +1
    if enviocont > 0:
        DistanciaGripper([int(x+w/2), int(y+h/2), dmap[int(y+h/2), int(x+w/2)]])
        enviocont=0

while not done:
    key = cv2.waitKey(1) & 255
    ## Read keystrokes
    if key == 27:  # terminate
        print("\tESC key detected!")
        done = True
    elif chr(key) == 's':  # screen capture
        print("\ts key detected. Saving image and distance map {}".format(s))
        cv2.imwrite("ex5_" + str(s) + '.png', canvas)
        np.savetxt("ex5dmap_" + str(s) + '.out', dmap)
        # s+=1 # uncomment for multiple captures
    # if
    rangomax=np.array([71,255,255])
    rangomin=np.array([40,66,123])
    ## Streams
    # RGB
    rgb,hsv = get_rgb()
    rgb= cv2.GaussianBlur(rgb, (7, 7), 1)


    # DEPTH
    dmap, d4d = get_depth()

    # Overlay rgb over the depth stream
    rgbd = mask_rgbd(d4d, rgb)

    # Ejecutar mascara
    mascara = cv2.inRange(hsv, rangomin, rangomax)
    mascara = cv2.erode(mascara, kernelC, iterations=1)
    mascara = cv2.GaussianBlur(mascara, (7, 7), 1)
    opening = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernelC)
    x, y, w, h = cv2.boundingRect(opening)

    ##### CANVAS

    ## Distance map
    if int(x+w/2)>639:
        XO=639
    elif int(x+w/2)<1:
        XO = 1
    else:
        XO=int(x+w/2)
    if int(y+h/2)>479:
        YO=479
    elif int(y+h/2)<1:
        YO = 1
    else:
        YO = int(y+h/2)

    ##Detectar Gripper
    rangomax = np.array([145, 255, 255])
    rangomin = np.array([116, 63, 25])
    # Ejecutar mascara
    mascara1 = cv2.inRange(hsv, rangomin, rangomax)
    mascara1 = cv2.erode(mascara1, kernelC, iterations=1)
    mascara1 = cv2.GaussianBlur(mascara1, (7, 7), 1)
    opening1 = cv2.morphologyEx(mascara1, cv2.MORPH_OPEN, kernelC)
    x1, y1, w1, h1 = cv2.boundingRect(opening1)

    # canvas

    ## Distance map
    if int(x1 + w1 / 2) > 639:
        XO = 639
    elif int(x1 + w1 / 2) < 1:
        XO = 1
    else:
        XO = int(x1 + w1 / 2)
    if int(y1 + h1 / 2) > 479:
        YO = 479
    elif int(y1 + h1 / 2) < 1:
        YO = 1
    else:
        YO = int(y1 + h1 / 2)  # Se esta vectorizando
    #print('Gripper pixel is {} mm away'.format(dmap[YO, XO]))
    #print('Pixel X:' + str(YO) + ' Y:' + str(XO))
    result = cv2.bitwise_and(rgb, rgb, mask=mascara1)
    result = cv2.GaussianBlur(result, (7, 7), 1)
    gray=cv2.cvtColor(result,cv2.COLOR_RGB2GRAY)
    Bordes=cv2.Canny(gray,255,215)
    kernel = np.ones((5, 5))
    imgDil = cv2.dilate(Bordes, kernel, iterations=1)
    Ccaja=getContours(imgDil, rgb)
    #print(FramesContador)
    try:
        if FramesContador>100:
            tracker.init(result, Ccaja)
            print("Hecho")
            drawBox(rgb, Ccaja)
            FramesContador=0
        else:
            FramesContador=FramesContador+1
            success, bbox = tracker.update(result)
            if success:
                drawBox(result, bbox)
            else:
                tracker.init(result, Ccaja)
    except:
        try:
            FramesContador = FramesContador + 1
            success, bbox = tracker.update(result)
            if success:
                drawBox(result, bbox)
        except:
            pass

    cv2.rectangle(rgb, (x, y), (x + w, y + h), (0, 255, 0), 3)
    cv2.circle(rgb, (int(x + w / 2), int(y + h / 2)), 5, (0, 0, 255))
    #canvas = np.hstack((d4d, rgb, rgbd))

    cv2.imshow('depth || rgb || rgbd', rgb)
    #cv2.imshow('Bordes',Bordes)
    #cv2.imshow('ColorNegro', result)
# end while

## Release resources
cv2.destroyAllWindows()
rgb_stream.stop()
depth_stream.stop()
openni2.unload()
print("Terminated")
