#-*- encoding: UTF-8 -*-

import cv2
import numpy
import argparse
import Image
import time
from naoqi import ALProxy
from naoqi import ALBroker

def nothing(x):
    pass

def choseHSV(filePath):
    img=cv2.imread(filePath,1)
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    cv2.namedWindow("image",cv2.WINDOW_NORMAL)

    cv2.createTrackbar('minH','image',0,179,nothing)
    cv2.createTrackbar('minS','image',0,255,nothing)
    cv2.createTrackbar('minV','image',0,255,nothing)
    cv2.createTrackbar('maxH','image',0,179,nothing)
    cv2.createTrackbar('maxS','image',0,255,nothing)
    cv2.createTrackbar('maxV','image',0,255,nothing)
    while(1):
        minH=cv2.getTrackbarPos('minH','image')
        minS=cv2.getTrackbarPos('minS','image')
        minV=cv2.getTrackbarPos('minV','image')
        maxH=cv2.getTrackbarPos('maxH','image')
        maxS=cv2.getTrackbarPos('maxS','image')
        maxV=cv2.getTrackbarPos('maxV','image')

        thresholdMin = numpy.array([minH, minS, minV])
        thresholdMax = numpy.array([maxH, maxS, maxV])
        mask=cv2.inRange(hsv,thresholdMin,thresholdMax)
        res=cv2.bitwise_and(img,img,mask = mask)
        cv2.imshow('image',res)
        k=cv2.waitKey(1)
        if k==ord('q'):
            break
    cv2.destroyAllWindows()

def readRobotTemperature():
    pass

def takePhotos(cameraID, robotIP, port):
    CAMERA = ALProxy("ALVideoDevice", robotIP, port)
    CAMERA.setActiveCamera(cameraID)
    # VGA  设置分辨率为2:640*480 0:160*120
    resolution = 2
    # RGB  设置颜色空间为RGB
    colorSpace = 11
    videoClient = CAMERA.subscribe("python_client", resolution, colorSpace, 5)
    #设置曝光度模式
    CAMERA.setCamerasParameter(videoClient,22,2)
    time.sleep(0.5)
    #获取照片
    naoImage = CAMERA.getImageRemote(videoClient)
    CAMERA.unsubscribe(videoClient)

    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]

    #装换为PIL图片格式
    img = Image.fromstring("RGB", (imageWidth, imageHeight), array)
    img.save("photo.png", "PNG")

def test():
    broker = ALBroker("broker","0.0.0.0",0,"127.0.0.1",9559)
    MOTION = ALProxy("ALMotion")
    MOTION.moveTo(1,0,0)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--filePath", type=str, help="file path.")
    args = parser.parse_args()
    #choseHSV(args.filePath)
    #takePhotos(0, "192.168.1.103", 9559)
    test()
