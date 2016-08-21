#!/usr/bin/python2.7
#-*- encoding: UTF-8 -*-

import math
import time
import sys
import argparse
import Image
import numpy
import cv2
from naoqi import ALProxy
from movement import standWithStick

#----------------------------------------------全局变量定义------------------------------------------------------------#
topCameraHeight = 0.532         #robot top camera height
bottomCameraHeight = 0.478      #robot bottom camera height
cameraHFOV = 60.97               #camera horizontal field of view
cameraVFOV = 47.64               #camera vertical field of view
imageWidth = 640                #image width pixel
imageHeight = 480               #image height pixel
naoMarkDiameter = 0.0913        #nao mark diameter
yellowStickHeight = 0.46        #yellow stick height
#--------------------------------------------全局变量定义结束-----------------------------------------------------------#

#---------------------------------------------------------------------------------------------------------------------#
#*********************************************************************************************************************
#@函数名：   searchBall()
#@参数：     angle - 机器人头部水平旋转角度，角度制
#           isenabled - True：绝对角度
#                       False：相对角度
#           robotIP - 机器人IP地址
#           port - 机器人端口
#@返回值：   如果检测到红球就返回一个包含红球信息的列表，否则返回None
#@功能说明： 写入一个角度值，机器人在该水平角度上寻找红球.只在redBallDetection()函数里的method=0的情况被调用
#@最后修改日期：2016-8-6
#*********************************************************************************************************************
def searchBall(angle, isenabled, robotIP="127.0.0.1", port=9559):
    MEMORY = ALProxy("ALMemory", robotIP, port)
    REDBALL = ALProxy("ALRedBallDetection", robotIP, port)
    MOTION = ALProxy("ALMotion", robotIP, port)
    #周期为200毫秒，也就是每隔200毫秒刷入一次内存（memory）
    period = 200
    REDBALL.subscribe("Redball",period,0.0)
    MEMORY.insertData("redBallDetected",None)
    time.sleep(0.5)
    MOTION.angleInterpolation("HeadYaw",angle*math.pi/180.0,0.5,isenabled)
    time.sleep(2)
    #读取检测到的红球数据，如果没有被检测到则返回None
    temp = MEMORY.getData("redBallDetected")
    MEMORY.insertData("redBallDetected",None)
    REDBALL.unsubscribe("Redball")
    return temp

#---------------------------------------------------------------------------------------------------------------------#
#*********************************************************************************************************************
#@函数名：   pictureHandle()
#@参数：     cameraID - 0：上方摄像头
#                      1：下发摄像头
#           thresholdMin - 目标在hsv颜色模式下最小阈值
#           thresholdMax - 目标在hsv颜色模式下最大阈值
#           robotIP - 机器人IP地址
#           port - 机器人端口
#@返回值：   获取轮廓值成功就返回轮廓值列表，否则返回空列表
#@功能说明： 用于红球和黄杆检测，获取照片并进行初步处理，提取目标轮廓值，若有多个色块，则返回最大色块的轮廓值
#@最后修改日期：2016-8-6
#*********************************************************************************************************************
def pictureHandle(cameraID, thresholdMin, thresholdMax, robotIP="127.0.0.1", port=9559):
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
    img.save("camImage.png", "PNG")

    imgTest = cv2.imread("camImage.png",1)
    #RGB转换为HSV颜色空间
    hsv = cv2.cvtColor(imgTest,cv2.COLOR_BGR2HSV)
    #获取掩膜
    mask=cv2.inRange(hsv,thresholdMin,thresholdMax)
    #原始图像与掩膜做位运算，提取色块
    res=cv2.bitwise_and(imgTest,imgTest,mask=mask)
    #将hsv转化为rgb
    color = cv2.cvtColor(res,cv2.COLOR_HSV2RGB)
    #转换为灰度图
    gray = cv2.cvtColor(color,cv2.COLOR_RGB2GRAY)
    #二值化
    ret,th1 = cv2.threshold(gray,100,255,cv2.THRESH_BINARY)

    #开运算，去除噪声(开运算=先腐蚀再膨胀)
    #kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,8))
    #opening = cv2.morphologyEx(th1, cv2.MORPH_OPEN, kernel)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(2,2))
    erosion = cv2.erode(th1,kernel,iterations = 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(4,8))
    dilation = cv2.dilate(erosion,kernel,iterations = 1)

    #--------------------------------测试代码-----------------------------------#
    #cv2.namedWindow("image",cv2.WINDOW_NORMAL)
    #cv2.namedWindow("image1",cv2.WINDOW_NORMAL)
    #cv2.imshow("image",imgTest)
    #cv2.imshow("image1",erosion)
    #cv2.waitKey(0)
    #--------------------------------测试结束-----------------------------------#

    #获取轮廓
    contours, hierarchy = cv2.findContours(dilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #如果存在多个轮廓，取最大的那个
    if(len(contours) > 1):
        lengths = map(len, contours)
        return [contours[lengths.index(max(lengths))]]
    return contours

#---------------------------------------------------------------------------------------------------------------------#
#*********************************************************************************************************************
#@函数名：   redBallDetection()
#@参数：     red_thresholdMin - 红色在hsv颜色模式下最小阈值
#           red_thresholdMax - 红色在hsv颜色模式下最大阈值
#           method - 0 :使用官方提供的API对红球进行识别
#                    1 : 使用自己的方法对红球进行识别
#           addAngle - 在机器人头部垂直方向当前角度上增加的角度，角度制
#           robotIP - 机器人IP地址
#           port - 机器人端口
#@返回值：   检测成功返回一个列表，包含红球距离和角度值
#            检测失败返回False
#@功能说明： 对红球进行识别和定位
#           单目测距，距离越远误差越大，可通过多次校验减少误差
#@最后修改日期：2016-8-20
#*********************************************************************************************************************
def redBallDetection(red_thresholdMin, red_thresholdMax, method=1, addAngle=0, robotIP="127.0.0.1", port=9559):
    MOTION = ALProxy("ALMotion", robotIP, port)
    CAMERA = ALProxy("ALVideoDevice", robotIP, port)
    TTS = ALProxy("ALTextToSpeech", robotIP, port)

    #初始化机器人姿势
    MOTION.setMoveArmsEnabled(False, False)
    standWithStick(0.3, robotIP, port)

    MOTION.angleInterpolation("HeadPitch", 0, 0.5, True)
    MOTION.angleInterpolation("HeadYaw", 0, 0.5, True)

    MOTION.angleInterpolation("HeadPitch", addAngle*math.pi/180.0, 0.5, False)

    angleFlag = 0
    cameraID = 1
    val = []

    TTS.say("开始检测红球")
    if method == 0:
        #首先识别正前方，然后是左边45度，最后是右边45度，三个方向由标志位angleFlag进行标记
        for i in range(0,3):
            #先打开下方摄像头进行检测
            CAMERA.setActiveCamera(1)
            cameraID = 1
            val = searchBall(0, True, robotIP, port)
            time.sleep(1)
            if(val and isinstance(val,list) and len(val) >= 2):
                angleFlag = 0
                i -= 1
                break

            val = searchBall(45, True, robotIP, port)
            time.sleep(1)
            if(val and isinstance(val,list) and len(val) >= 2):
                angleFlag = 1
                i -= 1
                break

            val = searchBall(-45,True, robotIP, port)
            time.sleep(1)
            if(val and isinstance(val,list) and len(val) >= 2):
                angleFlag = -1
                i -= 1
                break

            #先打开上方摄像头进行检测
            CAMERA.setActiveCamera(0)
            cameraID = 0
            val = searchBall(0,True, robotIP, port)
            time.sleep(1)
            if(val and isinstance(val,list) and len(val) >= 2):
                angleFlag = -1
                i -= 1
                break

            val = searchBall(45,True, robotIP, port)
            time.sleep(1)
            if(val and isinstance(val,list) and len(val) >= 2):
                angleFlag = -1
                i -= 1
                break

            val = searchBall(-45,True, robotIP, port)
            time.sleep(1)
            if(val and isinstance(val,list) and len(val) >= 2):
                angleFlag = -1
                i -= 1
                break
            TTS.say("球在哪里呢")
        #如果3次都没有检测到就返回False
        if(i == 2):
            TTS.say("红球检测失败")
            return False
        #检测完红球后，最后看向正前方
        MOTION.angleInterpolation("HeadYaw",0,0.5,True)
        time.sleep(0.5)
        TTS.say("哈哈!我看到了!")
        #看到球之后,获取球的水平垂直偏角
        ballinfo = val[1]
        thetah = ballinfo[0]
        thetav = ballinfo[1]

        #根据红球方向标志位确定红球具体角度值
        if(cameraID == 0):
            thetav = thetav + 1.2*math.pi/180.0 + addAngle*math.pi/180.0
            cameraHeight = topCameraHeight
        elif(cameraID == 1):
            thetav = thetav + 39.7*math.pi/180.0 + addAngle*math.pi/180.0
            cameraHeight = bottomCameraHeight
        if(angleFlag == 1):
            theta = thetah + 45*math.pi/180.0
        elif(angleFlag == -1):
            theta = thetah - 45*math.pi/180.0
        else:
            theta = thetah

        distBall = cameraHeight/(math.tan(thetav))
        return [distBall,theta]

    elif method == 1:
        redBallContours = []
        for i in range(0,3):
            CAMERA.setActiveCamera(1)
            cameraID = 1
            MOTION.angleInterpolation("HeadYaw",0,0.5,True)
            redBallContours = pictureHandle(cameraID, red_thresholdMin, red_thresholdMax, robotIP, port)
            if(redBallContours != []):
                angleFlag = 0
                i -= 1
                break
            #检测左前方45°
            MOTION.angleInterpolation("HeadYaw",45*math.pi/180,0.5,True)
            redBallContours = pictureHandle(cameraID, red_thresholdMin, red_thresholdMax, robotIP, port)
            if(redBallContours != []):
                angleFlag = 1
                i -= 1
                break
            #检测右前方45°
            MOTION.angleInterpolation("HeadYaw",-45*math.pi/180,0.5,True)
            redBallContours = pictureHandle(cameraID, red_thresholdMin, red_thresholdMax, robotIP, port)
            if(redBallContours != []):
                angleFlag = -1
                i -= 1
                break

            CAMERA.setActiveCamera(0)
            cameraID = 0
            MOTION.angleInterpolation("HeadYaw",0,0.5,True)
            redBallContours = pictureHandle(cameraID, red_thresholdMin, red_thresholdMax, robotIP, port)
            if(redBallContours != []):
                angleFlag = 0
                i -= 1
                break
            #检测左前方45°
            MOTION.angleInterpolation("HeadYaw",45*math.pi/180,0.5,True)
            redBallContours = pictureHandle(cameraID, red_thresholdMin, red_thresholdMax, robotIP, port)
            if(redBallContours != []):
                angleFlag = 1
                i -= 1
                break
            #检测右前方45°
            MOTION.angleInterpolation("HeadYaw",-45*math.pi/180,0.5,True)
            redBallContours = pictureHandle(cameraID, red_thresholdMin, red_thresholdMax, robotIP, port)
            if(redBallContours != []):
                angleFlag = -1
                i -= 1
                break
            TTS.say("球在哪里呢")
        #检测3次都没有检测到，就返回False
        if(i == 2):
            TTS.say("红球检测失败")
            return False
        #让机器人看向正前方
        MOTION.angleInterpolation("HeadYaw",0,0.5,True)
        TTS.say("哈哈，我看到了")

        cnt = redBallContours[0]
        bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])

        #计算红球最下方的点和图片中心的距离，正：红球在中心左边   负：红球在中心右边
        disX = imageWidth/2 - bottommost[0]
        disY = bottommost[1] - imageHeight/2

        #计算红球最下方和图片中心的偏转角度，机器人水平视角为60.97°
        ballAngleH = cameraHFOV / imageWidth * disX
        ballAngleV = cameraVFOV / imageHeight * disY
        if(cameraID == 0):
            ballAngleV = ballAngleV + 1.2 + addAngle
            cameraHeight = topCameraHeight
        elif(cameraID == 1):
            ballAngleV = ballAngleV + 39.7 + addAngle
            cameraHeight = bottomCameraHeight
        if(angleFlag == 1):
            ballAngleH = ballAngleH + 45
        elif(angleFlag == -1):
            ballAngleH = ballAngleH - 45
        else:
            ballAngleH = ballAngleH

        #机器人到球的距离
        distBall = cameraHeight / (math.tan(ballAngleV * math.pi/180.0)) / math.cos(ballAngleH*math.pi/180.0)

        #---------------------------------测试代码--------------------------------------------#
        #print "bottommost = ",bottommost
        #print "distBall = ",distBall
        #print "distBallDiret = ",distBallDiret
        #print "ballAngleH = ",ballAngleH*math.pi/180.0
        #imgTest = cv2.imread("camImage.png",1)
        #cv2.circle(imgTest, bottommost, 1, (0,255,255), -1)
        #cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        #cv2.namedWindow("image2",cv2.WINDOW_NORMAL)
        #cv2.namedWindow("image3",cv2.WINDOW_NORMAL)
        #cv2.imshow("image",imgTest)
        #cv2.imshow("image",imgTest)
        #cv2.imshow("image3",th1)
        #cv2.waitKey(0)
        #---------------------------------测试结束--------------------------------------------#
        return [distBall,ballAngleH*math.pi/180.0]

#---------------------------------------------------------------------------------------------------------------------#
#*********************************************************************************************************************
#@函数名：   naoMarkDetection()
#@参数：     robotIP - 机器人IP地址
#           port - 机器人端口
#@返回值：   检测成功返回一个列表，包含nao mark标志距离和角度值
#            检测失败返回False
#@功能说明： 计算机器人到nao mark的距离和角度。循环5次对nao mark进行识别，无法识别返回False，
#           识别成功根据等距离法计算出nao mark距离
#@最后修改日期：2016-8-20
#*********************************************************************************************************************
def naoMarkDetection(robotIP="127.0.0.1", port=9559):
    MOTION = ALProxy("ALMotion", robotIP, port)
    CAMERA = ALProxy("ALVideoDevice", robotIP, port)
    TTS = ALProxy("ALTextToSpeech", robotIP, port)
    LANDMARK = ALProxy("ALLandMarkDetection", robotIP, port)
    MEMORY = ALProxy("ALMemory", robotIP, port)
    #初始化机器人姿势
    standWithStick(0.1, robotIP, port)
    #设置机器人上方摄像头开启
    CAMERA.setActiveCamera(0)
    #加载mark识别EVENT
    memvalue1 = "LandmarkDetected"
    alpha = 0
    size = 0
    period = 200
    LANDMARK.subscribe("Test_LandMark",period,0.0)

    MOTION.angleInterpolation("HeadPitch",0,0.5,True)
    time.sleep(1.5)

    for i in range(0,2):
        #检测正前方
        MOTION.angleInterpolation("HeadYaw",0,0.5,True)
        time.sleep(2.5)
        val = MEMORY.getData(memvalue1)
        if(val and isinstance(val,list) and len(val) >= 2):
            TTS.say("前方我看到了")
            markInfo = val[1]
            shapeInfo = markInfo[0]
            rshapeInfo = shapeInfo[0]
            alpha = rshapeInfo[1]
            size = rshapeInfo[4]
            angleFlag = 0
            i -= 1
            break
        #检测左边45度
        MOTION.angleInterpolation("HeadYaw",45*math.pi/180.0,0.5,True)
        time.sleep(2.5)
        val = MEMORY.getData(memvalue1)
        if(val and isinstance(val,list) and len(val) >= 2):
            TTS.say("左边我看到了")
            markInfo = val[1]
            shapeInfo = markInfo[0]
            rshapeInfo = shapeInfo[0]
            alpha = rshapeInfo[1]
            size = rshapeInfo[4]
            angleFlag = 1
            i -= 1
            break
        #检测右边45度
        MOTION.angleInterpolation("HeadYaw",-45*math.pi/180.0,0.5,True)
        time.sleep(2.5)
        val = MEMORY.getData(memvalue1)
        if(val and isinstance(val,list) and len(val) >= 2):
            TTS.say("右边我看到了")
            markInfo = val[1]
            shapeInfo = markInfo[0]
            rshapeInfo = shapeInfo[0]
            alpha = rshapeInfo[1]
            size = rshapeInfo[4]
            angleFlag = -1
            i -= 1
            break

        TTS.say("没有检测到标志")
    #如果2次都没检测到mark就返回False
    if(i == 1):
        return False

    if(angleFlag == 1):
        alpha += math.pi/4
    elif(angleFlag == -1):
        alpha -= math.pi/4
    else:
        alpha = alpha

    #利用等比例法测出机器人到mark的距离
    distMark = (naoMarkDiameter / size) / math.cos(alpha)

    #返回nao mark距离和角度值
    print "the disMark = ",distMark
    print "the markAngle = ",alpha
    return [distMark,alpha]

#---------------------------------------------------------------------------------------------------------------------#
#*********************************************************************************************************************
#@函数名：   yellowStickDetection()
#@参数：     yellow_thresholdMin - 黄色在hsv颜色模式下最小阈值
#           yellow_thresholdMax - 黄色在hsv颜色模式下最大阈值
#           robotIP - 机器人IP地址
#           port - 机器人端口
#@返回值：   检测成功就返回一个列表，包括黄杆到机器人的距离和偏转角
#           检测失败则返回False
#@功能说明： 识别球洞黄杆，计算机器人和黄杆的偏转角度以及距离。
#           采用opencv进行图片处理，每次获取黄杆重心像素点的坐标值。
#           单目测距，等比例法，有误差。
#@最后修改日期：2016-8-20
#*********************************************************************************************************************
def yellowStickDetection(yellow_thresholdMin, yellow_thresholdMax, robotIP="127.0.0.1", port=9559):
    MOTION = ALProxy("ALMotion", robotIP, port)
    CAMERA = ALProxy("ALVideoDevice", robotIP, port)
    TTS = ALProxy("ALTextToSpeech", robotIP, port)
    #初始化机器人姿势
    standWithStick(0.1, robotIP, port)
    MOTION.angleInterpolation(["HeadPitch", "HeadYaw"], 0, 0.5, True)
    imageContours = []
    angleFlag = 0

    for i in range(0,3):
        #检测正前方
        MOTION.angleInterpolation("HeadYaw",0,0.5,True)
        #获取照片轮廓特征点
        time.sleep(2)
        imageContours = pictureHandle(0,yellow_thresholdMin,yellow_thresholdMax, robotIP, port)
        if(imageContours != []):
            angleFlag = 0
            i -= 1
            break
        #检测左前方45°
        MOTION.angleInterpolation("HeadYaw",45*math.pi/180.0,0.5,True)
        time.sleep(2)
        imageContours = pictureHandle(0,yellow_thresholdMin,yellow_thresholdMax, robotIP, port)
        if(imageContours != []):
            angleFlag = 1
            i -= 1
            break
        #检测右前方45°
        MOTION.angleInterpolation("HeadYaw",-45*math.pi/180.0,0.5,True)
        time.sleep(2)
        imageContours = pictureHandle(0,yellow_thresholdMin,yellow_thresholdMax, robotIP, port)
        if(imageContours != []):
            angleFlag = -1
            i -= 1
            break
        TTS.say("没有检测到黄杆")
    #检测3次都没有检测到，就返回False
    if(i == 2):
        return False
    #让机器人看向正前方
    MOTION.angleInterpolation("HeadYaw",0,0.5,True)

    TTS.say("哈哈，我看到了")
    cnt = imageContours[0]
    #获取黄杆最下方的点
    bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])

    #计算黄杆最下方的点和图片中心的距离，正：黄杆在中心左边   负：黄杆在中心右边
    disX = imageWidth/2 - bottommost[0]
    disY = bottommost[1] - imageHeight/2
    #计算黄杆最下方的点和图片中心的偏转角度，机器人水平视角为60.97°
    stickAngleH = cameraHFOV / imageWidth * disX
    stickAngleV = cameraVFOV / imageHeight * disY + 1.2
    if(angleFlag == 1):
        stickAngleH = stickAngleH + 45
    elif(angleFlag == -1):
        stickAngleH = stickAngleH - 45
    else:
        stickAngleH = stickAngleH
    #手动减去0.06，校准误差，不科学但是很管用
    distStick = (topCameraHeight - 0.06) / (math.tan(stickAngleV * math.pi/180.0)) / math.cos(stickAngleH*math.pi/180.0)

    #--------------------------测试代码-----------------------------------#
    #print "distStick = ",distStick
    #print "stickAngleH = ",stickAngleH
    #imgTest = cv2.imread("camImage.png",1)
    #cv2.circle(imgTest,bottommost, 5, (0,0,255), -1)
    #cv2.namedWindow("image",cv2.WINDOW_NORMAL)
    #cv2.namedWindow("image2",cv2.WINDOW_NORMAL)
    #cv2.namedWindow("image3",cv2.WINDOW_NORMAL)
    #cv2.imshow("image",imgTest)
    #cv2.imshow("image",imgTest)
    #cv2.imshow("image3",th1)
    #cv2.waitKey(0)
    #--------------------------测试结束-----------------------------------#
    return [distStick, stickAngleH*math.pi/180]

#---------------------------------------------------------------------------------------------------------------------#
#*********************************************************************************************************************
#@函数名：   whiteBlockDetection()
#@参数：     cameraID - 0:机器人上方摄像头
#                       1： 机器人下方摄像头
#@返回值：  检测到白色障碍物就返回机器人需要往右偏转的角度值，弧度制
#           没有检测到白色障碍物就返回False
#@功能说明： 在机器人、球、球洞三点一线的情况下，对白色障碍物和白色边界进行检测，如果中间有白色障碍物就往右偏转一定角度
#@最后修改日期：2016-8-18
#*********************************************************************************************************************
def whiteBlockDetection(cameraID=0, robotIP="127.0.0.1", port=9559):

    CAMERA = ALProxy("ALVideoDevice", robotIP, port)
    CAMERA.setActiveCamera(0)
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
    img.save("camImage.png", "PNG")
    imgTest = cv2.imread("camImage.png",1)

    #imgTest = cv2.imread("photo_02.png",1)
    #把黄杆以上的部分减掉
    #RGB转换为HSV颜色空间
    hsv = cv2.cvtColor(imgTest,cv2.COLOR_BGR2HSV)
    #获取掩膜
    mask=cv2.inRange(hsv,yellow_thresholdMin,yellow_thresholdMax)
    #原始图像与掩膜做位运算，提取色块
    res=cv2.bitwise_and(imgTest,imgTest,mask=mask)
    #将hsv转化为rgb
    color = cv2.cvtColor(res,cv2.COLOR_HSV2RGB)
    #转换为灰度图
    gray = cv2.cvtColor(color,cv2.COLOR_RGB2GRAY)
    #二值化
    ret,th1 = cv2.threshold(gray,100,255,cv2.THRESH_BINARY)

    #开运算，去除噪声(开运算=先腐蚀再膨胀)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(2,2))
    erosion = cv2.erode(th1,kernel,iterations = 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(4,8))
    dilation = cv2.dilate(erosion,kernel,iterations = 1)

    #获取轮廓
    contours, hierarchy = cv2.findContours(dilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #如果存在多个轮廓，取最大的那个
    if(len(contours) > 1):
        lengths = map(len, contours)
        contours = [contours[lengths.index(max(lengths))]]
    #根据轮廓值找极点，最下的那个点，像素坐标值
    cnt = contours[0]
    bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])

    cv2.circle(imgTest, bottommost, 5, (0,0,255), -1)

    #图像减运算，把黄杆以上的减掉
    maskArray1 = numpy.zeros((bottommost[1] + 10, imageWidth), numpy.uint8)
    maskArray2 = numpy.ones((imageHeight - bottommost[1] - 10, imageWidth), numpy.uint8)
    maskArray = numpy.concatenate((maskArray1, maskArray2), axis=0)
    imgTest = cv2.bitwise_and(imgTest, imgTest, mask = maskArray)

    imgROI = imgTest[0:imageHeight, imageWidth/2-20:imageWidth/2+20]
    #检测中间有没有障碍物
    grayROI = cv2.cvtColor(imgROI,cv2.COLOR_RGB2GRAY)
    ret,thresholdROI = cv2.threshold(grayROI,200,255,cv2.THRESH_BINARY)
    #获取轮廓
    contoursROI, hierarchy = cv2.findContours(thresholdROI,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #如果有白色障碍物
    if(contoursROI != []):
        #如果存在多个轮廓，取最大的那个
        if(len(contoursROI) > 1):
            lengthsROI = map(len, contoursROI)
            contoursROI = [contoursROI[lengthsROI.index(max(lengthsROI))]]
        #根据轮廓值找极点，最右的那个点，像素坐标值
        cntROI = contoursROI[0]
        rightmost = tuple(cntROI[cntROI[:,:,0].argmax()][0])
        cv2.circle(imgROI, rightmost, 5, (0,0,255), -1)
        #计算障碍物极点到机器人的距离
        disY = rightmost[1] - imageHeight/2
        ObjectAngleV = cameraVFOV / imageHeight * disY + 1.2
        distObject = robotHeight/(math.tan(ObjectAngleV * math.pi/180.0))
        ctheta = 0.3 / distObject
        finalAngle = - math.atan(ctheta) * math.pi/180.0

    #--------------------------------测试代码-----------------------------------#
    #cv2.circle(imgTest,rightmost, 5, (0,0,255), -1)
    #cv2.circle(imgTest,leftmost, 5, (0,0,255), -1)
    #cv2.circle(imgTest,(rightmost[0],leftmost[1]), 5, (0,0,255), -1)
    cv2.namedWindow("image",cv2.WINDOW_NORMAL)
    #cv2.namedWindow("image1",cv2.WINDOW_NORMAL)
    cv2.imshow("image",imgROI)
    #cv2.imshow("image1",grayROI)
    cv2.waitKey(0)
    #--------------------------------测试结束-----------------------------------#

    #    return finalAngle
    #没有障碍物就返回False
    #else:
    #    return False


#---------------------------------------------------------------------------------------------------------------------#
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number.")
    parser.add_argument("--robotHeight", type=float, default=0.478,
                        help="Robot's height.")

    args = parser.parse_args()

    #机器人下方摄像头的高度
    robotHeight = args.robotHeight
    #黄色HSV阈值
    yellow_thresholdMin = numpy.array([20,150,100])
    yellow_thresholdMax = numpy.array([60,255,255])
    #红色HSV阈值
    red_thresholdMin = numpy.array([150,80,0])
    red_thresholdMax = numpy.array([179,255,255])

    robotIP = args.ip
    port = args.port

    yellowStickDetection(yellow_thresholdMin, yellow_thresholdMax, robotIP, port)
    #pictureHandle(0, yellow_thresholdMin, yellow_thresholdMax, robotIP, port)
    #redBallDetection(red_thresholdMin, red_thresholdMax, 1, 0, robotIP, port)
    #naoMarkDetection(robotIP, port)
    #whiteBlockDetection(0,robotIP,port)
