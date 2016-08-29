#!/usr/bin/python2.7
#-*- encoding: UTF-8 -*-

import math
import time
import sys
import argparse
import Image
import numpy
import cv2
from naoqi import ALBroker
from naoqi import ALProxy
from naoqi import ALModule
from detection import redBallDetection,naoMarkDetection,yellowStickDetection,whiteBlockDetection
from movement import catchStick,hitBall,standWithStick,raiseStick,releaseStick

#------------------------------------------------全局变量定义-------------------------------------------------------------#
holeFlag = 0                    #球洞标志。   1：第一个球洞   2：第二个球洞   3：第三个球洞
robotHasFallenFlag = 0          #机器人跌倒检测标志
myGolf = None
codeTest = False                #测试标志，如果为True，代码直接从最后一个击球阶段进行

#------------------------------------------------参数定义（根据实际情况调整）----------------------------------------------#
#----------击球速度---------------------#
HitBallSpeed_FirstHole_Num_1 = 0.125
HitBallSpeed_FirstHole_Num_2 = 0.12

HitBallSpeed_SecondHole_Num_1 = 0.14
HitBallSpeed_SecondHole_Num_2 = 0.09
HitBallSpeed_SecondHole_Num_3 = 0.12

HitBallSpeed_ThirdHole_Num_1 = 0.1
HitBallSpeed_ThirdHole_Num_2 = 0.14
HitBallSpeed_ThirdHole_Num_3 = 0.12

#----------机器人直线走路参数补偿---------#
MoveDirectAngleOffSet = -30*math.pi/180.0
MoveDirectYOffSet = -0.05

#------------击球前旋转90度参数补偿-------#
AngleOffSetBeforeHitBall = 18*math.pi/180.0
XOffSetBeforeHitBall = -0.32
YOffSetBeforeHitBall = -0.32

#------------机器人击球前与红球的距离--------#
HitBallDist = 0.22

#-------------------------------------------------参数定义结束----------------------------------------------------------#
#-----------------------------------------------全局变量定义结束-------------------------------------------------------#

#---------------------------------------------------------------------------------------------------------------------#
#*********************************************************************************************************************
#@函数名：   calculation()
#@参数：     redBallInfo - 列表，包含机器人到红球的距离和角度值
#           targetInfo - 列表，包含机器人到目标的距离和角度值
#@返回值：   [angle,x,y]
#           angle - 机器人需要移动的角度
#           x - 机器人在x轴需要移动的距离
#           y - 机器人在y轴需要移动的距离
#@功能说明： 根据机器人、红球、目标三个点的距离及角度值，解算三角函数，
#           求出最终机器人移动到三点一线上需要移动的角度值，x和y。
#           为了避免机器人移动过程中碰撞到球，建议移动顺序是angle -> x -> y
#@最后修改日期：2016-8-22
#*********************************************************************************************************************
def calculation(redBallInfo,targetInfo):

    angleRobotAndBall = redBallInfo[1]
    angleRobotAndTarget = targetInfo[1]
    distRobotToBall = redBallInfo[0]
    distRobotToTarget = targetInfo[0]

    alpha = abs( angleRobotAndBall - angleRobotAndTarget)
    distBallToTarget2 = distRobotToBall*distRobotToBall + distRobotToTarget*distRobotToTarget -2*distRobotToBall*distRobotToTarget*math.cos(alpha)
    distBallToTarget = math.sqrt(distBallToTarget2)

    theta2 = (distRobotToBall*distRobotToBall + distBallToTarget2 - distRobotToTarget*distRobotToTarget)/(2*distRobotToBall*distBallToTarget)
    theta = math.acos(theta2)     #theta ：distRobotToBall 和 distBallToTarget 夹角

    if angleRobotAndBall - angleRobotAndTarget >= 0:
        if theta >= math.pi/2:
            #angle:锐角，负;    x： 正;   y: 正
            angle = theta - math.pi
            x = -distRobotToBall*math.cos(theta)
            y = distRobotToBall*math.sin(theta)
        else:
            #angle:钝角, 负;    x: 负;   y: 正
            angle = theta - math.pi
            x = -distRobotToBall*math.cos(theta)
            y = distRobotToBall*math.sin(theta)
    else:
        if theta >= math.pi/2:
            #angle: 锐角, 正;  x: 正;   y: 负
            angle = math.pi - theta
            x = -distRobotToBall*math.cos(theta)
            y = -distRobotToBall*math.sin(theta)
        else:
            #angle: 钝角, 正;  x: 负;   y: 负
            angle = math.pi - theta
            x = -distRobotToBall*math.cos(theta)
            y = -distRobotToBall*math.sin(theta)
    print "finalx = ",x,"finaly = ",y
    print "finalAngle = ",angle
    return [angle,x,y]

#---------------------------------------------------------------------------------------------------------------------#
#*********************************************************************************************************************
#@函数名：   robotRotate()
#@参数：     angle - 机器人旋转角度，弧度制
#           robotIP - 机器人IP地址
#           port - 机器人端口
#@返回值：   无
#@功能说明： 当机器人旋转角度大于90°的时候，为了减少误差就分两次旋转
#@最后修改日期：2016-8-6
#*********************************************************************************************************************
def robotRotate(angle, robotIP, port):
    MOTION = ALProxy("ALMotion", robotIP, port)
    if abs(angle) > math.pi/2:
        if angle >= 0:
            thetaz1 = angle - math.pi/2
            MOTION.setMoveArmsEnabled(False, False)
            MOTION.moveTo(0.0,0.0,math.pi/2,moveConfig)
            time.sleep(1)
            MOTION.setMoveArmsEnabled(False, False)
            MOTION.moveTo(0.0,0.0,thetaz1,moveConfig)
        else:
            thetaz1 = angle + math.pi/2
            MOTION.setMoveArmsEnabled(False, False)
            MOTION.moveTo(0.0,0.0,-math.pi/2,moveConfig)
            time.sleep(1)
            MOTION.setMoveArmsEnabled(False, False)
            MOTION.moveTo(0.0,0.0,thetaz1,moveConfig)
    else:
        MOTION.setMoveArmsEnabled(False, False)
        MOTION.moveTo(0.0,0.0,angle,moveConfig)

#自定义一个类，主要用于绑定事件和对应的执行函数
class Golf(ALModule):
    def __init__(self, name, robotIP, port):
        ALModule.__init__(self, name)
        self.robotIP = robotIP
        self.port = port
        self.memory = ALProxy("ALMemory")
        self.motion = ALProxy("ALMotion")
        self.posture = ALProxy("ALRobotPosture")
        self.memory.subscribeToEvent("ALChestButton/TripleClickOccurred", "myGolf", "chestButtonPressed")
        self.memory.subscribeToEvent("FrontTactilTouched", "myGolf", "frontTactilTouched")
        self.memory.subscribeToEvent("MiddleTactilTouched", "myGolf", "middleTactilTouched")
        self.memory.subscribeToEvent("RearTactilTouched", "myGolf", "rearTactilTouched")
        self.memory.subscribeToEvent("robotHasFallen", "myGolf", "fallDownDetected")

    def frontTactilTouched(self):
        print "frontTactilTouched!!!!!!!!!!!!!"
        self.memory.unsubscribeToEvent("FrontTactilTouched", "myGolf")
        global holeFlag
        holeFlag = 1
        self.memory.subscribeToEvent("FrontTactilTouched", "myGolf", "frontTactilTouched")

    def middleTactilTouched(self):
        print "middleTactilTouched!!!!!!!!!!!!!!!!!!"
        self.memory.unsubscribeToEvent("MiddleTactilTouched", "myGolf")
        global holeFlag
        holeFlag = 2
        self.memory.subscribeToEvent("MiddleTactilTouched", "myGolf", "middleTactilTouched")

    def rearTactilTouched(self):
        print "rearTactilTouched!!!!!!!!!!!!!!!!!!"
        self.memory.unsubscribeToEvent("RearTactilTouched", "myGolf")
        global holeFlag
        holeFlag = 3
        self.memory.subscribeToEvent("RearTactilTouched", "myGolf", "rearTactilTouched")

    def chestButtonPressed(self):
        print "chestButtonPressed!!!!!!!!!!!!!!!!"
        self.memory.unsubscribeToEvent("ALChestButton/TripleClickOccurred", "myGolf")
        global holeFlag
        holeFlag = 0
        self.motion.angleInterpolationWithSpeed("LHand", 0.8, 1)
        time.sleep(2)
        self.motion.angleInterpolationWithSpeed("LHand", 0.2, 1)
        self.motion.rest()
        self.memory.subscribeToEvent("ALChestButton/TripleClickOccurred", "myGolf", "chestButtonPressed")

    def fallDownDetected(self):
        print "fallDownDetected!!!!!!!!!!!!!!!!"
        self.memory.unsubscribeToEvent("robotHasFallen", "myGolf")
        global robotHasFallenFlag
        robotHasFallenFlag = 1
        self.posture.goToPosture("StandInit", 0.5)
        releaseStick(self.robotIP, self.port)
        time.sleep(15)
        catchStick(self.robotIP, self.port)
        standWithStick(0.1, self.robotIP, self.port)
        self.memory.subscribeToEvent("robotHasFallen", "myGolf", "fallDownDetected")

#----------------------------------------------------主函数------------------------------------------------------------#
def main(robotIP, port):

    myBroker = ALBroker("myBroker", "0.0.0.0", 0, robotIP, port)

    #设置机器人各个模块代理
    MEMORY = ALProxy("ALMemory", robotIP, port)
    REDBALL = ALProxy("ALRedBallDetection", robotIP, port)
    LANDMARK = ALProxy("ALLandMarkDetection", robotIP, port)
    TTS = ALProxy("ALTextToSpeech", robotIP, port)
    MOTION = ALProxy("ALMotion", robotIP, port)
    POSTURE = ALProxy("ALRobotPosture", robotIP, port)
    CAMERA = ALProxy("ALVideoDevice", robotIP, port)
    TOUCH = ALProxy("ALTouch", robotIP, port)
    SENSORS = ALProxy("ALSensors", robotIP, port)

    #实例化
    global myGolf
    myGolf = Golf("myGolf", robotIP, port)
    MOTION.setFallManagerEnabled(True)
    redBallDetectMethod = 1

    try:
        #-----------------------------------------------------主死循环，待机状态-----------------------------------------------------#
        while(True):
            if(MOTION.robotIsWakeUp() == False):
                MOTION.wakeUp()
                POSTURE.goToPosture("StandInit", 0.5)
                TTS.say("游戏开始")
                releaseStick(robotIP, port)
                time.sleep(5)
                catchStick(robotIP, port)
                time.sleep(2)
                raiseStick(robotIP, port)
                standWithStick(0.1, robotIP, port)
                hitBall(0.15, robotIP, port)
                time.sleep(1)
                catchStick(robotIP, port)
                time.sleep(10)
                raiseStick(robotIP, port)
                standWithStick(0.1, robotIP, port)
            time.sleep(1)
            print "holeFlag =",holeFlag
            if(holeFlag != 0):
                #---------------------------------------------次死循环，工作状态------------------------------------------------------#
                while(True):
                    MOTION.setMoveArmsEnabled(False, False)

                    if(codeTest == False):
                        #---------------------高尔夫击球第一个阶段，直打----------------------------------------------#
                        if(holeFlag == 1):
                            hitBall(HitBallSpeed_FirstHole_Num_1, robotIP, port)
                        elif(holeFlag == 2):
                            hitBall(HitBallSpeed_SecondHole_Num_1, robotIP, port)
                        elif(holeFlag == 3):
                            hitBall(HitBallSpeed_ThirdHole_Num_1, robotIP, port)

                        time.sleep(0.5)
                        raiseStick(robotIP, port)
                        standWithStick(0.1, robotIP, port)
                        MOTION.moveTo(0, 0, -math.pi/2+10*math.pi/180.0, moveConfig)
                        time.sleep(0.5)

                        if(holeFlag == 1):
                            MOTION.moveTo(1.00, -0.15, -45*math.pi/180.0, moveConfig)
                            redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, True, redBallDetectMethod, 0, robotIP, port)
                        if(holeFlag == 2):
                            MOTION.moveTo(0, -1.0, 0, moveConfig)
                            time.sleep(1)
                            MOTION.moveTo(0.60, MoveDirectYOffSet, MoveDirectAngleOffSet, moveConfig)
                            redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, True, redBallDetectMethod, 0, robotIP, port)
                        if(holeFlag == 3):
                            MOTION.moveTo(0.0, 0.0,-50*math.pi/180.0, moveConfig)
                            time.sleep(0.5)
                            MOTION.moveTo(0, 1.50, 0, moveConfig)
                            redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, False, redBallDetectMethod, 0, robotIP, port)

                        #----------------------高尔夫击球第二个阶段，靠近红球--------------------------------------------------#

                        #检测不到就往前走60cm以防万一
                        if(redBallInfo == False):
                            MOTION.moveTo(0.60, MoveDirectYOffSet, MoveDirectAngleOffSet,moveConfig)
                            time.sleep(0.5)
                            redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, True, redBallDetectMethod, 0, robotIP, port)
                        while(redBallInfo[0] >=0.60):
                            if(redBallInfo[0]*math.cos(redBallInfo[1]) - 0.60 <=0.30):
                                break
                            MOTION.moveTo(0,0,redBallInfo[1],moveConfig)
                            time.sleep(0.5)
                            MOTION.moveTo(0.60, MoveDirectYOffSet, MoveDirectAngleOffSet,moveConfig)
                            time.sleep(0.5)
                            if(redBallInfo[0] - 0.60 < 0.60):
                                redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, False, redBallDetectMethod, 0, robotIP, port)
                                break
                            if(redBallInfo[0] - 0.60 >= 1.2):
                                redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, True, redBallDetectMethod, 0, robotIP, port)
                            else:
                                redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, False, redBallDetectMethod, 0, robotIP, port)

                        #移动到机器人30cm前
                        MOTION.moveTo(0,0,redBallInfo[1],moveConfig)
                        time.sleep(0.5)
                        MOTION.moveTo(redBallInfo[0]-0.40, MoveDirectYOffSet/2, MoveDirectAngleOffSet/2, moveConfig)

                        #最后一次对红球的校准
                        redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, False, redBallDetectMethod, 20, robotIP, port)
                        MOTION.moveTo(0,0,redBallInfo[1],moveConfig)
                        time.sleep(0.5)
                        MOTION.moveTo(redBallInfo[0]-0.3,0,0,moveConfig)
                        #-----------------------高尔夫击球第三个阶段，后两个球洞的黄杆识别--------------------------------------#
                        if(holeFlag == 2 or holeFlag == 3):
                            #三点一线，第一次移动
                            redBallInfo = redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, False, redBallDetectMethod, 20, robotIP, port)
                            stickInfo = yellowStickDetection(yellow_thresholdMin, yellow_thresholdMax, robotIP, port)
                            theta,x,y = calculation(redBallInfo,stickInfo)
                            robotRotate(theta, robotIP, port)
                            time.sleep(0.5)
                            MOTION.moveTo(x-0.3,0.0,0.0,moveConfig)
                            time.sleep(0.5)
                            MOTION.moveTo(0.0,y,0.0,moveConfig)
                            MOTION.angleInterpolation("HeadYaw",0,0.5,True)
                            #校准，第二次移动
                            redBallInfo = redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, False, redBallDetectMethod, 20, robotIP, port)
                            stickInfo = yellowStickDetection(yellow_thresholdMin, yellow_thresholdMax, robotIP, port)
                            theta,x,y = calculation(redBallInfo,stickInfo)
                            robotRotate(theta, robotIP, port)
                            time.sleep(0.5)
                            MOTION.moveTo(x-0.2,0.0,0.0,moveConfig)
                            time.sleep(0.5)
                            MOTION.moveTo(0.0,y,0.0,moveConfig)
                            MOTION.angleInterpolation("HeadYaw",0,0.5,True)

                            #击球
                            MOTION.moveTo(0.0, 0.0, math.pi/2-AngleOffSetBeforeHitBall, moveConfig)
                            time.sleep(0.5)
                            MOTION.moveTo(XOffSetBeforeHitBall, 0.0, 0.0, moveConfig)
                            time.sleep(0.5)
                            MOTION.moveTo(0.0, YOffSetBeforeHitBall, 0.0, moveConfig)

                            redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, False, redBallDetectMethod, 20, robotIP, port)
                            MOTION.moveTo(redBallInfo[0]*math.cos(redBallInfo[1])-HitBallDist, 0, 0,moveConfig)
                            time.sleep(0.5)

                            if(holeFlag == 2):
                                hitBall(HitBallSpeed_SecondHole_Num_2, robotIP, port)
                            elif(holeFlag == 3):
                                hitBall(HitBallSpeed_ThirdHole_Num_2, robotIP, port)
                            time.sleep(0.5)
                            raiseStick(robotIP, port)
                            standWithStick(0.1, robotIP, port)
                            MOTION.moveTo(0, 0, -math.pi/2+10*math.pi/180.0, moveConfig)
                            time.sleep(0.5)

                            #while循环，目的是缩短机器人到红球的距离
                            redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, True, redBallDetectMethod, 0, robotIP, port)
                            #检测不到就往前走60cm以防万一
                            if(redBallInfo == False):
                                MOTION.moveTo(0.60, MoveDirectYOffSet, MoveDirectAngleOffSet,moveConfig)
                                time.sleep(0.5)
                                redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, True, redBallDetectMethod, 0, robotIP, port)
                            while(redBallInfo[0] >=0.60):
                                if(redBallInfo[0]*math.cos(redBallInfo[1]) - 0.60 <=0.30):
                                    break
                                MOTION.moveTo(0,0,redBallInfo[1],moveConfig)
                                time.sleep(1)
                                MOTION.moveTo(0.60, MoveDirectYOffSet, MoveDirectAngleOffSet,moveConfig)
                                time.sleep(1)
                                if(redBallInfo[0] - 0.60 < 0.60):
                                    redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, False, redBallDetectMethod, 0, robotIP, port)
                                    break
                                if(redBallInfo[0] - 0.60 >= 1.2):
                                    redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, True, redBallDetectMethod, 0, robotIP, port)
                                else:
                                    redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, False, redBallDetectMethod, 0, robotIP, port)

                            #移动到机器人30cm前
                            MOTION.moveTo(0,0,redBallInfo[1],moveConfig)
                            time.sleep(0.5)
                            MOTION.moveTo(redBallInfo[0]-0.3, MoveDirectYOffSet/2, MoveDirectAngleOffSet/2, moveConfig)

                    #--------------------------------------近距离击球测试，实际代码需要注释掉-----------
                    #redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, False, redBallDetectMethod, 0, robotIP, port)
                    #MOTION.moveTo(0,0,redBallInfo[1],moveConfig)
                    #time.sleep(1)
                    #MOTION.moveTo(redBallInfo[0]-0.3, MoveDirectYOffSet/2, MoveDirectAngleOffSet/2, moveConfig)

                    #-------------------------------------测试代码结束--------------------------------

                    #---------------------------------高尔夫击球第四个阶段-----------------------------------------------------------#
                    redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, False, redBallDetectMethod, 20, robotIP, port)
                    markInfo = naoMarkDetection(robotIP, port)
                    #-------------------------------基于nao_mark 击球
                    if(markInfo != False):

                        #三点一线，第一次移动
                        theta,x,y = calculation(redBallInfo,markInfo)
                        robotRotate(theta, robotIP, port)
                        time.sleep(0.5)
                        MOTION.moveTo(x-0.3,0.0,0.0,moveConfig)
                        time.sleep(0.5)
                        MOTION.moveTo(0.0,y,0.0,moveConfig)
                        MOTION.angleInterpolation("HeadYaw",0,0.5,True)
                        #校准，第二次移动
                        redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, False, redBallDetectMethod, 20, robotIP, port)
                        markInfo = naoMarkDetection(robotIP, port)
                        if(markInfo != False):
                            theta,x,y = calculation(redBallInfo,markInfo)
                            robotRotate(theta, robotIP, port)
                            time.sleep(0.5)
                            MOTION.moveTo(x-0.2,0.0,0.0,moveConfig)
                            time.sleep(0.5)
                            MOTION.moveTo(0.0,y,0.0,moveConfig)
                            MOTION.angleInterpolation("HeadYaw",0,0.5,True)
                        else:
                            MOTION.moveTo(0.05,0,0.0,moveConfig)
                        #击球
                        MOTION.moveTo(0.0, 0.0, math.pi/2-AngleOffSetBeforeHitBall, moveConfig)
                        time.sleep(0.5)
                        MOTION.moveTo(XOffSetBeforeHitBall, 0.0, 0.0, moveConfig)
                        time.sleep(0.5)
                        MOTION.moveTo(0.0, YOffSetBeforeHitBall, 0.0, moveConfig)

                        redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, False, redBallDetectMethod, 20, robotIP, port)
                        MOTION.moveTo(redBallInfo[0]*math.cos(redBallInfo[1])-HitBallDist, 0, 0,moveConfig)
                        time.sleep(0.5)
                        if(holeFlag == 1):
                            hitBall(HitBallSpeed_FirstHole_Num_2, robotIP, port)
                        elif(holeFlag == 2):
                            hitBall(HitBallSpeed_SecondHole_Num_3, robotIP, port)
                        elif(holeFlag == 3):
                            hitBall(HitBallSpeed_ThirdHole_Num_3, robotIP, port)
                        time.sleep(0.5)
                        raiseStick(robotIP, port)
                        standWithStick(0.1, robotIP, port)
                        MOTION.moveTo(0, 0, -math.pi/2, moveConfig)
                        time.sleep(0.5)
                    #-----------------------nao_mark没有检测到，基于黄杆击球----------------------------------------#
                    else:
                        #三点一线，第一次移动
                        redBallInfo = redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, False, redBallDetectMethod, 20, robotIP, port)
                        stickInfo = yellowStickDetection(yellow_thresholdMin, yellow_thresholdMax, robotIP, port)
                        theta,x,y = calculation(redBallInfo,stickInfo)
                        robotRotate(theta, robotIP, port)
                        time.sleep(0.5)
                        MOTION.moveTo(x-0.3,0.0,0.0,moveConfig)
                        time.sleep(0.5)
                        MOTION.moveTo(0.0,y,0.0,moveConfig)
                        MOTION.angleInterpolation("HeadYaw",0,0.5,True)
                        #校准，第二次移动
                        redBallInfo = redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, False, redBallDetectMethod, 20, robotIP, port)
                        stickInfo = yellowStickDetection(yellow_thresholdMin, yellow_thresholdMax, robotIP, port)
                        theta,x,y = calculation(redBallInfo,stickInfo)
                        robotRotate(theta, robotIP, port)
                        time.sleep(0.5)
                        MOTION.moveTo(x-0.2,0.0,0.0,moveConfig)
                        time.sleep(0.5)
                        MOTION.moveTo(0.0,y,0.0,moveConfig)
                        MOTION.angleInterpolation("HeadYaw",0,0.5,True)

                        #击球
                        MOTION.moveTo(0.0, 0.0, math.pi/2-AngleOffSetBeforeHitBall, moveConfig)
                        time.sleep(0.5)
                        MOTION.moveTo(XOffSetBeforeHitBall, 0.0, 0.0, moveConfig)
                        time.sleep(0.5)
                        MOTION.moveTo(0.0, YOffSetBeforeHitBall, 0.0, moveConfig)

                        redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, False, redBallDetectMethod, 20, robotIP, port)
                        MOTION.moveTo(redBallInfo[0]*math.cos(redBallInfo[1])-HitBallDist, 0, 0,moveConfig)
                        time.sleep(0.5)
                        if(holeFlag == 1):
                            hitBall(HitBallSpeed_FirstHole_Num_2, robotIP, port)
                        elif(holeFlag == 2):
                            hitBall(HitBallSpeed_SecondHole_Num_3, robotIP, port)
                        elif(holeFlag == 3):
                            hitBall(HitBallSpeed_ThirdHole_Num_3, robotIP, port)
                        time.sleep(0.5)
                        raiseStick(robotIP, port)
                        standWithStick(0.1, robotIP, port)
                        MOTION.moveTo(0, 0, -math.pi/2, moveConfig)
                        time.sleep(0.5)
                    global holeFlag
                    holeFlag = 0
                    break
                #------------------------------------------------次死循环结束--------------------------------------------------------#
        #--------------------------------------------------------主死循环结束--------------------------------------------------------#
    except KeyboardInterrupt:
        print
        print "Interrupted by user, shutting down"
        myBroker.shutdown()
        sys.exit(0)

#---------------------------------------------------------------------------------------------------------------------#
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number.")
    parser.add_argument("--MaxStepX", type=float, default=0.02,
                        help="maximum forward translation along X (meters)")
    parser.add_argument("--MaxStepY", type=float, default=0.12,
                        help="absolute maximum lateral translation along Y (meters)")
    parser.add_argument("--MaxStepTheta", type=float, default=0.3,
                        help="absolute maximum rotation around Z (radians)")
    parser.add_argument("--MaxStepFrequency", type=float, default=0.5,
                        help="maximum step frequency (normalized, unit-less)")
    parser.add_argument("--StepHeight", type=float, default=0.02,
                        help="peak foot elevation along Z (meters)")
    parser.add_argument("--TorsoWx", type=float, default=0.0,
                        help="peak torso rotation around X (radians)")
    parser.add_argument("--TorsoWy", type=float, default=0.0,
                        help="peak torso rotation around Y (radians)")
    parser.add_argument("--robotHeight", type=float, default=0.478,
                        help="robot's height")

    args = parser.parse_args()

    #机器人的行走参数
    moveConfig = [["MaxStepX",args.MaxStepX],["MaxStepY",args.MaxStepY],["MaxStepTheta",args.MaxStepTheta],["MaxStepFrequency",args.MaxStepFrequency],
                    ["StepHeight",args.StepHeight],["TorsoWx",args.TorsoWx],["TorsoWy",args.TorsoWy]]

    robotIP = args.ip
    port = args.port

    #黄色HSV阈值
    yellow_thresholdMin = numpy.array([20,130,100])
    yellow_thresholdMax = numpy.array([45,255,255])
    #红色HSV阈值
    red_thresholdMin = numpy.array([160,85,110])
    red_thresholdMax = numpy.array([179,255,255])

    robotHeight = args.robotHeight

    main(robotIP, port)
