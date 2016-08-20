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
from detection import redBallDetection,naoMarkDetection,yellowStickDetection,whiteBlockDetection,whiteBorderDetection
from movement import catchStick,hitBall,standWithStick,raiseStick

#------------------------------------------------全局变量定义-------------------------------------------------------------#
holeFlag = 0
robotHasFallenFlag = 0
myGolf = None
#-----------------------------------------------全局变量定义结束-------------------------------------------------------#

#---------------------------------------------------------------------------------------------------------------------#
#*********************************************************************************************************************
#@函数名：   calculation()
#@参数：     redBallInfo - 列表，包含机器人到红球的距离和角度值
#           targetInfo - 列表，包含机器人到目标的距离和角度值
#@返回值：   列表，包含机器人最终旋转角度，x,y
#@功能说明： 根据机器人、红球、目标三个点的距离及角度值，解算三角函数，
#           求出最终机器人需要移动的角度值，x和y。
#@最后修改日期：2016-8-18
#*********************************************************************************************************************
def calculation(redBallInfo,targetInfo):

    thetah = redBallInfo[1]
    alpha = targetInfo[1]
    dx = redBallInfo[0]
    d1 = targetInfo[0]

    theta3 = abs( thetah- alpha)
    dball = dx
    dmark = d1
    dbm2 = dx*dx + d1*d1 -2*dx*d1*math.cos(theta3)
    dbm = math.sqrt(dbm2)       #dbm : 球和mark标志的距离

    ctheta4 = (dball*dball + dbm2 - dmark*dmark)/(2*dball*dbm)
    theta4 = math.acos(ctheta4)     #theta4 ：机器人到球直线 和 球到mark直线  夹角
    print "distance for mark to ball = ",dbm
    print "temp angle = ",theta4
    if thetah - alpha >= 0:
        if theta4 >= math.pi/2:
            theta = theta4 - math.pi/2      #theta就是最终机器人需要旋转的角度，旋转90°是因为要和球保持垂直
            x = dball*math.sin(theta4)      #x:最终机器人x轴的移动距离
            y = dball*math.cos(theta4)      # y：最终机器人y轴的移动距离
        else:
            theta = math.pi/2 - theta4
            x = dball*math.sin(theta4)
            y = dball*math.cos(theta4)

    else:
        if theta4 >= math.pi/2:
            theta = 3*math.pi/2 - theta4
            x = -dball*math.sin(theta4)
            y = dball*math.cos(theta4)
        else:
            theta = -math.pi/2 - theta4
            x = -dball*math.sin(theta4)
            y = dball*math.cos(theta4)

    print "finalx = ",x,"finaly = ",y
    print "finalAngle = ",theta
    return [theta,x,y]

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

class Golf(ALModule):
    def __init__(self, name, robotIP, port):
        ALModule.__init__(self, name)
        self.robotIP = robotIP
        self.port = port
        self.memory = ALProxy("ALMemory")
        self.memory.subscribeToEvent("ChestButtonPressed", "myGolf", "chestButtonPressed")
        self.memory.subscribeToEvent("FrontTactilTouched", "myGolf", "frontTactilTouched")
        self.memory.subscribeToEvent("MiddleTactilTouched", "myGolf", "middleTactilTouched")
        self.memory.subscribeToEvent("RearTactilTouched", "myGolf", "rearTactilTouched")
        self.memory.subscribeToEvent("robotHasFallen", "myGolf", "fallDownDetected")

    def frontTactilTouched(self):
        self.memory.unsubscribeToEvent("FrontTactilTouched", "myGolf")
        holeFlag = 1
        self.memory.subscribeToEvent("FrontTactilTouched", "myGolf", "frontTactilTouched")

    def middleTactilTouched(self):
        self.memory.unsubscribeToEvent("MiddleTactilTouched", "myGolf")
        holeFlag = 2
        self.memory.subscribeToEvent("MiddleTactilTouched", "myGolf", "middleTactilTouched")

    def rearTactilTouched(self):

        self.memory.unsubscribeToEvent("RearTactilTouched", "myGolf")
        holeFlag = 3
        self.memory.subscribeToEvent("RearTactilTouched", "myGolf", "rearTactilTouched")

    def chestButtonPressed(self):
        self.memory.unsubscribeToEvent("ChestButtonPressed", "myGolf")
        holeFlag = 0
        self.memory.subscribeToEvent("ChestButtonPressed", "myGolf", "chestButtonPressed")

    def fallDownDetected(self):
        self.memory.unsubscribeToEvent("robotHasFallen", "myGolf")
        robotHasFallenFlag = 1
        catchStick(self.robotIP, self.port)
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

    redBallDetectMethod = 1

    #初始化机器人电机
    MOTION.wakeUp()
    #让机器人保持一个便于运动的姿势
    POSTURE.goToPosture("StandInit", 0.5)
    TTS.say("游戏开始")

    catchStick(robotIP, port)
    #让杆放在合适位置，准备移动
    raiseStick(robotIP, port)y
    standWithStick(0.1, robotIP, port)
    try:
        while(True):
            time.sleep(1)
            if(holeFlag != 0):
                while(True):
                    MOTION.setMoveArmsEnabled(False, False)
                    redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, redBallDetectMethod, 0, robotIP, port)
                    while(redBallInfo[0] >= 1):
                        MOTION.moveTo(0,0,redBallInfo[1],moveConfig)
                        time.sleep(1)
                        MOTION.moveTo(0.8,0,0,moveConfig)
                        time.sleep(1)
                        redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, redBallDetectMethod, 0, robotIP, port)
                        if(holeFlag == 0 or robotHasFallenFlag == 1):
                            break

                    if(holeFlag == 0 or robotHasFallenFlag == 1):
                        robotHasFallenFlag = 0
                        break

                    MOTION.moveTo(0,0,redBallInfo[1],moveConfig)
                    time.sleep(1)
                    MOTION.moveTo(redBallInfo[0]-0.2,0,0,moveConfig)

                    if(holeFlag == 0 or robotHasFallenFlag == 1):
                        robotHasFallenFlag = 0
                        break

                    redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, redBallDetectMethod, 20, robotIP, port)
                    #检测mark标志
                    markInfo = naoMarkDetection(robotIP, port)
                    if(markInfo != False):
                        #根据机器人、球、球洞解算三角关系，并移到最终击球位置
                        theta,x,y = calculation(redBallInfo,markInfo)

                        #修正阈值，在场上需要根据实际情况调整
                        theta -= 0.1
                        x -= 0.30
                        y -= 0.05
                        #先移动角度
                        robotRotate(theta, robotIP, port)
                        time.sleep(1.0)
                        MOTION.moveTo(x,0.0,0.0,moveConfig)
                        time.sleep(1.0)
                        MOTION.moveTo(0.0,y,0.0,moveConfig)
                        MOTION.angleInterpolation("HeadYaw",0,0.5,True)

                        if(holeFlag == 0 or robotHasFallenFlag == 1):
                            robotHasFallenFlag = 0
                            break

                        if(holeFlag == 1):
                            hitBall(0.1, robotIP, port)
                            time.sleep(1)
                        elif(holeFlag == 2):
                            if(tempAngle = whiteBlockDetection(0, robotIP, port)):
                                #旋转tempAngle把球打出去
                            else:
                                hitBall(0.1, robotIP, port)
                                time.sleep(1)
                        elif(holeFlag == 3):
                            pass
                    #没有检测到mark标志，检测黄杆
                    else:
                        stickInfo = yellowStickDetection(yellow_thresholdMin, yellow_thresholdMax, robotIP, port)
                        theta,x,y = calculation(redBallInfo,stickInfo)
                        x -= 0.25
                        y -= 0.05
                        robotRotate(theta, robotIP, port)
                        time.sleep(1.0)
                        MOTION.moveTo(x,0.0,0.0,moveConfig)
                        time.sleep(1.0)
                        MOTION.moveTo(0.0,y,0.0,moveConfig)
                        MOTION.angleInterpolation("HeadYaw",0,0.5,True)

                        if(holeFlag == 0 or robotHasFallenFlag == 1):
                            robotHasFallenFlag = 0
                            break

                        if(holeFlag == 1):
                            hitBall(0.1, robotIP, port)
                            time.sleep(1)
                        elif(holeFlag == 2):
                            if(tempAngle = whiteBlockDetection(0, robotIP, port)):
                                #旋转tempAngle把球打出去
                            else:
                                hitBall(0.1, robotIP, port)
                                time.sleep(1)
                        elif(holeFlag == 3):
                            if(tempAngle = whiteBorderDetection(0, robotIP, port)):
                                #旋转tempAngle把球打出去
                            else:
                                hitBall(0.1, robotIP, port)
                                time.sleep(1)

                        if(holeFlag == 0 or robotHasFallenFlag == 1):
                            robotHasFallenFlag = 0
                            break

    except KeyboardInterrupt:
        print
        print "Interrupted by user, shutting down"
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
    parser.add_argument("--MaxStepY", type=float, default=0.14,
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
    yellow_thresholdMin = numpy.array([20,150,100])
    yellow_thresholdMax = numpy.array([60,255,255])
    #红色HSV阈值
    red_thresholdMin = numpy.array([150,90,0])
    red_thresholdMax = numpy.array([179,255,255])

    robotHeight = args.robotHeight

    main(robotIP, port)
