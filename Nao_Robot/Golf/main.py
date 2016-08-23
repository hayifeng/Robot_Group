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
from movement import catchStick,hitBall,standWithStick,raiseStick,releaseStick

#------------------------------------------------全局变量定义-------------------------------------------------------------#
holeFlag = 0
robotHasFallenFlag = 0
leftHandTouchedFlag = 0
rightHandTouchedFlag = 0
myGolf = None
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
        time.sleep(5)
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
    SENSORS.subscribe("HandLeftLeftTouched",500,0.0)
    redBallDetectMethod = 1

    try:
        #-----------------------------------------------------主死循环，待机状态-----------------------------------------------------#
        while(True):
            if(MOTION.robotIsWakeUp() == False):
                MOTION.wakeUp()
                POSTURE.goToPosture("StandInit", 0.5)
                TTS.say("游戏开始")
                releaseStick(robotIP, port)
                while(True):
                    temp =MEMORY.getData("HandLeftLeftTouched")
                    if(temp):
                        catchStick(robotIP, port)
                        break
                #让杆放在合适位置，准备移动
                raiseStick(robotIP, port)
                standWithStick(0.1, robotIP, port)
            time.sleep(1)
            print "holeFlag =",holeFlag
            if(holeFlag != 0):
                #---------------------------------------------次死循环，工作状态------------------------------------------------------#
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
                        global robotHasFallenFlag
                        robotHasFallenFlag = 0
                        break

                    MOTION.moveTo(0,0,redBallInfo[1],moveConfig)
                    time.sleep(1)
                    MOTION.moveTo(redBallInfo[0]-0.2,0,0,moveConfig)

                    if(holeFlag == 0 or robotHasFallenFlag == 1):
                        global robotHasFallenFlag
                        robotHasFallenFlag = 0
                        break

                    redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, redBallDetectMethod, 20, robotIP, port)
                    markInfo = naoMarkDetection(robotIP, port)
                    #-----------------------------------检测到 nao_mark 标志，近距离打球-----------------------------------------------#
                    if(markInfo != False):
                        theta,x,y = calculation(redBallInfo,markInfo)

                        #修正阈值，在场上需要根据实际情况调整
                        theta -= 0.1
                        x -= 0.30
                        y -= 0.05

                        robotRotate(theta, robotIP, port)
                        time.sleep(1.0)
                        MOTION.moveTo(x,0.0,0.0,moveConfig)
                        time.sleep(1.0)
                        MOTION.moveTo(0.0,y,0.0,moveConfig)
                        MOTION.angleInterpolation("HeadYaw",0,0.5,True)

                        if(holeFlag == 0 or robotHasFallenFlag == 1):
                            global robotHasFallenFlag
                            robotHasFallenFlag = 0
                            break
                        #-------------------------------------近距离第一个球洞--------------------------------------------------------#
                        if(holeFlag == 1):
                            MOTION.moveTo(0.0, 0.0, math.pi/2, moveConfig)
                            time.sleep(1)
                            MOTION.moveTo(-0.2, 0.0, 0.0, moveConfig)
                            time.sleep(1)
                            MOTION.moveTo(0.0, -0.2, 0.0, moveConfig)
                            hitBall(0.1, robotIP, port)
                            time.sleep(1)
                            MOTION.moveTo(0, 0, -math.pi/2, moveConfig)
                            time.sleep(1)
                        #--------------------------------------近距离第二个球洞-------------------------------------------------------#
                        elif(holeFlag == 2):
                            tempAngle = whiteBlockDetection(robotIP, port)
                            if(tempAngle):
                                angleHit = math.pi/2 - tempAngle
                                x = 0.2 * math.sin(tempAngle) - 0.20
                                y = 0.2 * math.cos(tempAngle) - 0.05
                                MOTION.moveTo(0.0, 0.0, angleHit, moveConfig)
                                time.sleep(1.0)
                                MOTION.moveTo(0.0, y, 0.0, moveConfig)
                                time.sleep(1.0)
                                MOTION.moveTo(x, 0.0, 0.0, moveConfig)
                                time.sleep(1.0)
                                hitBall(0.1, robotIP, port)
                                time.sleep(1.0)
                            else:
                                hitBall(0.1, robotIP, port)
                                time.sleep(1)
                        #--------------------------------------近距离第三个球洞---------------------------------------------------------#
                        elif(holeFlag == 3):
                            hitBall(0.1, robotIP, port)
                            time.sleep(1)
                    #-----------------------------没有检测到 nao_mark 标志，检测黄杆，远距离打球----------------------------------------#
                    else:
                        stickInfo = yellowStickDetection(yellow_thresholdMin, yellow_thresholdMax, robotIP, port)
                        theta,x,y = calculation(redBallInfo,stickInfo)
                        #修正阈值，在场上需要根据实际情况调整
                        theta -= 0.1
                        x -= 0.30
                        y -= 0.05
                        robotRotate(theta, robotIP, port)
                        time.sleep(1.0)
                        MOTION.moveTo(x,0.0,0.0,moveConfig)
                        time.sleep(1.0)
                        MOTION.moveTo(0.0,y,0.0,moveConfig)
                        MOTION.angleInterpolation("HeadYaw",0,0.5,True)

                        if(holeFlag == 0 or robotHasFallenFlag == 1):
                            global robotHasFallenFlag
                            robotHasFallenFlag = 0
                            break
                        #---------------------------------------远距离第一个球洞------------------------------------------------------#
                        if(holeFlag == 1):
                            MOTION.moveTo(0.0, 0.0, math.pi/2, moveConfig)
                            time.sleep(1)
                            MOTION.moveTo(-0.2, 0.0, 0.0, moveConfig)
                            time.sleep(1)
                            MOTION.moveTo(0.0, -0.2, 0.0, moveConfig)
                            time.sleep(1)
                            hitBall(0.1, robotIP, port)
                            time.sleep(1)
                            MOTION.moveTo(0, 0, -math.pi/2, moveConfig)
                            time.sleep(1)
                        #---------------------------------------远距离第二个球洞------------------------------------------------------#
                        elif(holeFlag == 2):
                            tempAngle = whiteBlockDetection(robotIP, port)
                            if(tempAngle):
                                angleHit = math.pi/2 - tempAngle
                                x = 0.2 * math.sin(tempAngle) - 0.20
                                y = 0.2 * math.cos(tempAngle) - 0.05
                                MOTION.moveTo(0.0, 0.0, angleHit, moveConfig)
                                time.sleep(1.0)
                                MOTION.moveTo(0.0, y, 0.0, moveConfig)
                                time.sleep(1.0)
                                MOTION.moveTo(x, 0.0, 0.0, moveConfig)
                                time.sleep(1.0)
                                hitBall(0.1, robotIP, port)
                                time.sleep(1.0)

                            else:
                                hitBall(0.1, robotIP, port)
                                time.sleep(1)
                        #---------------------------------------远距离第三个球洞------------------------------------------------------#
                        elif(holeFlag == 3):
                            tempAngle = whiteBorderDetection(robotIP, port)
                            if(tempAngle):
                                pass
                            else:
                                hitBall(0.1, robotIP, port)
                                time.sleep(1)

                        if(holeFlag == 0 or robotHasFallenFlag == 1):
                            global robotHasFallenFlag
                            robotHasFallenFlag = 0
                            break
                #------------------------------------------------次死循环结束--------------------------------------------------------#
        #--------------------------------------------------------主死循环结束--------------------------------------------------------#
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
