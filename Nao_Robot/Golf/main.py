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
from naoqi import ALModule
from goto import goto, comefrom, label
from detection import redBallDetection,naoMarkDetection,yellowStickDetection,whiteBlockDetection
from movement import catchStick,hitBall,standWithStick,raiseStick

#------------------------------------------------全局变量定义-------------------------------------------------------------#
holeFlag = 0
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
    def __init__(self, name):
        ALModule.__init__(self, name):
        self.memory = ALProxy("ALMemory")
        self.memory.subscribeToEvent("ChestButtonPressed", "Golf", "chestButtonPressed")
        self.memory.subscribeToEvent("FrontTactilTouched", "Golf", "frontTactilTouched")
        self.memory.subscribeToEvent("MiddleTactilTouched", "Golf", "middleTactilTouched")
        self.memory.subscribeToEvent("RearTactilTouched", "Golf", "rearTactilTouched")

    def frontTactilTouched(self):
        self.memory.unsubscribeToEvent("FrontTactilTouched", "Golf")
        holeFlag = 1
        self.memory.subscribeToEvent("FrontTactilTouched", "Golf", "frontTactilTouched")
        goto .restart
    def middleTactilTouched(self):
        self.memory.unsubscribeToEvent("MiddleTactilTouched", "Golf")
        holeFlag = 2
        self.memory.subscribeToEvent("MiddleTactilTouched", "Golf", "middleTactilTouched")
        goto .restart
    def rearTactilTouched(self):
        self.memory.unsubscribeToEvent("RearTactilTouched", "Golf")
        holeFlag = 3
        self.memory.subscribeToEvent("RearTactilTouched", "Golf", "rearTactilTouched")
        goto .restart
    def chestButtonPressed(self):
        self.memory.unsubscribeToEvent("ChestButtonPressed", "Golf")
        holeFlag = 0
        self.memory.subscribeToEvent("ChestButtonPressed", "Golf", "chestButtonPressed")
        goto .restart
    def fallDownDetected(self):
        pass

#----------------------------------------------------主函数------------------------------------------------------------#
def main(robotIP, port):

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
    Golf = Golf("Golf")

    redBallDetectMethod = 1

    #初始化机器人电机
    MOTION.wakeUp()
    #让机器人保持一个便于运动的姿势
    POSTURE.goToPosture("StandInit", 0.5)
    TTS.say("游戏开始")
    catchStick(robotIP, port)
    try:
        while(True):
            label .restart
            #第一个球洞
            if(holeFlag == 1):
                while(True):
                    #让杆放在合适位置，准备移动
                    raiseStick(robotIP, port)
                    standWithStick(0.1, robotIP, port)
                    #检测红球
                    redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, redBallDetectMethod, 0, robotIP, port)

                    #第一次,机器人转到正对球的方向
                    TTS.say("第一次移动")
                    MOTION.setMoveArmsEnabled(False, False)
                    MOTION.moveTo(0,0,redBallInfo[1],moveConfig)
                    time.sleep(1)

                    #第二次,机器人走到距离球20厘米的位置
                    TTS.say("第二次移动")
                    MOTION.setMoveArmsEnabled(False, False)
                    MOTION.moveTo(redBallInfo[0]-0.3,0,0,moveConfig)

                    MOTION.waitUntilMoveIsFinished()

                    #第三次,机器人再次对准球,进行修正
                    TTS.say("第三次移动")
                    redBallInfo = redBallDetection(red_thresholdMin, red_thresholdMax, redBallDetectMethod, 20, robotIP, port)
                    MOTION.setMoveArmsEnabled(False, False)
                    MOTION.moveTo(0,0,redBallInfo[1],moveConfig)

                    time.sleep(1)

                    #第四次,机器人走到离球10厘米的位置
                    TTS.say("第四次移动")
                    MOTION.setMoveArmsEnabled(False, False)
                    MOTION.moveTo(redBallInfo[0]-0.2,0,0,moveConfig)

                    TTS.say("移动完毕")
                    time.sleep(1)
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
                        MOTION.setMoveArmsEnabled(False, False)
                        MOTION.moveTo(0.0,y,0.0,moveConfig)

                        MOTION.angleInterpolation("HeadYaw",0,0.5,True)
                        #击球
                        hitBall(0.1, robotIP, port)
                        time.sleep(3)
                    #没有检测到mark标志，检测黄杆
                    else:
                        stickInfo = yellowStickDetection(yellow_thresholdMin, yellow_thresholdMax, robotIP, port)
                        #移动机器人正对黄杆
                        theta,x,y = calculation(redBallInfo,stickInfo)
                        #先移动角度
                        robotRotate(theta, robotIP, port)
                        time.sleep(1.0)
                        #修正阈值，在场上需要根据实际情况调整
                        x -= 0.25
                        y -= 0.05

                        MOTION.moveTo(x,0.0,0.0,moveConfig)
                        MOTION.setMoveArmsEnabled(False, False)
                        MOTION.moveTo(0.0,y,0.0,moveConfig)

                        MOTION.angleInterpolation("HeadYaw",0,0.5,True)
                        #击球
                        hitBall(0.1, robotIP, port)
                        time.sleep(3)

            #第二个球洞
            if(holeFlag == 2):
                while(True):
                    pass
            #第三个球洞
            if(holeFlag == 3):
                while(True):
                    pass
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
