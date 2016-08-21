#!/usr/bin/python2.7
# -*- encoding: UTF-8 -*-

import math
import time
import sys
import argparse
from naoqi import ALProxy

#-----------------------------------------------------机器人姿势定义---------------------------------------------------#
def positionReleaseStick():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1.2])
    keys.append([0.0475121])

    names.append("HeadYaw")
    times.append([1.2])
    keys.append([0.0106959])

    names.append("LAnklePitch")
    times.append([1.2])
    keys.append([-0.366668])

    names.append("LAnkleRoll")
    times.append([1.2])
    keys.append([0.00157595])

    names.append("LElbowRoll")
    times.append([1.2])
    keys.append([-1.20597])

    names.append("LElbowYaw")
    times.append([1.2])
    keys.append([-1.45369])

    names.append("LHand")
    times.append([1.2])
    keys.append([0.85])

    names.append("LHipPitch")
    times.append([1.2])
    keys.append([-0.452488])

    names.append("LHipRoll")
    times.append([1.2])
    keys.append([0.0061779])

    names.append("LHipYawPitch")
    times.append([1.2])
    keys.append([-0.026036])

    names.append("LKneePitch")
    times.append([1.2])
    keys.append([0.682588])

    names.append("LShoulderPitch")
    times.append([1.2])
    keys.append([0.816046])

    names.append("LShoulderRoll")
    times.append([1.2])
    keys.append([-0.01845])

    names.append("LWristYaw")
    times.append([1.2])
    keys.append([-0.0859461])

    names.append("RAnklePitch")
    times.append([1.2])
    keys.append([-0.392662])

    names.append("RAnkleRoll")
    times.append([1.2])
    keys.append([0.00771189])

    names.append("RElbowRoll")
    times.append([1.2])
    keys.append([0.556884])

    names.append("RElbowYaw")
    times.append([1.2])
    keys.append([1.3959])

    names.append("RHand")
    times.append([1.2])
    keys.append([0.0612])

    names.append("RHipPitch")
    times.append([1.2])
    keys.append([-0.458708])

    names.append("RHipRoll")
    times.append([1.2])
    keys.append([-0.00302601])

    names.append("RHipYawPitch")
    times.append([1.2])
    keys.append([-0.026036])

    names.append("RKneePitch")
    times.append([1.2])
    keys.append([0.696478])

    names.append("RShoulderPitch")
    times.append([1.2])
    keys.append([1.46041])

    names.append("RShoulderRoll")
    times.append([1.2])
    keys.append([-0.200996])

    names.append("RWristYaw")
    times.append([1.2])
    keys.append([0.0429101])
    return [names,keys,times]
def positionCatchStick():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1.2])
    keys.append([0.0475121])

    names.append("HeadYaw")
    times.append([1.2])
    keys.append([0.0106959])

    names.append("LAnklePitch")
    times.append([1.2])
    keys.append([-0.366668])

    names.append("LAnkleRoll")
    times.append([1.2])
    keys.append([0.00157595])

    names.append("LElbowRoll")
    times.append([1.2])
    keys.append([-1.20597])

    names.append("LElbowYaw")
    times.append([1.2])
    keys.append([-1.45369])

    names.append("LHand")
    times.append([1.2])
    keys.append([0.2864])

    names.append("LHipPitch")
    times.append([1.2])
    keys.append([-0.452488])

    names.append("LHipRoll")
    times.append([1.2])
    keys.append([0.0061779])

    names.append("LHipYawPitch")
    times.append([1.2])
    keys.append([-0.026036])

    names.append("LKneePitch")
    times.append([1.2])
    keys.append([0.682588])

    names.append("LShoulderPitch")
    times.append([1.2])
    keys.append([0.826784])

    names.append("LShoulderRoll")
    times.append([1.2])
    keys.append([-0.01845])

    names.append("LWristYaw")
    times.append([1.2])
    keys.append([-0.098218])

    names.append("RAnklePitch")
    times.append([1.2])
    keys.append([-0.392662])

    names.append("RAnkleRoll")
    times.append([1.2])
    keys.append([0.00771189])

    names.append("RElbowRoll")
    times.append([1.2])
    keys.append([0.556884])

    names.append("RElbowYaw")
    times.append([1.2])
    keys.append([1.3959])

    names.append("RHand")
    times.append([1.2])
    keys.append([0.0612])

    names.append("RHipPitch")
    times.append([1.2])
    keys.append([-0.458708])

    names.append("RHipRoll")
    times.append([1.2])
    keys.append([-0.00302601])

    names.append("RHipYawPitch")
    times.append([1.2])
    keys.append([-0.026036])

    names.append("RKneePitch")
    times.append([1.2])
    keys.append([0.696478])

    names.append("RShoulderPitch")
    times.append([1.2])
    keys.append([1.46041])

    names.append("RShoulderRoll")
    times.append([1.2])
    keys.append([-0.200996])

    names.append("RWristYaw")
    times.append([1.2])
    keys.append([0.0429101])
    return [names,keys,times]
def positionHitBall1():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([0.8])
    keys.append([0.0383081])

    names.append("HeadYaw")
    times.append([0.8])
    keys.append([0.00609398])

    names.append("LAnklePitch")
    times.append([0.8])
    keys.append([-0.354396])

    names.append("LAnkleRoll")
    times.append([0.8])
    keys.append([0.00310993])

    names.append("LElbowRoll")
    times.append([0.8])
    keys.append([-1.204651])

    names.append("LElbowYaw")
    times.append([0.8])
    keys.append([-1.46837])

    names.append("LHand")
    times.append([0.8])
    keys.append([0.2728])

    names.append("LHipPitch")
    times.append([0.8])
    keys.append([-0.443284])

    names.append("LHipRoll")
    times.append([0.8])
    keys.append([-0.00455999])

    names.append("LHipYawPitch")
    times.append([0.8])
    keys.append([-0.0137641])

    names.append("LKneePitch")
    times.append([0.8])
    keys.append([0.68719])

    names.append("LShoulderPitch")
    times.append([0.8])
    keys.append([0.7102])

    names.append("LShoulderRoll")
    times.append([0.8])
    keys.append([-0.0107799])

    names.append("LWristYaw")
    times.append([0.8])
    keys.append([1.20602])

    names.append("RAnklePitch")
    times.append([0.8])
    keys.append([-0.369652])

    names.append("RAnkleRoll")
    times.append([0.8])
    keys.append([0.00464392])

    names.append("RElbowRoll")
    times.append([0.8])
    keys.append([0.599836])

    names.append("RElbowYaw")
    times.append([0.8])
    keys.append([1.40817])

    names.append("RHand")
    times.append([0.8])
    keys.append([0.0328])

    names.append("RHipPitch")
    times.append([0.8])
    keys.append([-0.457174])

    names.append("RHipRoll")
    times.append([0.8])
    keys.append([4.19617e-05])

    names.append("RHipYawPitch")
    times.append([0.8])
    keys.append([-0.0137641])

    names.append("RKneePitch")
    times.append([0.8])
    keys.append([0.699546])

    names.append("RShoulderPitch")
    times.append([0.8])
    keys.append([1.48495])

    names.append("RShoulderRoll")
    times.append([0.8])
    keys.append([-0.17185])

    names.append("RWristYaw")
    times.append([0.8])
    keys.append([0.0106959])

    return [names, keys, times]
def positionHitBall2(speedTime):
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([0.8])
    keys.append([0.032172])

    names.append("HeadYaw")
    times.append([0.8])
    keys.append([0.00609398])

    names.append("LAnklePitch")
    times.append([0.8])
    keys.append([-0.369736])

    names.append("LAnkleRoll")
    times.append([0.8])
    keys.append([-0.00302601])

    names.append("LElbowRoll")
    times.append([0.8])
    keys.append([-1.20455])

    names.append("LElbowYaw")
    times.append([0.8])
    keys.append([-1.4515])

    names.append("LHand")
    times.append([0.8])
    keys.append([0.2804])

    names.append("LHipPitch")
    times.append([0.8])
    keys.append([-0.44942])

    names.append("LHipRoll")
    times.append([0.8])
    keys.append([0.00310993])

    names.append("LHipYawPitch")
    times.append([0.8])
    keys.append([-0.030638])

    names.append("LKneePitch")
    times.append([0.8])
    keys.append([0.685656])

    names.append("LShoulderPitch")
    times.append([0.8])
    keys.append([0.780764])

    names.append("LShoulderRoll")
    times.append([0.8])
    keys.append([-0.0138481])

    names.append("LWristYaw")
    times.append([speedTime])
    keys.append([-0.212398])

    names.append("RAnklePitch")
    times.append([0.8])
    keys.append([-0.38039])

    names.append("RAnkleRoll")
    times.append([0.8])
    keys.append([0.0123138])

    names.append("RElbowRoll")
    times.append([0.8])
    keys.append([0.57836])

    names.append("RElbowYaw")
    times.append([0.8])
    keys.append([1.39743])

    names.append("RHand")
    times.append([0.8])
    keys.append([0.0532])

    names.append("RHipPitch")
    times.append([0.8])
    keys.append([-0.474048])

    names.append("RHipRoll")
    times.append([0.8])
    keys.append([-0.00762796])

    names.append("RHipYawPitch")
    times.append([0.8])
    keys.append([-0.030638])

    names.append("RKneePitch")
    times.append([0.8])
    keys.append([0.698012])

    names.append("RShoulderPitch")
    times.append([0.8])
    keys.append([1.47575])

    names.append("RShoulderRoll")
    times.append([0.8])
    keys.append([-0.182588])

    names.append("RWristYaw")
    times.append([0.8])
    keys.append([0.030638])
    return [names, keys, times]
def positionStandWithStick():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1.2])
    keys.append([0.032172])

    names.append("HeadYaw")
    times.append([1.2])
    keys.append([-0.00464392])

    names.append("LAnklePitch")
    times.append([1.2])
    keys.append([-0.0061779])

    names.append("LAnkleRoll")
    times.append([1.2])
    keys.append([0.0061779])

    names.append("LElbowRoll")
    times.append([1.2])
    keys.append([-0.053648])

    names.append("LElbowYaw")
    times.append([1.2])
    keys.append([-1.37144])

    names.append("LHand")
    times.append([1.2])
    keys.append([0.288])

    names.append("LHipPitch")
    times.append([1.2])
    keys.append([-0.00302601])

    names.append("LHipRoll")
    times.append([1.2])
    keys.append([-0.00149202])

    names.append("LHipYawPitch")
    times.append([1.2])
    keys.append([4.19617e-05])

    names.append("LKneePitch")
    times.append([1.2])
    keys.append([0.00302601])

    names.append("LShoulderPitch")
    times.append([1.2])
    keys.append([1.56771])

    names.append("LShoulderRoll")
    times.append([1.2])
    keys.append([0.317496])

    names.append("LWristYaw")
    times.append([1.2])
    keys.append([-0.131966])

    names.append("RAnklePitch")
    times.append([1.2])
    keys.append([4.19617e-05])

    names.append("RAnkleRoll")
    times.append([1.2])
    keys.append([-0.00455999])

    names.append("RElbowRoll")
    times.append([1.2])
    keys.append([0.0583339])

    names.append("RElbowYaw")
    times.append([1.2])
    keys.append([1.33147])

    names.append("RHand")
    times.append([1.2])
    keys.append([0.0216])

    names.append("RHipPitch")
    times.append([1.2])
    keys.append([-0.0061779])

    names.append("RHipRoll")
    times.append([1.2])
    keys.append([-0.00916195])

    names.append("RHipYawPitch")
    times.append([1.2])
    keys.append([4.19617e-05])

    names.append("RKneePitch")
    times.append([1.2])
    keys.append([4.19617e-05])

    names.append("RShoulderPitch")
    times.append([1.2])
    keys.append([1.60614])

    names.append("RShoulderRoll")
    times.append([1.2])
    keys.append([-0.297638])

    names.append("RWristYaw")
    times.append([1.2])
    keys.append([0.0106959])

    return [names, keys, times]
def positionRaiseStick():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([0.8])
    keys.append([0.049046])

    names.append("HeadYaw")
    times.append([0.8])
    keys.append([0.00609398])

    names.append("LAnklePitch")
    times.append([0.8])
    keys.append([-0.366668])

    names.append("LAnkleRoll")
    times.append([0.8])
    keys.append([0.00157595])

    names.append("LElbowRoll")
    times.append([0.8])
    keys.append([-1.03694])

    names.append("LElbowYaw")
    times.append([0.8])
    keys.append([-1.3699])

    names.append("LHand")
    times.append([0.8])
    keys.append([0.2872])

    names.append("LHipPitch")
    times.append([0.8])
    keys.append([-0.446352])

    names.append("LHipRoll")
    times.append([0.8])
    keys.append([-0.00302601])

    names.append("LHipYawPitch")
    times.append([0.8])
    keys.append([-0.0199001])

    names.append("LKneePitch")
    times.append([0.8])
    keys.append([0.688724])

    names.append("LShoulderPitch")
    times.append([0.8])
    keys.append([0.262272])

    names.append("LShoulderRoll")
    times.append([0.8])
    keys.append([0.366584])

    names.append("LWristYaw")
    times.append([0.8])
    keys.append([0.45709])

    names.append("RAnklePitch")
    times.append([0.8])
    keys.append([-0.389594])

    names.append("RAnkleRoll")
    times.append([0.8])
    keys.append([0.00310993])

    names.append("RElbowRoll")
    times.append([0.8])
    keys.append([0.579894])

    names.append("RElbowYaw")
    times.append([0.8])
    keys.append([1.39743])

    names.append("RHand")
    times.append([0.8])
    keys.append([0.0484])

    names.append("RHipPitch")
    times.append([0.8])
    keys.append([-0.46331])

    names.append("RHipRoll")
    times.append([0.8])
    keys.append([4.19617e-05])

    names.append("RHipYawPitch")
    times.append([0.8])
    keys.append([-0.0199001])

    names.append("RKneePitch")
    times.append([0.8])
    keys.append([0.699546])

    names.append("RShoulderPitch")
    times.append([0.8])
    keys.append([1.47575])

    names.append("RShoulderRoll")
    times.append([0.8])
    keys.append([-0.184122])

    names.append("RWristYaw")
    times.append([0.8])
    keys.append([0.024502])
    return [names, keys, times]
#---------------------------------------------------机器人姿势定义结束--------------------------------------------------#

#---------------------------------------------------------------------------------------------------------------------#
#*********************************************************************************************************************
#@函数名：   raiseStick()
#@参数：    robotIP - 机器人IP地址
#           port - 机器人端口
#@返回值：   无
#@功能说明： 机器人放杆过程中的中间值
#@最后修改日期：2016-8-6
#*********************************************************************************************************************
def raiseStick(robotIP="127.0.0.1", port=9559):
    MOTION = ALProxy("ALMotion", robotIP, port)
    names, keys, times = positionRaiseStick()
    MOTION.angleInterpolation(names, keys, times, True)

#---------------------------------------------------------------------------------------------------------------------#
#*********************************************************************************************************************
#@函数名：   standWithStick()
#@参数：    speed - 站立速度，目前保留
#           robotIP - 机器人IP地址
#           port - 机器人端口
#@返回值：   无
#@功能说明： 机器人在移动过程中以及识别物体时的站立姿势
#@最后修改日期：2016-8-6
#*********************************************************************************************************************
def standWithStick(speed=0.1, robotIP="127.0.0.1", port=9559):
    speed = speed
    MOTION = ALProxy("ALMotion", robotIP, port)
    names, keys, times = positionStandWithStick()
    MOTION.angleInterpolation(names, keys, times, True)

#---------------------------------------------------------------------------------------------------------------------#
#*********************************************************************************************************************
#@函数名：   catchStick()
#@参数：     robotIP - 机器人IP地址
#           port - 机器人端口
#@返回值：   无
#@功能说明： 最开始的抓杆动作，左手抓杆。触摸手臂即触发机器人抓杆。
#@最后修改日期：2016-8-6
#*********************************************************************************************************************
def catchStick(robotIP="127.0.0.1", port=9559):
    MOTION = ALProxy("ALMotion", robotIP, port)
    names, keys, times = positionCatchStick()
    MOTION.angleInterpolation(names, keys, times, True)


def releaseStick(robotIP="127.0.0.1", port=9559):
    MOTION = ALProxy("ALMotion", robotIP, port)
    names, keys, times = positionReleaseStick()
    MOTION.angleInterpolation(names, keys, times, True)

#---------------------------------------------------------------------------------------------------------------------#
#*********************************************************************************************************************
#@函数名：   hitBall()
#@参数：     hitBallSpeed - 击球力度
#           robotIP - 机器人IP地址
#           port - 机器人端口
#@返回值：   无
#@功能说明： 机器人最终的击球动作。
#           目前击球力度是手动设置，以后需要根据距离值计算出击球力度。
#@最后修改日期：2016-8-6
#*********************************************************************************************************************
def hitBall(hitBallSpeed, robotIP="127.0.0.1", port=9559):
    MOTION = ALProxy("ALMotion", robotIP, port)
    names, keys, times = positionHitBall1()
    MOTION.angleInterpolation(names, keys, times, True)
    time.sleep(2)
    names, keys, times = positionHitBall2(hitBallSpeed)
    MOTION.angleInterpolation(names, keys, times, True)
    time.sleep(1)
    raiseStick(robotIP, port)
    time.sleep(1)
    standWithStick(0.1, robotIP, port)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number.")

    args = parser.parse_args()

    robotIP = args.ip
    port = args.port
    #catchStick(robotIP, port)
    #time.sleep(2)
    hitBall(0.1, robotIP, port)
    #time.sleep(2)
    #raiseStick(robotIP, port)
    #time.sleep(2)
    #standWithStick(0.1, robotIP, port)
