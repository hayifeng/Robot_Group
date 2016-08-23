#!/usr/bin/python2.7
# -*- encoding: UTF-8 -*-

import math
import time
import sys
import argparse
from naoqi import ALProxy

#-----------------------------------------------------机器人姿势定义---------------------------------------------------#
def postureReleaseStick():

    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1.08])
    keys.append([-0.00215657])

    names.append("HeadYaw")
    times.append([1.08])
    keys.append([0])

    names.append("LAnklePitch")
    times.append([1.08])
    keys.append([-0.358999])

    names.append("LAnkleRoll")
    times.append([1.08])
    keys.append([-0.00147909])

    names.append("LElbowRoll")
    times.append([1.08])
    keys.append([-0.537794])

    names.append("LElbowYaw")
    times.append([1.08])
    keys.append([-1.4488])

    names.append("LHand")
    times.append([1.08])
    keys.append([1])

    names.append("LHipPitch")
    times.append([1.08])
    keys.append([-0.444635])

    names.append("LHipRoll")
    times.append([1.08])
    keys.append([0.00744654])

    names.append("LHipYawPitch")
    times.append([1.08])
    keys.append([-0.00721956])

    names.append("LKneePitch")
    times.append([1.08])
    keys.append([0.696988])

    names.append("LShoulderPitch")
    times.append([1.08])
    keys.append([0.283386])

    names.append("LShoulderRoll")
    times.append([1.08])
    keys.append([-0.00171348])

    names.append("LWristYaw")
    times.append([1.08])
    keys.append([-0.0934926])

    names.append("RAnklePitch")
    times.append([1.08])
    keys.append([-0.357381])

    names.append("RAnkleRoll")
    times.append([1.08])
    keys.append([0.00361548])

    names.append("RElbowRoll")
    times.append([1.08])
    keys.append([0.546897])

    names.append("RElbowYaw")
    times.append([1.08])
    keys.append([1.39182])

    names.append("RHand")
    times.append([1.08])
    keys.append([0.0644])

    names.append("RHipPitch")
    times.append([1.08])
    keys.append([-0.45268])

    names.append("RHipRoll")
    times.append([1.08])
    keys.append([-0.00643682])

    names.append("RHipYawPitch")
    times.append([1.08])
    keys.append([-0.00721956])

    names.append("RKneePitch")
    times.append([1.08])
    keys.append([0.693492])

    names.append("RShoulderPitch")
    times.append([1.08])
    keys.append([1.45116])

    names.append("RShoulderRoll")
    times.append([1.08])
    keys.append([-0.252997])

    names.append("RWristYaw")
    times.append([1.08])
    keys.append([0.0160101])

    return [names,keys,times]
def positionCatchStick():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1.08])
    keys.append([-0.00215657])

    names.append("HeadYaw")
    times.append([1.08])
    keys.append([0])

    names.append("LAnklePitch")
    times.append([1.08])
    keys.append([-0.358999])

    names.append("LAnkleRoll")
    times.append([1.08])
    keys.append([-0.00147909])

    names.append("LElbowRoll")
    times.append([1.08])
    keys.append([-0.537794])

    names.append("LElbowYaw")
    times.append([1.08])
    keys.append([-1.4488])

    names.append("LHand")
    times.append([1.08])
    keys.append([0.0])

    names.append("LHipPitch")
    times.append([1.08])
    keys.append([-0.444635])

    names.append("LHipRoll")
    times.append([1.08])
    keys.append([0.00744654])

    names.append("LHipYawPitch")
    times.append([1.08])
    keys.append([-0.00721956])

    names.append("LKneePitch")
    times.append([1.08])
    keys.append([0.696988])

    names.append("LShoulderPitch")
    times.append([1.08])
    keys.append([0.283386])

    names.append("LShoulderRoll")
    times.append([1.08])
    keys.append([-0.00171348])

    names.append("LWristYaw")
    times.append([1.08])
    keys.append([-0.0934926])

    names.append("RAnklePitch")
    times.append([1.08])
    keys.append([-0.357381])

    names.append("RAnkleRoll")
    times.append([1.08])
    keys.append([0.00361548])

    names.append("RElbowRoll")
    times.append([1.08])
    keys.append([0.546897])

    names.append("RElbowYaw")
    times.append([1.08])
    keys.append([1.39182])

    names.append("RHand")
    times.append([1.08])
    keys.append([0.0644])

    names.append("RHipPitch")
    times.append([1.08])
    keys.append([-0.45268])

    names.append("RHipRoll")
    times.append([1.08])
    keys.append([-0.00643682])

    names.append("RHipYawPitch")
    times.append([1.08])
    keys.append([-0.00721956])

    names.append("RKneePitch")
    times.append([1.08])
    keys.append([0.693492])

    names.append("RShoulderPitch")
    times.append([1.08])
    keys.append([1.45116])

    names.append("RShoulderRoll")
    times.append([1.08])
    keys.append([-0.252997])

    names.append("RWristYaw")
    times.append([1.08])
    keys.append([0.0160101])
    return [names,keys,times]
def positionHitBall1():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1])
    keys.append([0.0])

    names.append("HeadYaw")
    times.append([1])
    keys.append([0.0])

    names.append("LAnklePitch")
    times.append([1])
    keys.append([-0.385075])

    names.append("LAnkleRoll")
    times.append([1])
    keys.append([4.19617e-05])

    names.append("LElbowRoll")
    times.append([1])
    keys.append([-1.1704])

    names.append("LElbowYaw")
    times.append([1])
    keys.append([-1.42666])

    names.append("LHand")
    times.append([1])
    keys.append([0])

    names.append("LHipPitch")
    times.append([1])
    keys.append([-0.454021])

    names.append("LHipRoll")
    times.append([1])
    keys.append([-0.00302602])

    names.append("LHipYawPitch")
    times.append([1])
    keys.append([-0.021434])

    names.append("LKneePitch")
    times.append([1])
    keys.append([0.682588])

    names.append("LShoulderPitch")
    times.append([1])
    keys.append([0.895815])

    names.append("LShoulderRoll")
    times.append([1])
    keys.append([-0.00157595])

    names.append("LWristYaw")
    times.append([0.8])
    keys.append([1.20602])

    names.append("RAnklePitch")
    times.append([1])
    keys.append([-0.397265])

    names.append("RAnkleRoll")
    times.append([1])
    keys.append([0.00157595])

    names.append("RElbowRoll")
    times.append([1])
    keys.append([0.530805])

    names.append("RElbowYaw")
    times.append([1])
    keys.append([1.38976])

    names.append("RHand")
    times.append([1])
    keys.append([0.0512])

    names.append("RHipPitch")
    times.append([1])
    keys.append([-0.466378])

    names.append("RHipRoll")
    times.append([1])
    keys.append([0.00310993])

    names.append("RHipYawPitch")
    times.append([1])
    keys.append([-0.021434])

    names.append("RKneePitch")
    times.append([1])
    keys.append([0.694945])

    names.append("RShoulderPitch")
    times.append([1])
    keys.append([1.47728])

    names.append("RShoulderRoll")
    times.append([1])
    keys.append([-0.224006])

    names.append("RWristYaw")
    times.append([1])
    keys.append([0.030638])

    return [names,keys,times]
def positionHitBall(speedTime):

    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1.5, 3])
    keys.append([0, 0])

    names.append("HeadYaw")
    times.append([1.5, 3])
    keys.append([0, 0])

    names.append("LAnklePitch")
    times.append([1.5, 3])
    keys.append([-0.358999, -0.358999])

    names.append("LAnkleRoll")
    times.append([1.5, 3])
    keys.append([0, 0])

    names.append("LElbowRoll")
    times.append([1.5, 3])
    keys.append([-0.545276, -0.545276])

    names.append("LElbowYaw")
    times.append([1.5, 3])
    keys.append([-1.44257, -1.44257])

    names.append("LHand")
    times.append([1.5, 3])
    keys.append([0.05, 0.08])

    names.append("LHipPitch")
    times.append([1.5, 3])
    keys.append([-0.446352, -0.446352])

    names.append("LHipRoll")
    times.append([1.5, 3])
    keys.append([0, 0])

    names.append("LHipYawPitch")
    times.append([1.5, 3])
    keys.append([0, 0])

    names.append("LKneePitch")
    times.append([1.5, 3])
    keys.append([0.704064, 0.704064])

    names.append("LShoulderPitch")
    times.append([1.5, 3])
    keys.append([0.286009, 0.286009])

    names.append("LShoulderRoll")
    times.append([1.5, 3])
    keys.append([1.32042, -0.00551104])

    names.append("LWristYaw")
    times.append([1.5, 3])
    keys.append([1.07408, 0.524645])

    names.append("RAnklePitch")
    times.append([1.5, 3])
    keys.append([-0.357381, -0.357381])

    names.append("RAnkleRoll")
    times.append([1.5, 3])
    keys.append([0, 0])

    names.append("RElbowRoll")
    times.append([1.5, 3])
    keys.append([0.548729, 0.548729])

    names.append("RElbowYaw")
    times.append([1.5, 3])
    keys.append([1.39238, 1.39238])

    names.append("RHand")
    times.append([1.5, 3])
    keys.append([0.0644, 0.0644])

    names.append("RHipPitch")
    times.append([1.5, 3])
    keys.append([-0.455641, -0.455641])

    names.append("RHipRoll")
    times.append([1.5, 3])
    keys.append([0, 0])

    names.append("RHipYawPitch")
    times.append([1.5, 3])
    keys.append([0, 0])

    names.append("RKneePitch")
    times.append([1.5, 3])
    keys.append([0.699545, 0.699545])

    names.append("RShoulderPitch")
    times.append([1.5, 3])
    keys.append([1.44445, 1.44445])

    names.append("RShoulderRoll")
    times.append([1.5, 3])
    keys.append([-0.271401, -0.25895])

    names.append("RWristYaw")
    times.append([1.5, 3])
    keys.append([0.0152981, 0.0152981])


    return [names,keys,times]
def positionStandWithStick():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([2])
    keys.append([-0.00215657])

    names.append("HeadYaw")
    times.append([2])
    keys.append([0])

    names.append("LAnklePitch")
    times.append([2])
    keys.append([-0.358999])

    names.append("LAnkleRoll")
    times.append([2])
    keys.append([-0.00147909])

    names.append("LElbowRoll")
    times.append([2])
    keys.append([-0.0701831])

    names.append("LElbowYaw")
    times.append([2])
    keys.append([-0.00257829])

    names.append("LHand")
    times.append([2])
    keys.append([0.000842703])

    names.append("LHipPitch")
    times.append([2])
    keys.append([-0.444635])

    names.append("LHipRoll")
    times.append([2])
    keys.append([0.00744654])

    names.append("LHipYawPitch")
    times.append([2])
    keys.append([-0.00721956])

    names.append("LKneePitch")
    times.append([2])
    keys.append([0.696988])

    names.append("LShoulderPitch")
    times.append([2])
    keys.append([1.80089])

    names.append("LShoulderRoll")
    times.append([2])
    keys.append([0.38314])

    names.append("LWristYaw")
    times.append([2])
    keys.append([-1.62353])

    names.append("RAnklePitch")
    times.append([2])
    keys.append([-0.368008])

    names.append("RAnkleRoll")
    times.append([2])
    keys.append([0.00361548])

    names.append("RElbowRoll")
    times.append([2])
    keys.append([0.546897])

    names.append("RElbowYaw")
    times.append([2])
    keys.append([1.39182])

    names.append("RHand")
    times.append([2])
    keys.append([0.0644])

    names.append("RHipPitch")
    times.append([2])
    keys.append([-0.45268])

    names.append("RHipRoll")
    times.append([2])
    keys.append([-0.00643682])

    names.append("RHipYawPitch")
    times.append([2])
    keys.append([-0.00721956])

    names.append("RKneePitch")
    times.append([2])
    keys.append([0.693492])

    names.append("RShoulderPitch")
    times.append([2])
    keys.append([1.44085])

    names.append("RShoulderRoll")
    times.append([2])
    keys.append([-0.262521])

    names.append("RWristYaw")
    times.append([2])
    keys.append([0.0160101])

    return [names, keys, times]
def positionRaiseStick():

    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1.08, 1.68])
    keys.append([-0.00215657, -0.00215657])

    names.append("HeadYaw")
    times.append([1.08, 1.68])
    keys.append([0, 0])

    names.append("LAnklePitch")
    times.append([1.08, 1.68])
    keys.append([-0.358999, -0.358999])

    names.append("LAnkleRoll")
    times.append([1.08, 1.68])
    keys.append([-0.00147909, -0.00147909])

    names.append("LElbowRoll")
    times.append([1.08, 1.68])
    keys.append([-0.537794, -0.537794])

    names.append("LElbowYaw")
    times.append([1.08, 1.68])
    keys.append([-1.4488, -1.4488])

    names.append("LHand")
    times.append([1.08, 1.68])
    keys.append([0.0687508, 0.16])

    names.append("LHipPitch")
    times.append([1.08, 1.68])
    keys.append([-0.444635, -0.444635])

    names.append("LHipRoll")
    times.append([1.08, 1.68])
    keys.append([0.00744654, 0.00744654])

    names.append("LHipYawPitch")
    times.append([1.08, 1.68])
    keys.append([-0.00721956, -0.00721956])

    names.append("LKneePitch")
    times.append([1.08, 1.68])
    keys.append([0.696988, 0.696988])

    names.append("LShoulderPitch")
    times.append([1.08, 1.68])
    keys.append([0.283386, 0.283386])

    names.append("LShoulderRoll")
    times.append([1.08, 1.68])
    keys.append([-0.000886967, 1.32045])

    names.append("LWristYaw")
    times.append([1.08, 1.68])
    keys.append([1.65869, 1.07807])

    names.append("RAnklePitch")
    times.append([1.08, 1.68])
    keys.append([-0.357381, -0.357381])

    names.append("RAnkleRoll")
    times.append([1.08, 1.68])
    keys.append([0.00361548, 0.00361548])

    names.append("RElbowRoll")
    times.append([1.08, 1.68])
    keys.append([0.546897, 0.546897])

    names.append("RElbowYaw")
    times.append([1.08, 1.68])
    keys.append([1.39182, 1.39182])

    names.append("RHand")
    times.append([1.08, 1.68])
    keys.append([0.0644, 0.0644])

    names.append("RHipPitch")
    times.append([1.08, 1.68])
    keys.append([-0.45268, -0.45268])

    names.append("RHipRoll")
    times.append([1.08, 1.68])
    keys.append([-0.00643682, -0.00643682])

    names.append("RHipYawPitch")
    times.append([1.08, 1.68])
    keys.append([-0.00721956, -0.00721956])

    names.append("RKneePitch")
    times.append([1.08, 1.68])
    keys.append([0.693492, 0.693492])

    names.append("RShoulderPitch")
    times.append([1.08, 1.68])
    keys.append([1.44085, 1.44085])

    names.append("RShoulderRoll")
    times.append([1.08, 1.68])
    keys.append([-0.265356, -0.276408])

    names.append("RWristYaw")
    times.append([1.08, 1.68])
    keys.append([0.0160101, 0.0160101])
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
    #打球后需要旋转90°面向击球方向

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
    parser.add_argument("--MaxStepFrequency", type=float, default=0.9,
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

    moveConfig = [["MaxStepX",args.MaxStepX],["MaxStepY",args.MaxStepY],["MaxStepTheta",args.MaxStepTheta],["MaxStepFrequency",args.MaxStepFrequency],
                    ["StepHeight",args.StepHeight],["TorsoWx",args.TorsoWx],["TorsoWy",args.TorsoWy]]

    robotIP = args.ip
    port = args.port
    #catchStick(robotIP, port)

    #raiseStick(robotIP, port)
    #time.sleep(2)
    #standWithStick(0.1, robotIP, port)
    #time.sleep(2)
    #raiseStick(robotIP, port)
    #time.sleep(2)
    #hitBall(0.1, robotIP, port)
    #MOTION = ALProxy("ALMotion", robotIP, port)
