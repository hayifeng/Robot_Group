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
    times.append([0.48])
    keys.append([0])

    names.append("HeadYaw")
    times.append([0.48])
    keys.append([0])

    names.append("LAnklePitch")
    times.append([0.48])
    keys.append([-0.351406])

    names.append("LAnkleRoll")
    times.append([0.48])
    keys.append([0])

    names.append("LElbowRoll")
    times.append([0.48])
    keys.append([-0.517034])

    names.append("LElbowYaw")
    times.append([0.48])
    keys.append([-1.45072])

    names.append("LHand")
    times.append([0.48])
    keys.append([0.0])

    names.append("LHipPitch")
    times.append([0.48])
    keys.append([-0.444635])

    names.append("LHipRoll")
    times.append([0.48])
    keys.append([0])

    names.append("LHipYawPitch")
    times.append([0.48])
    keys.append([0])

    names.append("LKneePitch")
    times.append([0.48])
    keys.append([0.704064])

    names.append("LShoulderPitch")
    times.append([0.48])
    keys.append([0.385181])

    names.append("LShoulderRoll")
    times.append([0.48])
    keys.append([-0.00935468])

    names.append("LWristYaw")
    times.append([0.48])
    keys.append([-0.077322])

    names.append("RAnklePitch")
    times.append([0.48])
    keys.append([-0.350177])

    names.append("RAnkleRoll")
    times.append([0.48])
    keys.append([0])

    names.append("RElbowRoll")
    times.append([0.48])
    keys.append([0.532616])

    names.append("RElbowYaw")
    times.append([0.48])
    keys.append([1.39879])

    names.append("RHand")
    times.append([0.48])
    keys.append([0.0590473])

    names.append("RHipPitch")
    times.append([0.48])
    keys.append([-0.45268])

    names.append("RHipRoll")
    times.append([0.48])
    keys.append([0])

    names.append("RHipYawPitch")
    times.append([0.48])
    keys.append([0])

    names.append("RKneePitch")
    times.append([0.48])
    keys.append([0.699545])

    names.append("RShoulderPitch")
    times.append([0.48])
    keys.append([1.41542])

    names.append("RShoulderRoll")
    times.append([0.48])
    keys.append([-0.276462])

    names.append("RWristYaw")
    times.append([0.48])
    keys.append([0.0384861])
    return [names,keys,times]
def positionHitBall():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1.5, 3.0])
    keys.append([-0.00215657, -0.00215657])

    names.append("HeadYaw")
    times.append([1.5, 3.0])
    keys.append([0, 0])

    names.append("LAnklePitch")
    times.append([1.5, 3.0])
    keys.append([-0.351406, -0.351406])

    names.append("LAnkleRoll")
    times.append([1.5, 3.0])
    keys.append([-0.00147909, -0.00147909])

    names.append("LElbowRoll")
    times.append([1.5, 3.0])
    keys.append([-0.541034, -0.541034])

    names.append("LElbowYaw")
    times.append([1.5, 3.0])
    keys.append([-1.45072, -1.45072])

    names.append("LHand")
    times.append([1.5, 3.0])
    keys.append([0.0, 0.0])

    names.append("LHipPitch")
    times.append([1.5, 3.0])
    keys.append([-0.444635, -0.444635])

    names.append("LHipRoll")
    times.append([1.5, 3.0])
    keys.append([0.00744654, 0.00744654])

    names.append("LHipYawPitch")
    times.append([1.5, 3.0])
    keys.append([-0.00721956, -0.00721956])

    names.append("LKneePitch")
    times.append([1.5, 3.0])
    keys.append([0.704064, 0.704064])

    names.append("LShoulderPitch")
    times.append([1.5, 3.0])
    keys.append([0.284888, 0.363146])

    names.append("LShoulderRoll")
    times.append([1.5, 3.0])
    keys.append([1.32389, -3.01811e-05])

    names.append("LWristYaw")
    times.append([1.5, 3.0])
    keys.append([1.07199, 0.527726])

    names.append("RAnklePitch")
    times.append([1.5, 3.0])
    keys.append([-0.350177, -0.350177])

    names.append("RAnkleRoll")
    times.append([1.5, 3.0])
    keys.append([0.00361548, 0.00361548])

    names.append("RElbowRoll")
    times.append([1.5, 3.0])
    keys.append([0.530618, 0.530618])

    names.append("RElbowYaw")
    times.append([1.5, 3.0])
    keys.append([1.39613, 1.39613])

    names.append("RHand")
    times.append([1.5, 3.0])
    keys.append([0.0590473, 0.0590473])

    names.append("RHipPitch")
    times.append([1.5, 3.0])
    keys.append([-0.45268, -0.45268])

    names.append("RHipRoll")
    times.append([1.5, 3.0])
    keys.append([-0.00643682, -0.00643682])

    names.append("RHipYawPitch")
    times.append([1.5, 3.0])
    keys.append([-0.00721956, -0.00721956])

    names.append("RKneePitch")
    times.append([1.5, 3.0])
    keys.append([0.699545, 0.699545])

    names.append("RShoulderPitch")
    times.append([1.5, 3.0])
    keys.append([1.42169, 1.42169])

    names.append("RShoulderRoll")
    times.append([1.5, 3.0])
    keys.append([-0.279325, -0.279325])

    names.append("RWristYaw")
    times.append([1.5, 3.0])
    keys.append([0.0384861, 0.0384861])
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
    keys.append([-0.00487317])

    names.append("LAnkleRoll")
    times.append([2])
    keys.append([0.00464392])


    names.append("LElbowRoll")
    times.append([2])
    keys.append([-0.0707442])

    names.append("LElbowYaw")
    times.append([2])
    keys.append([-0.00491104])

    names.append("LHand")
    times.append([2])
    keys.append([0])

    names.append("LHipPitch")
    times.append([2])
    keys.append([-0.00626551])

    names.append("LHipRoll")
    times.append([2])
    keys.append([4.19617e-05])

    names.append("LHipYawPitch")
    times.append([2])
    keys.append([-0.00255999])

    names.append("LKneePitch")
    times.append([2])
    keys.append([0.00974634])

    names.append("LShoulderPitch")
    times.append([2])
    keys.append([1.78726])

    names.append("LShoulderRoll")
    times.append([2])
    keys.append([0.387533])

    names.append("LWristYaw")
    times.append([2])
    keys.append([-1.61286])

    names.append("RAnklePitch")
    times.append([2])
    keys.append([-0.00487317])

    names.append("RAnkleRoll")
    times.append([2])
    keys.append([-0.00149202])

    names.append("RElbowRoll")
    times.append([2])
    keys.append([0.421753])

    names.append("RElbowYaw")
    times.append([2])
    keys.append([1.21901])

    names.append("RHand")
    times.append([2])
    keys.append([0.3])

    names.append("RHipPitch")
    times.append([2])
    keys.append([-0.00626551])

    names.append("RHipRoll")
    times.append([2])
    keys.append([0.00157595])

    names.append("RHipYawPitch")
    times.append([2])
    keys.append([-0.00255999])

    names.append("RKneePitch")
    times.append([2])
    keys.append([0.00974634])

    names.append("RShoulderPitch")
    times.append([2])
    keys.append([1.41927])

    names.append("RShoulderRoll")
    times.append([2])
    keys.append([-0.262217])

    names.append("RWristYaw")
    times.append([2])
    keys.append([0.0915835])


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
    keys.append([0.0, 0.0])

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
#@功能说明： 机器人放杆过程中防止让杆碰地，所以加了一个举杆的中间动作
#@最后修改日期：2016-8-29
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
#@最后修改日期：2016-8-29
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
#@功能说明： 最开始的抓杆动作，左手抓杆。
#@最后修改日期：2016-8-29
#*********************************************************************************************************************
def catchStick(robotIP="127.0.0.1", port=9559):
    MOTION = ALProxy("ALMotion", robotIP, port)
    names, keys, times = positionCatchStick()
    MOTION.angleInterpolation(names, keys, times, True)

#---------------------------------------------------------------------------------------------------------------------#
#*********************************************************************************************************************
#@函数名：   releaseStick()
#@参数：     robotIP - 机器人IP地址
#           port - 机器人端口
#@返回值：   无
#@功能说明： 松杆
#@最后修改日期：2016-8-29
#*********************************************************************************************************************
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
#@最后修改日期：2016-8-29
#*********************************************************************************************************************
def hitBall(hitBallSpeed=0.1, robotIP="127.0.0.1", port=9559):
    MOTION = ALProxy("ALMotion", robotIP, port)
    names, keys, times = positionHitBall()
    MOTION.angleInterpolation(names, keys, times, True)
    time.sleep(2)
    MOTION.angleInterpolationWithSpeed("LWristYaw", -30*math.pi/180.0, hitBallSpeed)

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

    moveConfig = [["MaxStepX",args.MaxStepX],["MaxStepY",args.MaxStepY],["MaxStepTheta",args.MaxStepTheta],["MaxStepFrequency",args.MaxStepFrequency],
                    ["StepHeight",args.StepHeight],["TorsoWx",args.TorsoWx],["TorsoWy",args.TorsoWy]]

    robotIP = args.ip
    port = args.port

    MOTION = ALProxy("ALMotion", robotIP, port)
    POSTURE = ALProxy("ALRobotPosture", robotIP, port)
    MOTION.setFallManagerEnabled(True)

    MOTION.moveTo(1.00, -0.15, -45*math.pi/180.0, moveConfig)
