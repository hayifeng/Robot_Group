#!/usr/bin/python2.7
# -*- encoding: UTF-8 -*-

import math
import time
import sys
import argparse
from naoqi import ALProxy

#-----------------------------------------------------机器人姿势定义---------------------------------------------------#
def positionCatchStick1():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1])
    keys.append([0.05825])

    names.append("HeadYaw")
    times.append([1])
    keys.append([0.00609397])

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
    keys.append([0.95])

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
    times.append([1])
    keys.append([-0.11049])

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
def positionCatchStick2():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([0.92])
    keys.append([0.05825])

    names.append("HeadYaw")
    times.append([0.92])
    keys.append([0.00609397])

    names.append("LAnklePitch")
    times.append([0.92])
    keys.append([-0.385075])

    names.append("LAnkleRoll")
    times.append([0.92])
    keys.append([4.19617e-05])

    names.append("LElbowRoll")
    times.append([0.92])
    keys.append([-1.1704])

    names.append("LElbowYaw")
    times.append([0.92])
    keys.append([-1.42666])

    names.append("LHand")
    times.append([0.92])
    keys.append([0])

    names.append("LHipPitch")
    times.append([0.92])
    keys.append([-0.454021])

    names.append("LHipRoll")
    times.append([0.92])
    keys.append([-0.00302602])

    names.append("LHipYawPitch")
    times.append([0.92])
    keys.append([-0.021434])

    names.append("LKneePitch")
    times.append([0.92])
    keys.append([0.682588])

    names.append("LShoulderPitch")
    times.append([0.92])
    keys.append([0.895815])

    names.append("LShoulderRoll")
    times.append([0.92])
    keys.append([-0.00157595])

    names.append("LWristYaw")
    times.append([0.92])
    keys.append([-0.11049])

    names.append("RAnklePitch")
    times.append([0.92])
    keys.append([-0.397265])

    names.append("RAnkleRoll")
    times.append([0.92])
    keys.append([0.00157595])

    names.append("RElbowRoll")
    times.append([0.92])
    keys.append([0.530805])

    names.append("RElbowYaw")
    times.append([0.92])
    keys.append([1.38976])

    names.append("RHand")
    times.append([0.92])
    keys.append([0.0512])

    names.append("RHipPitch")
    times.append([0.92])
    keys.append([-0.466378])

    names.append("RHipRoll")
    times.append([0.92])
    keys.append([0.00310993])

    names.append("RHipYawPitch")
    times.append([0.92])
    keys.append([-0.021434])

    names.append("RKneePitch")
    times.append([0.92])
    keys.append([0.694945])

    names.append("RShoulderPitch")
    times.append([0.92])
    keys.append([1.47728])

    names.append("RShoulderRoll")
    times.append([0.92])
    keys.append([-0.224006])

    names.append("RWristYaw")
    times.append([0.92])
    keys.append([0.030638])

    return [names,keys,times]
def positionHitBall1():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1])
    keys.append([0.05825])

    names.append("HeadYaw")
    times.append([1])
    keys.append([0.00609397])

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
    times.append([0.8])
    keys.append([0.2728])

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
    times.append([1])
    keys.append([1.0])

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
def positionHitBall2(speedTime):
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1])
    keys.append([0.05825])

    names.append("HeadYaw")
    times.append([1])
    keys.append([0.00609397])

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
    times.append([speedTime])
    keys.append([0.2728])

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
    times.append([1])
    keys.append([-1.0])

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
def positionStandWithStick():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1.2])
    keys.append([0.05825])

    names.append("HeadYaw")
    times.append([1.2])
    keys.append([0.00609397])

    names.append("LAnklePitch")
    times.append([1.2])
    keys.append([-0.00924586])

    names.append("LAnkleRoll")
    times.append([1.2])
    keys.append([4.19617e-05])

    names.append("LElbowRoll")
    times.append([1.2])
    keys.append([-0.076658])

    names.append("LElbowYaw")
    times.append([1.2])
    keys.append([-1.38218])

    names.append("LHand")
    times.append([1.2])
    keys.append([0])

    names.append("LHipPitch")
    times.append([1.2])
    keys.append([-0.00762796])

    names.append("LHipRoll")
    times.append([1.2])
    keys.append([-0.00302602])

    names.append("LHipYawPitch")
    times.append([1.2])
    keys.append([-0.00149202])

    names.append("LKneePitch")
    times.append([1.2])
    keys.append([0.00609397])

    names.append("LShoulderPitch")
    times.append([1.2])
    keys.append([1.82736])

    names.append("LShoulderRoll")
    times.append([1.2])
    keys.append([0.289883])

    names.append("LWristYaw")
    times.append([1.2])
    keys.append([-0.138102])

    names.append("RAnklePitch")
    times.append([1.2])
    keys.append([0.00157595])

    names.append("RAnkleRoll")
    times.append([1.2])
    keys.append([0.00157595])

    names.append("RElbowRoll")
    times.append([1.2])
    keys.append([0.0874801])

    names.append("RElbowYaw")
    times.append([1.2])
    keys.append([1.34681])

    names.append("RHand")
    times.append([1.2])
    keys.append([0.0404])

    names.append("RHipPitch")
    times.append([1.2])
    keys.append([-0.00310993])

    names.append("RHipRoll")
    times.append([1.2])
    keys.append([0.00310993])

    names.append("RHipYawPitch")
    times.append([1.2])
    keys.append([-0.00149202])

    names.append("RKneePitch")
    times.append([1.2])
    keys.append([4.19617e-05])

    names.append("RShoulderPitch")
    times.append([1.2])
    keys.append([1.57393])

    names.append("RShoulderRoll")
    times.append([1.2])
    keys.append([-0.280764])

    names.append("RWristYaw")
    times.append([1.2])
    keys.append([0.030638])

    return [names, keys, times]
def positionRaiseStick():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1])
    keys.append([0.05825])

    names.append("HeadYaw")
    times.append([1])
    keys.append([0.00609397])

    names.append("LAnklePitch")
    times.append([1])
    keys.append([-0.382009])

    names.append("LAnkleRoll")
    times.append([1])
    keys.append([4.19617e-05])

    names.append("LElbowRoll")
    times.append([1])
    keys.append([-0.995524])

    names.append("LElbowYaw")
    times.append([1])
    keys.append([-1.34229])

    names.append("LHand")
    times.append([1])
    keys.append([0.2872])

    names.append("LHipPitch")
    times.append([1])
    keys.append([-0.450955])

    names.append("LHipRoll")
    times.append([1])
    keys.append([-0.00302602])

    names.append("LHipYawPitch")
    times.append([1])
    keys.append([-0.024502])

    names.append("LKneePitch")
    times.append([1])
    keys.append([0.682588])

    names.append("LShoulderPitch")
    times.append([1])
    keys.append([0.380391])

    names.append("LShoulderRoll")
    times.append([1])
    keys.append([0.320565])

    names.append("LWristYaw")
    times.append([1])
    keys.append([0.846485])

    names.append("RAnklePitch")
    times.append([1])
    keys.append([-0.38806])

    names.append("RAnkleRoll")
    times.append([1])
    keys.append([0.00464392])

    names.append("RElbowRoll")
    times.append([1])
    keys.append([0.549213])

    names.append("RElbowYaw")
    times.append([1])
    keys.append([1.38823])

    names.append("RHand")
    times.append([1])
    keys.append([0.0648])

    names.append("RHipPitch")
    times.append([1])
    keys.append([-0.467912])

    names.append("RHipRoll")
    times.append([1])
    keys.append([0.00310993])

    names.append("RHipYawPitch")
    times.append([1])
    keys.append([-0.024502])

    names.append("RKneePitch")
    times.append([1])
    keys.append([0.705682])

    names.append("RShoulderPitch")
    times.append([1])
    keys.append([1.49416])

    names.append("RShoulderRoll")
    times.append([1])
    keys.append([-0.208666])

    names.append("RWristYaw")
    times.append([1])
    keys.append([0.0444441])
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
    SENSORS = ALProxy("ALSensors", robotIP, port)
    MOTION = ALProxy("ALMotion", robotIP, port)
    MEMORY = ALProxy("ALMemory", robotIP, port)
    #监听左手触摸事件
    SENSORS.subscribe("HandLeftLeftTouched",500,0.0)
    names, keys, times = positionCatchStick1()
    MOTION.angleInterpolation(names, keys, times, True)
    while(True):
        temp =MEMORY.getData("HandLeftLeftTouched")
        if(temp):
            names, keys, times = positionCatchStick2()
            MOTION.angleInterpolation(names, keys, times, True)
            break

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


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number.")

    args = parser.parse_args()

    robotIP = args.ip
    port = args.port
    catchStick(robotIP, port)
    time.sleep(2)
    #time.sleep(2)
    raiseStick(robotIP, port)
    time.sleep(2)
    standWithStick(0.1, robotIP, port)
    time.sleep(2)
    raiseStick(robotIP, port)
    time.sleep(2)
    hitBall(0.1, robotIP, port)
