#! /usr/bin/env python
# -*- coding: utf-8 -*-

""" Gives commands to take an object from the floor """

import rospy
import redis
import time
from nao_control.srv import Prendi, PrendiResponse


def handle_prendi(req):
    red = redis.Redis(host='redis-ip', port=6379)

    vel = "0.5"	# velocita dei movimenti

    red.publish("commands", "ALRobotPostureProxy/goToPosture;['StandZero', 1.0]")

    # evita tavolo
    red.publish("commands", "ALMotionProxy/setAngles;["
                            "['RShoulderRoll','LShoulderRoll'],"
                            "[-1.32,           1.32,       ], "+vel+"]")
    time.sleep(1)
    red.publish("commands", "ALMotionProxy/setAngles;["
                            "['RShoulderPitch','LShoulderPitch'],"
                            "[ 0.00,            0.00,       ], "+vel+"]")
    time.sleep(1)
    red.publish("commands", "ALRobotPostureProxy/goToPosture;['StandZero', 1.0]")
    time.sleep(2)

    ### prendi oggetto dal tavolo
    red.publish("commands", "ALMotionProxy/setAngles;["
                            "['LWristYaw','RWristYaw','LHand','RHand'],"
                            "[-1.82,       1.82,       1.0,    1.0 ], "+vel+"]")
    # LShoulderPitch = 25°
    red.publish("commands", "ALMotionProxy/setAngles;["
                            "['LShoulderPitch','RShoulderPitch'],"
                            "[ 0.43,            0.43], "+vel+"]")
    time.sleep(2)
    red.publish("commands", "ALMotionProxy/setAngles;["
                            "['LElbowRoll','RElbowRoll'],"
                            "[-0.78,        0.78], "+vel+"]")
    time.sleep(2)
    red.publish("commands", "ALMotionProxy/setAngles;["
                            "['LShoulderPitch','RShoulderPitch'],"
                            "[ 0.00,            0.00], "+vel+"]")
    return PrendiResponse(True)

def prendi_server():
    rospy.init_node('prendi')
    s = rospy.Service('prendi', Prendi, handle_prendi)
    rospy.spin()


if __name__ == "__main__":
    prendi_server()
