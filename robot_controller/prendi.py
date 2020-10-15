#! /usr/bin/env python
# -*- coding: utf-8 -*-

""" Gives commands to take an object from the floor """

from NAO_interface import execute_command
import time


def prendi():

    vel = 0.1  # velocita dei movimenti

    execute_command("ALRobotPostureProxy", "goToPosture", ['Stand', 0.7])
    #red.publish("commands", "ALRobotPostureProxy/goToPosture;['Stand', 0.7]")

    # evita tavolo
    execute_command("ALMotionProxy", "setAngles", [
        ['RShoulderRoll', 'LShoulderRoll'],
        [-1.32          , 1.32           ], vel])
    #red.publish("commands", "ALMotionProxy/setAngles;["
    #                        "['RShoulderRoll','LShoulderRoll'],"
    #                        "[-1.32,           1.32,       ], "+vel+"]")
    time.sleep(2)

    execute_command("ALMotionProxy", "setAngles", [
        ['RShoulderPitch', 'LShoulderPitch'],
        [0.00            , 0.00            ], vel])
    #red.publish("commands", "ALMotionProxy/setAngles;["
    #                       "['RShoulderPitch','LShoulderPitch'],"
    #                       "[ 0.00,            0.00,       ], "+vel+"]")
    time.sleep(2)
    execute_command("ALRobotPostureProxy", "goToPosture", ['StandZero', 0.7])
    #red.publish("commands", "ALRobotPostureProxy/goToPosture;['StandZero', 0.7]")
    time.sleep(3)

    # prendi oggetto dal tavolo
    execute_command("ALMotionProxy", "setAngles", [
        ['LWristYaw', 'RWristYaw', 'LHand', 'RHand'],
        [-1.82      , 1.82       , 1.0    , 1.0    ], vel])
    #red.publish("commands", "ALMotionProxy/setAngles;["
    #                        "['LWristYaw','RWristYaw','LHand','RHand'],"
    #                       "[-1.82,       1.82,       1.0,    1.0 ], "+vel+"]")
    time.sleep(3)
    # LShoulderPitch = 25°
    execute_command("ALMotionProxy", "setAngles", [
        ['LShoulderPitch', 'RShoulderPitch'],
        [0.43            , 0.43            ], vel])
    #red.publish("commands", "ALMotionProxy/setAngles;["
    #                        "['LShoulderPitch','RShoulderPitch'],"
    #                        "[ 0.43,            0.43], "+vel+"]")
    time.sleep(3)
    execute_command("ALMotionProxy", "setAngles", [
        ['LElbowRoll', 'RElbowRoll'],
        [-0.78       , 0.78        ], vel])
    #red.publish("commands", "ALMotionProxy/setAngles;["
    #                        "['LElbowRoll','RElbowRoll'],"
    #                        "[-0.78,        0.78], "+vel+"]")
    time.sleep(3)
    execute_command("ALMotionProxy", "setAngles", [
        ['LShoulderPitch', 'RShoulderPitch'],
        [0.00            , 0.00            ], vel])
    #red.publish("commands", "ALMotionProxy/setAngles;["
    #                        "['LShoulderPitch','RShoulderPitch'],"
    #                        "[ 0.00,            0.00], "+vel+"]")


if __name__ == "__main__":
    prendi()
