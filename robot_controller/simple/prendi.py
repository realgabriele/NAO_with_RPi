#! /usr/bin/env python
# -*- coding: utf-8 -*-

""" Gives commands to take an object from the table. """

from nao_interface import execute_command
import time


def stampa(car):
    #input()
    print(car)


def prendi():
    """ Take the object from the table. """

    vel = 0.1  # velocita dei movimenti

    stampa("a")
    # standing initial position
    execute_command("ALRobotPostureProxy", "goToPosture", ['Stand', 0.7])

    time.sleep(1)

    ''' evita tavolo '''

    stampa("b")
    # braccia larghe dai fianchi
    execute_command("ALMotionProxy", "setAngles", [
        ['RShoulderRoll', 'LShoulderRoll'],
        [-1.32          , 1.32           ], vel])

    time.sleep(2)

    stampa("c")
    # gira le braccia
    execute_command("ALMotionProxy", "setAngles", [
        ['RShoulderPitch', 'LShoulderPitch'],
        [-0.14            ,-0.14            ], vel])

    time.sleep(2)

    stampa("d")
    # stand (quasi) zero
    execute_command("ALMotionProxy", "setAngles", [
        ['RShoulderRoll', 'LShoulderRoll','RElbowRoll', 'LElbowRoll', 'RElbowYaw', 'LElbowYaw'],
        [ 0.0,             0.0,            0.0,          0.0,          0.0,         0.0], vel])

    time.sleep(3)

    stampa("e")
    # gira le mani verso oggetto
    execute_command("ALMotionProxy", "setAngles", [
        ['LWristYaw', 'RWristYaw', 'LHand', 'RHand'],
        [-1.82      ,  1.82      ,  1.00  ,  1.00  ], vel])

    time.sleep(3)

    stampa("g")
    # unisci braccia
    execute_command("ALMotionProxy", "setAngles", [
        ['LElbowRoll', 'RElbowRoll'],
        [-0.80       , 0.80        ], vel])     # 0.78
    # afferra oggetto con dita
    execute_command("ALMotionProxy", "setAngles", [
        ['LHand', 'RHand'],
        [ 0.80  ,  0.80  ], vel])

    time.sleep(3)

    stampa("h")
    # solleva oggetto
    execute_command("ALMotionProxy", "setAngles", [
        ['LShoulderPitch', 'RShoulderPitch'],
        [-0.78           , -0.78            ], vel])
    # .. e testa
    execute_command("ALMotionProxy", "setAngles", [
        ['HeadYaw', 'HeadPitch'], [0, -0.23], 0.2
    ])


if __name__ == "__main__":
    prendi()
