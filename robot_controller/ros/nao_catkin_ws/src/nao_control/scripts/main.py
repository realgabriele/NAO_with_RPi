#!/usr/bin/env python
# -*- encoding: UTF-8 -*-

import nao_interface as nao
from detect import detect
from prendi import prendi

import time

import rospy
from std_msgs.msg import String #è giusto così?
from nao_control.srv import *


def before():
    nao.execute_command("ALRobotPostureProxy", "goToPosture", ['StandInit', 0.7])
    print("posizione iniziale")


def after():
    nao.execute_command("ALMotionProxy", "rest")
    print("resting")


def search(obj):
    threshold = 0.1
    curr_head = 0

    rospy.wait_for_service('detect')
    detect = rospy.ServiceProxy('detect', Detect)

    while True:
        print("-----")
        print()
        pos = detect(obj)
        center, radius = ((pos[0], pos[1]), pos[2])
        print("center: " + str(center))
        print("radius: " + str(radius))

        if center is None:  # non trovato
            # cerca (muovi in tondo)
            if curr_head < 0:curr_head += 0.1
            elif curr_head > 0.1: curr_head -= 0.1
            nao.execute_command("ALMotionProxy", "setAngles", [
                ['HeadYaw', 'HeadPitch'], [0, curr_head], 0.2
            ])
            nao.move(0, 0, -0.2)
            time.sleep(1)

        else:  # trovato
            cntr_x, cntr_y = center
            x, y, z = 0, 0, 0

            # centra orizzontale (con threshold)
            if cntr_x < 0.5 - threshold:
                z = +0.1
            elif cntr_x > 0.5 + threshold:
                z = -0.1

            # centra verticale (con threshold) - movimento testa
            if cntr_y < 0.5 - threshold:
                y = cntr_y - 0.5
            elif cntr_y > 0.5 + threshold:
                y = cntr_y - 0.5

            # muovi (verso oggetto)
            if radius < 0.24:
                x = 0.24 - radius
            elif radius > 0.4:
                x = -0.1

            print("(x, y, z): " + str((x, y, z)))
            if x == y == z == 0:
                nao.move(0, 0, 0)
                print("prendendo!")
                prendi()
                return
            else:
                curr_head += y
                nao.execute_command("ALMotionProxy", "setAngles", [
                    ['HeadYaw', 'HeadPitch'], [0, curr_head], 0.2
                ])
                nao.move(x, 0, z)
                time.sleep(0.3)


def main():
    before()
    time.sleep(3)

    # wait for the input object
    # obj = listen()

    # search the object
    # search(obj)
    search("palla")

    # release the resources
    # after()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        after()
