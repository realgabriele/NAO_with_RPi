#!/usr/bin/env python
# -*- encoding: UTF-8 -*-

import nao_interface as nao
from listen import listen
from detect import detect
from prendi import prendi

import time


def before():
    nao.execute_command("ALRobotPostureProxy", "goToPosture", ['StandInit', 0.7])
    print("posizione iniziale")


def after():
    nao.execute_command("ALMotionProxy", "rest")


def search(obj):
    threshold = 0.1
    curr_head = 0

    while True:
        center, radius = detect(obj)
        print("center: " + str(center))
        print("radius: " + str(radius))

        if center is None:  # non trovato
            # cerca (muovi in tondo)
            nao.move(0, 0, -0.2)
            time.sleep(2)

        else:  # trovato
            cntr_x, cntr_y = center
            x, y, z = 0, 0, 0

            if cntr_x < 0.5 - threshold:  # centra orizzontale (con threshold)
                z = +0.1
            elif cntr_x > 0.5 + threshold:
                z = -0.1

            if cntr_y < 0.5 - threshold:  # centra verticale (con threshold) - movimento testa
                y = -0.1
            elif cntr_y > 0.5 + threshold:
                y = +0.1

            if radius < 0.27:  # muovi (verso oggetto)
                x = 0.27 - radius
            elif radius > 0.37:
                x = -0.1

            print((x, y, z))
            if x == y == z == 0:
                print("prendendo!")
                prendi()
            else:
                print("(x, y, z): " + str((x, y, z)))
                curr_head += y
                nao.execute_command("ALMotionProxy", "setAngles", [
                    ['HeadYaw', 'HeadPitch'], [0, curr_head], 0.2
                ])
                nao.move(x, 0, z)
                time.sleep(1)


def main():
    before()
    time.sleep(3)

    # wait for the input object
    # obj = listen()

    # search the object
    # search(obj)
    search("palla")

    # release the resources
    after()


if __name__ == "__main__":
    main()
