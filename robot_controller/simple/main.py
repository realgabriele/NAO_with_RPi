#!/usr/bin/env python
# -*- encoding: UTF-8 -*-

import nao_robot
from events import HandleEvent
import time

nao = nao_robot.Robot()
ev_handler = HandleEvent(nao)


def main():
    ev_handler.setDaemon(True)
    ev_handler.start()

    nao.nao_interface.execute_command("ALRobotPostureProxy", "goToPosture", ['StandInit', 0.7])
    time.sleep(3)

    # wait for the input object
    #obj = nao.listen()
    obj = "palla"

    # search the object
    nao.search(obj)

    # release the resources
    nao.nao_interface.execute_command("ALMotionProxy", "rest")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("rest")
        nao.nao_interface.execute_command("ALMotionProxy", "rest")
