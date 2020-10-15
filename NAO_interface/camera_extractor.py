#! /usr/bin/env python
# -*- coding: utf-8 -*-

""" Streams camera images via a Redis channel """

import sys
import time
import redis
import configuration as cfg

from naoqi import ALProxy
import vision_definitions

camera = None
Zero = None
Uno = None

red = None


def camera_extractor():
    # naoImage = camera.getImageRemote(nameId)
    #                               width        height        array
    # img = Image.frombytes("RGB", (naoImage[0], naoImage[1]), naoImage[6])

    red.publish("camera0", str(camera.getImageRemote(Zero)))    # upper
    red.publish("camera1", str(camera.getImageRemote(Uno)))     # lower

    red.set("camera0", str(camera.getImageRemote(Zero)))
    red.set("camera1", str(camera.getImageRemote(Uno)))


def main():
    global camera, Zero, Uno
    camera = ALProxy("ALVideoDevice", cfg.NAO_IP, cfg.NAO_PORT)
    #                                    id  resolution 320*240       color space                        fps
    Zero = camera.subscribeCamera("Zero", 0, vision_definitions.kVGA, vision_definitions.kRGBColorSpace, 30)
    Uno  = camera.subscribeCamera("Uno",  1, vision_definitions.kVGA, vision_definitions.kRGBColorSpace, 30)

    global red
    red = redis.Redis(host=cfg.REDIS_IP, port=cfg.REDIS_PORT)

    try:
        while True:
            camera_extractor()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down")
        # camera.unsubscribe(nameId)
        camera.unsubscribe(Zero)
        camera.unsubscribe(Uno)
        sys.exit(0)


if __name__ == "__main__":
    main()
