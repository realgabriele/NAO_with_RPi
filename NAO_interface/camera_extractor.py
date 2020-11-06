#! /usr/bin/env python
# -*- coding: utf-8 -*-

""" streams camera images via a Redis channel """

import sys
import time
import redis
import configuration as cfg
from PIL import Image

from naoqi import ALProxy
import vision_definitions

camera = None
Zero = None
Uno = None

red = None
i = 0


def camera_extractor():
    naoImage = camera.getImageRemote(Zero)
    #                               width        height        array
    img = Image.frombytes("RGB", (naoImage[0], naoImage[1]), naoImage[6])

    # perform transformation from PIL to OpenCV image
    image = img
    red.rpush("camera0", str(image))
    red.ltrim("camera0", 0, 9)  # only 10 images queued


    #red.publish("camera0", str(camera.getImageRemote(Zero)))    # upper
    #red.publish("camera1", str(camera.getImageRemote(Uno)))     # lower

    #red.set("camera0", str(camera.getImageRemote(Zero)))
    #red.set("camera1", str(camera.getImageRemote(Uno)))
    #red.set("cameraid", str(int(red.get("cameraid"))+1))
    red.publish("camera_notice", "published")
    print("published")


def main():
    print("main")
    global camera, Zero, Uno
    camera = ALProxy("ALVideoDevice", cfg.NAO_IP, cfg.NAO_PORT)

    #                                    id  resolution 320*240       color space                        fps
    Zero = camera.subscribeCamera("Zero", 0, vision_definitions.kVGA, vision_definitions.kRGBColorSpace, 30)
    # Uno  = camera.subscribeCamera("Uno",  1, vision_definitions.kVGA, vision_definitions.kRGBColorSpace, 30)

    global red
    red = redis.Redis(host=cfg.REDIS_IP, port=cfg.REDIS_PORT)

    try:
        while True:
            camera_extractor()
            time.sleep(1)
    except KeyboardInterrupt:
        print("Camera: Interrupted, shutting down")
        camera.unsubscribe(Zero)
        #camera.unsubscribe(Uno)
        sys.exit(0)


if __name__ == "__main__":
    main()
