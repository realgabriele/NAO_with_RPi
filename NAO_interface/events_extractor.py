#! /usr/bin/env python
# -*- coding: utf-8 -*-

""" Subscribe to all desired events and publish them on redis"""

import sys
import time
import redis
import configuration as cfg

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule


Events = None

memory = None
sonar = None

red = None
i = 0


class EventSubscriber(ALModule):
    def __init__(self, name):
        ALModule.__init__(self, name)
        # Subscribe to generic event:
        memory.subscribeToEvent(name,   # event name
                                name,   # variable name
                                "onDetected")   # method name

    # callback(std::string eventName, float distance, std::string subscriberIdentifier)
    def onDetected(self, name, value):
        # Unsubscribe to the event when talking, to avoid repetitions
        memory.unsubscribeToEvent(name,
                                  name)

        # Redis
        print(name + ":[" + str(value) + "]")

        global red, i
        i = i+1
        red.publish("events", name + ";" + str(value) + ";" + str(i))

        # Subscribe again to the event
        memory.subscribeToEvent(name,
                                name,
                                "onDetected")


def main():
    """ Main entry point """
    # We need this broker to be able to construct
    # NAOqi modules and subscribe to other modules
    # The broker must stay alive until the program exists
    myBroker = ALBroker("sensors_extractor",
                        "0.0.0.0",  # listen to anyone
                        0,  # find a free port and use it
                        cfg.NAO_IP,  # parent broker IP
                        cfg.NAO_PORT)  # parent broker port

    global red
    red = redis.Redis(host=cfg.REDIS_IP, port=cfg.REDIS_PORT, db=0)

    global memory, sonar
    memory = ALProxy("ALMemory", cfg.NAO_IP, cfg.NAO_PORT)
    sonar = ALProxy("ALSonar", cfg.NAO_IP, cfg.NAO_PORT)
    sonar.subscribe("sonar")

    global Events
    Events = [line for line in open('events.txt', 'r').read().rstrip().split('\n') if not line[0] == '#']
    for event in Events:
        print event
        globals()[event] = EventSubscriber(event)

    try:
        while True:
            time.sleep(1)
            print("")
    except KeyboardInterrupt:
        print("Events: Interrupted, shutting down")
        sonar.unsubscribe("sonar")
        myBroker.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
