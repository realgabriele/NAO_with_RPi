#! /usr/bin/env python
# -*- coding: utf-8 -*-

""" Take all desired values from NAO memory """

import sys
import time
import redis
import configuration as cfg

from naoqi import ALProxy

Values = None
memory = None
red = None


def memory_extractor():
    global Values, red
    for value in Values:
        read_val = memory.getData(value)
        # print(value + ":\t" + str(read_val))
        red.set(value, read_val)


def main():
    global Values
    Values = [line for line in open('memory.txt', 'r').read().rstrip().split('\n') if not line[0] == '#']

    global red, memory
    red = redis.Redis(host=cfg.REDIS_IP, port=cfg.REDIS_PORT)
    memory = ALProxy("ALMemory", cfg.NAO_IP, cfg.NAO_PORT)

    try:
        while True:
            # print("\n\n")
            memory_extractor()
            time.sleep(1)

    except KeyboardInterrupt:
        print("Memory: Interrupted, shutting down")
        sys.exit(0)


if __name__ == "__main__":
    main()
