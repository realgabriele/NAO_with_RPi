#! /usr/bin/env python
# -*- coding: utf-8 -*-
# don't judge me for the name, I have a really dark humor

""" Listen the commands from a Redis channel and execute them on NAO
    publishes the response to another channel """

import sys
import redis
import re
import configuration as cfg

from naoqi import ALProxy

Commands = None

red = None
subCom = None


def executioner():
    global subCom

    pattern = re.compile("(.*)/(.*);\[(.*)\]")
    for message in subCom.listen():
        if message['type'] == 'message':
            proxy, command, parameters = pattern.findall(message['data'])[0]
            print(proxy+'.'+command+'('+parameters+')')
            var = globals()[proxy]
            e = eval('var.'+command+'('+parameters+')')
            red.publish("response", command+";"+str(e))
            print e


def main():
    global red, subCom, Commands
    Commands = open('commands.txt', 'r').read().rstrip().split('\n')
    red = redis.Redis(host=cfg.REDIS_IP, port=cfg.REDIS_PORT)
    subCom = red.pubsub()
    subCom.subscribe('commands')

    for com in Commands:
        globals()[com] = ALProxy(com[:-5], cfg.NAO_IP, cfg.NAO_PORT)

    try:
        executioner()
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down")
        sys.exit(0)


if __name__ == "__main__":
    main()
