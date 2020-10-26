#!/usr/bin/env python
# -*- encoding: UTF-8 -*-

import rospy
import time
import redis
import argparse
from std_msgs.msg import String #è giusto così?
from nao_control.srv import *

r = None

def before():
    global r
    r = redis.Redis(host='redis-ip', port=6379)

def after():
    # Go to rest position
    cmd="ALMotionProxy/rest;[]"
    #r.publish("commands", cmd)

def search():
    global r
    # ascola comandi vocali
    rospy.wait_for_service('listen')
    listen = rospy.ServiceProxy('listen', Listen)
    res = listen()
    print(res)
    r.publish("object", res.object)

    #avvicinamento alla papera
    rospy.wait_for_service('muoviti')
    muoviti = rospy.ServiceProxy('muoviti', Muoviti)
    res = muoviti()
    print(res)

    # afferra papera
    rospy.wait_for_service('prendi')
    prendi = rospy.ServiceProxy('prendi', Prendi)
    res = prendi()
    print(res)

def main(robotIP, PORT=9559):
    before()
    search()
    after()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)
