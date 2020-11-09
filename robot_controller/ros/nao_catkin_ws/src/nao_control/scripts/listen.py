#! /usr/bin/env python
# -*- coding: utf-8 -*-

""" Gives commands to take an object from the floor """

import rospy
import redis
import time
from nao_control.srv import Listen, ListenResponse


def handle_listen(req):
    red = redis.Redis(host='redis-ip', port=6379)
    sub = red.pubsub()
    sub.subscribe("events")

    red.publish("commands", "ALSpeechRecognitionProxy/pause;[True]")
    red.publish("commands", "ALSpeechRecognitionProxy/setLanguage;['Italian']")
    red.publish("commands", "ALSpeechRecognitionProxy/setVocabulary;["
                            "['palla','macchina'],False]")
    red.publish("commands", "ALSpeechRecognitionProxy/subscribe;['listen']")
    red.publish("commands", "ALSpeechRecognitionProxy/pause;[False]")
    red.publish("commands", "ALTextToSpeechProxy/setParameter;['speed',80]")

    # return ListenResponse("palla")
    for msg in sub.listen():
        if msg['type'] == 'message':
            event = msg['data'].split(';')[0]
            value = msg['data'].split(';')[1]
            if event == 'WordRecognized':
                arr = eval(value)
                word = arr[0]
                if arr[1] > 0.4:
                    red.publish("commands", "ALTextToSpeechProxy/say;['"+word+"']")
                    red.publish("commands", "ALSpeechRecognitionProxy/unsubscribe;['listen']")
                    return ListenResponse(word)


def listen_server():
    rospy.init_node('listen')
    s = rospy.Service('listen', Listen, handle_listen)
    rospy.spin()


if __name__ == "__main__":
    listen_server()


