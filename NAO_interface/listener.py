#! /usr/bin/env python
# -*- coding: utf-8 -*-

""" Utility module:
    Show everything """

import redis
import cv2
import numpy as np
import sys
from PIL import Image

Memory = None
Events = None

red = None
subEv = None
subCam = None

memory_win = None
events_win = None
camera_win = None


def update_memory():
    global memory_win, Memory, red
    for num, value in enumerate(Memory):
        read_val = red.get(value)
        txt = Label(memory_win, text=value)
        txt.grid(column=0, row=num)
        val = Label(memory_win, text=read_val)
        val.grid(column=1, row=num)
    memory_win.update_idletasks()
    memory_win.update()


def update_events():
    global events_win, Events, subEv
    dict = {e: False for e in Events}
    message = subEv.get_message()
    while message:
        print(message)
        if message['type'] == 'message':
            event = message['data'].split(';')[0]
            value = message['data'].split(';')[1]
            dict[event] = value
        message = subEv.get_message()
    for num, event in enumerate(dict):
        txt = Label(events_win, text=event)
        txt.grid(column=0, row=num)
        val = Label(events_win, text=dict[event])
        val.grid(column=1, row=num)
    events_win.update_idletasks()
    events_win.update()


def update_camera():
    global camera_win, subCam
    message = subCam.get_message()
    if message and message['type'] == 'message':
        nao_image = eval(message['data'])
        #                                   width         height         array
        pil_image = Image.frombytes("RGB", (nao_image[0], nao_image[1]), nao_image[6])

        # CV2
        open_cv_image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)

        cv2.imshow("image", open_cv_image)
        cv2.waitKey(5)


def main():
    global red, subEv, subCam
    red = redis.Redis(host='redis-ip', port=6379)
    subEv = red.pubsub()
    subEv.subscribe('events')
    subCam = red.pubsub()
    subCam.subscribe('camera0')

    global memory_win, events_win, camera_win
    #memory_win = Tk()
    #memory_win.title("Memory Values")
    #events_win = Tk()
    #events_win.title("Events")
    #camera_win = Tk()
    #camera_win.title("Camera")

    try:
        while True:
            #update_memory()
            #update_events()
            update_camera()
            #time.sleep(1)
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down")
        sys.exit(0)


def init_val():
    global Memory, Events
    Memory = [line for line in open('memory.txt', 'r').read().rstrip().split('\n') if not line[0] == '#']
    Events = [line for line in open('events.txt', 'r').read().rstrip().split('\n') if not line[0] == '#']


if __name__ == "__main__":
    init_val()
    main()
