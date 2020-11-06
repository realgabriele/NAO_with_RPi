# -*- coding: utf-8 -*-
"""
Created on Thu Jul  9 15:30:54 2015

@author: Pierre Jacquot
"""

import vrep,time,sys
import matplotlib.pyplot as plt
from PIL import Image as I
import numpy as np
import cv2
import array
import redis
import pickle


def getVisionSensor(visionSensorName,clientID):
    """
    exports camera from V-Rep simulation to Redis channel
    """

    red = redis.Redis(host="localhost", port=6379)

    #Get the handle of the vision sensor
    res1,visionSensorHandle=vrep.simxGetObjectHandle(clientID,visionSensorName,vrep.simx_opmode_oneshot_wait)

    #Get the image
    res2,resolution,image=vrep.simxGetVisionSensorImage(clientID,visionSensorHandle,0,vrep.simx_opmode_streaming)
    time.sleep(1)

    id = 0
    while vrep.simxGetConnectionId(clientID)!=-1:
        #Get the image of the vision sensor
        res, resolution, image = vrep.simxGetVisionSensorImage(clientID, visionSensorHandle, 0, vrep.simx_opmode_buffer)
        print(resolution)

        image_byte_array = array.array('b', image)
        im = I.frombuffer("RGB", (resolution[0], resolution[1]), image_byte_array, "raw", "RGB", 0, 1)

        open_cv_image = cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)
        open_cv_image = cv2.flip(open_cv_image, 0)
        red.publish("camera0", pickle.dumps(open_cv_image, protocol=2))
        red.set("camera0", pickle.dumps(open_cv_image, protocol=2))
        red.set("cameraid", id)
        id += 1

        time.sleep(0.5)

    print 'End of Simulation'

if __name__ == '__main__':
    #vrep.simxFinish(-1)
    clientID=vrep.simxStart('127.0.0.2',19998,True,True,5000,5)
    if clientID!=-1:
        print 'Connected to remote API server'
        getVisionSensor('NAO_vision1',clientID)

    else:
        print 'Connection non successful'
        sys.exit('Could not connect')
