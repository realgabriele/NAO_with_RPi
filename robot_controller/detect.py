#! /usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import imutils
import numpy as np

import nao_interface


def detect_ocv(frame, obj):

    # define the lower and upper boundaries of the objects in the HSV color space
    low_bnd, up_bnd = None, None
    if obj == "Palla":      # RED ball
        low_bnd = (0, 1, 0)
        up_bnd = (13, 255, 255)
    elif obj == "Macchina":  # BLUE car
        low_bnd = (70, 60, 80)
        up_bnd = (120, 255, 255)
    elif obj == "Papera":   # YELLOW duck
        low_bnd = (10, 70, 127)
        up_bnd = (30, 255, 255)

    # resize the frame, blur it, and convert it to the HSV color space
    # frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the choosen color, then perform a series of
    # dilations and erosions to remove any small blobs left in the mask
    mask = cv2.inRange(hsv, low_bnd, up_bnd)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Use "close" morphological operation to close small gaps between contours
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10)))

    # find contours in the mask and initialize the current (x, y) center of the object
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # filters the cnts based on reference contours
    ref_contours = np.load("cnt/papera.npy", allow_pickle=True)

    result = []
    for cnt in cnts:
        if cv2.contourArea(cnt) > 200:  # minimum area
            for ref_cnt in ref_contours:
                match = cv2.matchShapes(ref_cnt, cnt, 1, 0)
                if match < 0.10:        # maximum match
                    result.append(cnt)
    cnts = result

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        return center, radius
    else:
        return None, None

def detect_colorblob():
    sub = NAO_interface.get_events()

    for msg in sub.listen():
        if msg['type'] != 'message':
            continue
        event, value = msg['data'].strip(";")
        if event != "ALTracker/ColorBlobDetected":
            continue
        return str(value)


def detect(obj, cam):
    """
    detect the object inside the current camera frame
    :param str obj: object to be detected
    :return: ((center_x, center_y), radius)
    """

    # frame = NAO_interface.get_camera()

    # da eliminare
    ret, frame = cam.read()
    frame = cv2.flip(frame, 1)
    cv2.imshow("camera", frame)
    k = cv2.waitKey(5)

    return detect_ocv(frame, obj)
