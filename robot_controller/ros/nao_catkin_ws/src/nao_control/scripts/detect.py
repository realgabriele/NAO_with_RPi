#! /usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import imutils
import numpy as np
import configuration as cfg
from nao_interface import get_camera
import rospy
import redis
from nao_control.srv import Detect, DetectResponse


def detect_ocv(image, obj):
    """ Detect the object in the frame.

    This method uses OpenCV library to detect the object shape and color inside the frame.
    :param OpenCV image: the image to analyze
    :param str obj: the object to look for
    :return: position and size of the object relative to the frame
    :rtype: ((float, float), float)
    """

    # define the lower and upper boundaries of the objects in the HSV color space
    low_bnd, up_bnd = None, None
    if obj == "palla":      # RED ball
        low_bnd = [(160, 100, 100), (0, 100, 100)]
        up_bnd = [(180, 255, 255), (30, 255, 255)]
    elif obj == "macchina": # BLUE car
        low_bnd = (70, 60, 80)
        up_bnd = (120, 255, 255)
    elif obj == "papera":   # YELLOW duck
        low_bnd = (10, 70, 127)
        up_bnd = (30, 255, 255)

    # blur the frame, and convert it to the HSV color space
    res_y, res_x, _ = image.shape
    blurred = cv2.GaussianBlur(image, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the chosen color, then perform a series of
    # dilations and erosions to remove any small blobs left in the mask
    if type(low_bnd == list):
        mask = cv2.inRange(hsv, low_bnd[0], up_bnd[0])
        for i in range(1, len(low_bnd)):
            mask += cv2.inRange(hsv, low_bnd[i], up_bnd[i])
    else:
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

    # filters the cnts based on shape of reference contours
    ref_contours = np.load("cnt/" + obj + ".npy", allow_pickle=True)
    result = []
    for cnt in cnts:
        if cv2.contourArea(cnt) > 200:  # minimum area
            for ref_cnt in ref_contours:
                match = cv2.matchShapes(ref_cnt, cnt, 1, 0)
                if match < 0.10:  # maximum match
                    result.append(cnt)
    cnts = result

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use it
        # to compute the minimum enclosing circle and centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # normalize the position and radius compared to the frame size
        center = (center[0] / res_x, center[1] / res_y)
        radius = radius / res_y

        return center, radius
    else:   # the obj was not found in the image
        return None, None


def handle_detect(req):
    """
    detect the object inside the current camera frame
    :param req: Detect service request
    :return: Detect service response - position and size of the object relative to the frame
    :rtype: float64[]
    """

    '''
    obj = req.obj

    # get current frame
    frame = get_camera()

    # show the frame in a popup window
    cv2.imshow("frame", frame)
    cv2.waitKey(5)

    # detect object position in frame
    (x, y), r = detect_ocv(frame, obj)
    '''

    (x, y), r = ((0.5, 0.4), 16)
    return DetectResponse([x, y, r])


def detect_server():
    rospy.init_node('detect')
    s = rospy.Service('detect', Detect, handle_detect)
    rospy.spin()


if __name__ == "__main__":
    detect_server()



