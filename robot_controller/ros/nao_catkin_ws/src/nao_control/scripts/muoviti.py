#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import redis
import time
from nao_control.srv import Muoviti, MuovitiResponse

def handle_muoviti(req):
    r = redis.Redis(host='redis-ip', port=6379)
    sub = r.pubsub()
    sub.subscribe("posizione")

    x_nao_cam=640
    y_nao_cam=480
    soglia_sx=float((x_nao_cam/2)-10)	#310
    soglia_dx=float((x_nao_cam/2)+10)	#330
    soglia_su=float((y_nao_cam/2)-10)	#230
    soglia_giu=float((y_nao_cam/2)+10)	#250

    arrivato=False
    switch_cam=False

    while(not arrivato):
        temp=sub.get_message()
        if temp == None: continue
        if temp["type"] != "message": continue
        temp = temp["data"]

        array_pos=temp.split(",")
        if array_pos[0]=="0":
            is_nao_watching_the_object=False
        else:
            is_nao_watching_the_object=True
        x_cam=float(array_pos[1])
        y_cam=float(array_pos[2])
        if array_pos[3]=="0":
            switch_cam=False
        else:
            switch_cam=True


        if not is_nao_watching_the_object:		# non sta guardando l'oggetto
            x=0
            y=0
            z=0.4
        else:						# ha trovato l'oggetto
            if not switch_cam:				# e ancora lontano
                if not soglia_sx < x_cam < soglia_dx:	# non e centrato orizzontalmente, gira
                    if x_cam < soglia_sx:
                        x=0
                        y=0
                        z=0.1
                    if x_cam > soglia_dx:
                        x=0
                        y=0
                        z=-0.1
                else:					# e centrato orizzontalmente, avanza
                    x=0.5
                    y=0
                    z=0
            else:					# e vicino
                if not soglia_su < y_cam < soglia_giu:	# non e arrivato al punto desiderato, avanza piano
                    x=0.1
                    y=0
                    z=0
                else:                                   # arrivato, stop!
                    x=0
                    y=0
                    z=0
                    arrivato = True

        cmd = "ALMotionProxy/moveToward;["+ str(x) +","+ str(y) +","+ str(z) + "]"
        r.publish("commands", cmd)
        x=0
        y=0
        z=0
        time.sleep(1)
        cmd = "ALMotionProxy/moveToward;["+ str(x) +","+ str(y) +","+ str(z) + "]"
        r.publish("commands", cmd)

    return MuovitiResponse(True)


def camera_server():
    rospy.init_node('muoviti')
    s = rospy.Service('muoviti', Muoviti, handle_muoviti)
    rospy.spin()

if __name__ == '__main__':
    camera_server()
