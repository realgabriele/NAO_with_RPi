#!/usr/bin/python

# Copyright 2014 Keith Clark, Peter Robinson
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import threading
import math
import pedroclient
import time
import cv2
import imutils
from nao_interface import NaoInterface

import queue as Queue

message_queue = Queue.Queue()

SPEED_FACTOR = 0.1
TURN_FACTOR = 0.1
OBJECT = "palla"

RADIUS = 0.30
SHOULDER_PITCH = -0.14
THRESHOLD_X = 0.15


def dir2text(dir):
    if dir < -THRESHOLD_X:
        return "left"
    elif dir > THRESHOLD_X:
        return "right"
    else:
        return "centre"


def text2dir(dir):
    if dir == "left":
        return 1
    elif dir == "right":
        return -1
    else:
        return 0


def rad2dist(rad):
    # convert radius to distance
    return (RADIUS - rad) * 100


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def approx_dist(n):
    if n < 2:
        return 0
    return 2 ** (int(round(math.log(n) / math.log(2))))


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
    if obj == "palla":  # RED ball
        low_bnd = [(160, 100, 100), (0, 100, 100)]
        up_bnd = [(180, 255, 255), (10, 255, 255)]
    elif obj == "macchina":  # BLUE car
        low_bnd = (70, 60, 80)
        up_bnd = (120, 255, 255)
    elif obj == "papera":  # YELLOW duck
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
    '''ref_contours = np.load("cnt/" + obj + ".npy", allow_pickle=True)
    result = []
    for cnt in cnts:
        if cv2.contourArea(cnt) > 100:  # minimum area
            for ref_cnt in ref_contours:
                match = cv2.matchShapes(ref_cnt, cnt, 1, 0)
                if match < 0.10:  # maximum match
                    result.append(cnt)
    cnts = result'''

    cv2.imshow("frame", image)
    cv2.waitKey(5)

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
    else:  # the obj was not found in the image
        return None, None


class GetNotifications(threading.Thread):
    def __init__(self, client):
        self.client = client
        self.running = True
        threading.Thread.__init__(self)

    def run(self):
        """Run as thread - post an event when notifications arrive"""

        while self.running:
            try:
                p2pmsg = self.client.get_term()[0]
                message_queue.put(p2pmsg)
            except:
                break

    def stop(self):
        self.running = False


class Robot(object):
    def __init__(self, env):
        self.env = env
        self.nao_interface = NaoInterface(self)

        self.see = None  # (distance, direction)
        self.obstacle = None    # (distance, direction)
        self.grip_close = False
        self.holding = False
        self.fallen = False

        self.speed = [0, 0, 0]
        self.head_y = 0
        self.object = OBJECT

        self.camera_id = 0
        self.camera = None

    def send_msg(self, msg):
        self.env.send_msg(msg)

    def send_perceptions(self):
        perceptions = []
        if self.see is not None:
            perceptions.append("see(" + str(round(self.see[0])) + ", " + self.see[1] + ")")
        if self.obstacle is not None:
            perceptions.append("obstacle(" + str(round(self.see[0])) + ", " + self.see[1] + ")")
        if self.holding:
            perceptions.append("holding()")
        if self.fallen:
            perceptions.append("fallen()")
        if self.grip_close:
            perceptions.append("grip_close()")
        perceptions_str = "[" + ",".join(perceptions) + "]"
        self.send_msg(perceptions_str)
        print("perceptions: " + perceptions_str)

    def update_perceptions(self):
        self.check_seeing()
        self.check_holding()

    def check_seeing(self):
        image = self.nao_interface.get_camera()

        center, radius = detect_ocv(image, self.object)
        if center is not None:
            self.see = (rad2dist(radius), dir2text(center[0] - 0.5))
            self.head_y += 0.1 if center[1] > 0.5 else -0.1
        else:  # the obj was not found in the image
            self.see = None
            self.head_y += 0.1 if self.head_y < 0 else -0.1
        # update head pitch
        self.nao_interface.execute_command("ALMotionProxy", "setAngles", [
            ['HeadYaw', 'HeadPitch'], [0, self.head_y], 0.2
        ])

    def check_holding(self):
        if self.grip_close:
            self.check_seeing()
            if self.see is not None:
                self.holding = True
        self.holding = False

    def move_forward(self, speed):
        self.speed[0] = speed * SPEED_FACTOR

    def move_sideways(self, sdir):
        dir = text2dir(sdir)
        self.speed[1] = dir * SPEED_FACTOR

    def stop_move(self):
        self.speed[0] = 0
        self.speed[1] = 0

    def turn(self, tdir):
        dir = text2dir(tdir)
        self.speed[2] = dir * TURN_FACTOR

    def stop_turn(self):
        self.speed[2] = 0

    def grab(self):
        self.take()
        self.grip_close = True
        self.check_holding()

    def release(self):
        self.stand()
        self.grip_close = False
        self.holding = False

    def stand(self):
        sub = self.nao_interface.get_response()
        self.nao_interface.execute_command("ALRobotPostureProxy", "goToPosture", ['StandInit', 0.7])
        while True:
            msg = sub.listen()
            if msg['type'] == 'message':
                response = msg['data'].split(';')[0]
                value = msg['data'].split(';')[1]
                if response == "goToPosture":
                    if value == "True":
                        self.fallen = False
                        break

    def update_move(self):
        self.nao_interface.execute_command("ALMotionProxy", "move", self.speed)

    def take(self):
        """ Take the object from the table. """
        self.update_move()
        time.sleep(3)
        vel = 0.1  # movements speed

        # standing initial position
        self.nao_interface.execute_command("ALRobotPostureProxy", "goToPosture", ['Stand', 0.7])
        self.nao_interface.execute_command("ALMotionProxy", "setAngles", [
            ['HeadYaw', 'HeadPitch'], [0, 0.27], 0.2
        ])

        time.sleep(3)

        ''' evita tavolo '''

        # braccia larghe dai fianchi
        self.nao_interface.execute_command("ALMotionProxy", "setAngles", [
            ['RShoulderRoll', 'LShoulderRoll'],
            [-1.32, 1.32], vel])

        time.sleep(1)

        # gira le braccia
        self.nao_interface.execute_command("ALMotionProxy", "setAngles", [
            ['RShoulderPitch', 'LShoulderPitch'],
            [SHOULDER_PITCH, SHOULDER_PITCH], vel])

        time.sleep(1)

        # stand (quasi) zero
        self.nao_interface.execute_command("ALMotionProxy", "setAngles", [
            ['RShoulderRoll', 'LShoulderRoll', 'RElbowRoll', 'LElbowRoll', 'RElbowYaw', 'LElbowYaw'],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel])

        time.sleep(3)

        # abbassa le braccia
        self.nao_interface.execute_command("ALMotionProxy", "setAngles", [
            ['RShoulderPitch', 'LShoulderPitch'],
            [SHOULDER_PITCH, SHOULDER_PITCH], vel])

        # gira le mani verso oggetto
        self.nao_interface.execute_command("ALMotionProxy", "setAngles", [
            ['LWristYaw', 'RWristYaw', 'LHand', 'RHand'],
            [-1.82, 1.82, 1.00, 1.00], vel])

        time.sleep(3)

        # unisci braccia
        self.nao_interface.execute_command("ALMotionProxy", "setAngles", [
            ['LElbowRoll', 'RElbowRoll'],
            [-0.80, 0.80], vel])

        time.sleep(2)

        # mentre unisce ancora un po
        self.nao_interface.execute_command("ALMotionProxy", "setAngles", [
            ['LElbowRoll', 'RElbowRoll'],
            [-0.87, 0.87], vel])
        # afferra oggetto con dita
        self.nao_interface.execute_command("ALMotionProxy", "setAngles", [
            ['LHand', 'RHand'],
            [0.50, 0.50], vel])

        time.sleep(3)

        # solleva oggetto
        self.nao_interface.execute_command("ALMotionProxy", "setAngles", [
            ['LShoulderPitch', 'RShoulderPitch'],
            [-0.78, -0.78], vel])
        # .. e testa
        self.nao_interface.execute_command("ALMotionProxy", "setAngles", [
            ['HeadYaw', 'HeadPitch'], [0, -0.23], 0.2
        ])

        time.sleep(5)


class Environment(object):
    def __init__(self, parent):
        self.parent = parent
        self.messages_to_send = None

        self.robot = Robot(self)

        self.sender_process = None

    def process_msg(self, term):
        msg = term.args[2]
        print("processing message: " + str(msg))
        sender_process = term.args[1].args[0].args[1].val
        self.sender_process = sender_process

        if str(msg) == "initialise_":
            self.robot.started = True
            self.robot.send_msg("init")
            return
        elif msg.get_type() == pedroclient.PObject.atomtype and msg.val == 'finish':
            return

        elif msg.get_type() == pedroclient.PObject.structtype and msg.functor.val == 'msg':
            cmd = msg.args[0].functor.val
            if cmd == 'count':
                cmd_args = msg.args[0].args
                print("count {}".format(cmd_args[0].val))
                self.robot.count = cmd_args[0].val

        elif msg.get_type() == pedroclient.PObject.structtype and msg.functor.val == 'controls':
            actions = msg.args[0]
            if actions.get_type() != pedroclient.PObject.listtype:
                assert False, "COMMAND: {}".format(str(msg))
            for a in actions.toList():
                self.process_action(a)
            return

    def process_action(self, msg):
        assert (msg.get_type() == pedroclient.PObject.structtype), "Command is not a structure"
        functor = msg.functor
        cmd_type = functor.val
        cmd = msg.args[0]
        print(cmd_type + str(cmd))
        assert (cmd.get_type() == pedroclient.PObject.structtype), "Command is not a structure"
        cmd_args = cmd.args
        cmd = cmd.functor.val
        if cmd_type == 'stop_':
            if cmd == 'turn':
                self.robot.stop_turn()
            elif cmd == 'move' or cmd == 'side_move':
                self.robot.stop_move()
            else:
                assert False, "COMMAND: {}".format(str(msg))
        elif cmd_type in ['start_', 'mod_']:
            if cmd == 'turn':
                self.robot.turn(cmd_args[0].val)
            elif cmd == 'move':
                self.robot.move_forward(cmd_args[0].val)
            elif cmd == 'side_move':
                self.robot.move_sideways(cmd_args[0].val)
            else:
                assert False, "COMMAND: {} {}".format(cmd, str(msg))
        elif cmd_type == 'exec_':
            if cmd == 'grab':
                self.robot.grab()
            elif cmd == 'release':
                self.robot.release()
            elif cmd == "stand":
                self.robot.stand()
            else:
                print("COMMAND: {}".format(str(msg)))
                assert False, "COMMAND: {}".format(str(msg))
        else:
            assert False, "COMMAND: {}".format(str(msg))

    def send_msg(self, msg):
        if self.sender_process is not None:
            self.parent.client.p2p("percepts:{0}".format(self.sender_process), msg)

    def send_message(self, msg):
        self.parent.client.p2p("messages:robot0", msg)

    def send_log(self, msg):
        self.parent.client.p2p("thread0:logger2", '[' + msg + ',nl]')


class NaoApp(object):
    """ Main app; creates all needed instances. """

    def __init__(self):
        self.env = Environment(self)
        self.running = True
        self.client = pedroclient.PedroClient()
        self.client.register('nao_robot')

        self.thread = GetNotifications(self.client)
        self.thread.setDaemon(True)
        self.thread.start()

    def on_close(self):
        self.env.send_message("quiting")
        time.sleep(1)

    def main_loop(self):
        while True:
            if not message_queue.empty():
                self.env.process_msg(message_queue.get())
                self.env.robot.update_move()
            self.env.robot.update_perceptions()
            self.env.robot.send_perceptions()
            time.sleep(0.3)


if __name__ == "__main__":
    app = NaoApp()
    try:
        app.env.robot.nao_interface.execute_command("ALRobotPostureProxy", "goToPosture", ["StandInit", 0.5])
        app.main_loop()
    except KeyboardInterrupt:
        app.env.robot.nao_interface.execute_command("ALMotionProxy", "rest")
