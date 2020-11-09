import cv2
import time
import imutils
from nao_interface import NaoInterface

import rospy
from nao_control.srv import *

# SIMULATED robot
OBJECT_SIZE = 0.29      # pixel width of object from image when close to the robot
SHOULDER_PITCH = -0.14  # L/RShoulderPitch when collecting the object

# REAL robot
# OBJECT_SIZE = 0.23      # pixel width of object from image when close to the robot
# SHOULDER_PITCH = 0.14   # L/RShoulderPitch when collecting the object

THRESHOLD_X = 0.02
THRESHOLD_Y = 0.05
THRESHOLD_Z = 0.07

SPEED_FACTOR = 1


class Robot:
    def __init__(self):
        self.nao_interface = NaoInterface(self)
        self.camera_image = None
        self.seeing = (None, None)
        self.speed = [0, 0, 0]
        self.taken = False   # object taken and currently in hands

        self.head_y = 0

        self.obstacle_detected_distance = 0
        self.obstacle_detected_direction = None
        self.fallen = False

        rospy.wait_for_service('detect')
        self.detect_srv = rospy.ServiceProxy('detect', Detect)

    def update_camera(self, camera=0):
        self.camera_image = self.nao_interface.get_camera()

    def update_speed(self):
        self.nao_interface.move(self.speed[0]*SPEED_FACTOR, self.speed[1]*SPEED_FACTOR, self.speed[2]*SPEED_FACTOR)

    def update_head(self):
        self.nao_interface.execute_command("ALMotionProxy", "setAngles", [
            ['HeadYaw', 'HeadPitch'], [0, self.head_y], 0.2
        ])

    def get_up(self):
        sub = self.nao_interface.get_response()
        self.nao_interface.execute_command("ALRobotPostureProxy", "goToPosture", ['StandInit', 0.7])
        for msg in sub.listen():
            if msg['type'] == 'message':
                response = msg['data'].split(';')[0]
                value = msg['data'].split(';')[1]
                if response == "goToPosture":
                    if value == "True":
                        self.fallen = False
                        break

    def listen(self):
        rospy.wait_for_service('listen')
        listen = rospy.ServiceProxy('listen', Listen)
        obj = listen().object
        return obj

    def detect(self, obj, cam=0):
        # update current frame
        self.update_camera()

        # detect object position in frame
        pos = self.detect_srv(obj).position
        return pos

    def approach(self):
        center, radius = self.seeing
        cntr_x, cntr_y = center
        x, y, z = 0, 0, 0

        # move towards object
        if radius < OBJECT_SIZE - THRESHOLD_X:
            x = OBJECT_SIZE - radius - THRESHOLD_X
        elif radius > OBJECT_SIZE + THRESHOLD_X:
            x = OBJECT_SIZE - radius + THRESHOLD_X

        # centra verticale (con threshold) - movimento testa
        if cntr_y < 0.5 - THRESHOLD_Y:
            y = cntr_y - 0.5
        elif cntr_y > 0.5 + THRESHOLD_Y:
            y = cntr_y - 0.5

        # centra orizzontale (con threshold)
        if cntr_x < 0.5 - THRESHOLD_Z:
            z = 0.5 - cntr_x - THRESHOLD_Z
        elif cntr_x > 0.5 + THRESHOLD_Z:
            z = 0.5 - cntr_x + THRESHOLD_Z

        print("(x, y, z): " + str((x, y, z)))
        if x == z == 0:
            self.speed = [0, 0, 0]
            self.update_speed()
            return True
        else:
            self.head_y += y
            self.update_head()

            self.speed = [x, 0, z]
            self.update_speed()
            return False

    def take(self):
        """ Take the object from the table. """

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

        self.taken = True
        return True

    def search(self, obj):
        while True:
            print("-----")
            print("-----")
            print()
            center, radius = self.seeing = self.detect(obj)
            print("center: " + str(center))
            print("radius: " + str(radius))

            if self.fallen:
                self.get_up()
                continue

            if center is None or (center is not None and radius < OBJECT_SIZE-0.1):
                print("obstacle dir: " + str(self.obstacle_detected_direction))
                print("obstacle: dist" + str(self.obstacle_detected_distance))
                if self.obstacle_detected_direction is not None and self.obstacle_detected_distance < 0.2:
                    if self.obstacle_detected_direction == 'right':
                        self.speed = [0, 0.5, 0]
                    elif self.obstacle_detected_direction == 'left':
                        self.speed = [0, -0.5, 0]
                    self.update_speed()
                    continue

            if center is None:  # not found
                # search (turn around)
                if self.head_y < 0:
                    self.head_y += 0.1
                elif self.head_y > 0.1:
                    self.head_y -= 0.1
                self.update_head()

                self.speed = [0, 0, -0.5]
                self.update_speed()
                time.sleep(2)
                self.speed = [0, 0, 0]
                self.update_speed()

                continue

            if not self.approach():
                time.sleep(1)
                self.speed = [0, 0, 0]
                self.update_speed()
                continue

            self.take()
            self.nao_interface.move(-1, 0, 0)
            time.sleep(4)

            return True


nao = Robot()


if __name__ == "__main__":
    try:
        nao.nao_interface.execute_command("ALRobotPostureProxy", "goToPosture", ['StandInit', 0.7])
        time.sleep(2)

        # wait for the input object
        obj = nao.listen()

        # search the object
        nao.search(obj)

        # release the resources
        nao.nao_interface.execute_command("ALMotionProxy", "rest")
    except KeyboardInterrupt:
        print("rest")
        nao.nao_interface.execute_command("ALMotionProxy", "rest")
