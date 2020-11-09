import cv2
import numpy as np
import time
import imutils
from nao_interface import NaoInterface

# SIMULATED robot
OBJECT_SIZE = 0.28      # pixel width of object from image when close to the robot
SHOULDER_PITCH = -0.14  # L/RShoulderPitch when collecting the object

# REAL robot
# OBJECT_SIZE = 0.23      # pixel width of object from image when close to the robot
# SHOULDER_PITCH = 0.14   # L/RShoulderPitch when collecting the object

THRESHOLD_X = 0.02
THRESHOLD_Y = 0.05
THRESHOLD_Z = 0.07

SPEED_FACTOR = 1


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
        up_bnd = [(180, 255, 255), (10, 255, 255)]
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
    '''ref_contours = np.load("cnt/" + obj + ".npy", allow_pickle=True)
    result = []
    for cnt in cnts:
        if cv2.contourArea(cnt) > 100:  # minimum area
            for ref_cnt in ref_contours:
                match = cv2.matchShapes(ref_cnt, cnt, 1, 0)
                if match < 0.10:  # maximum match
                    result.append(cnt)
    cnts = result'''

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
        words = ['Palla', 'Papera', 'Macchina']
        # return "palla"

        sub = self.nao_interface.get_events()

        self.nao_interface.execute_command("ALSpeechRecognitionProxy", "pause", [True])
        self.nao_interface.execute_command("ALSpeechRecognitionProxy", "setLanguage", ['Italian'])
        self.nao_interface.execute_command("ALSpeechRecognitionProxy", "setVocabulary", [words, False])
        self.nao_interface.execute_command("ALSpeechRecognitionProxy", "subscribe", ['listen'])
        self.nao_interface.execute_command("ALSpeechRecognitionProxy", "pause", [False])
        self.nao_interface.execute_command("ALSpeechRecognitionProxy", "setParameter", ['speed', 80])

        for msg in sub.listen():
            if msg['type'] == 'message':
                event = msg['data'].split(';')[0]
                value = msg['data'].split(';')[1]
                if event == 'WordRecognized':
                    arr = eval(value)
                    word = arr[0]
                    if arr[1] > 0.4:
                        self.nao_interface.execute_command("ALTextToSpeechProxy", "say", [word])
                        self.nao_interface.execute_command("ALSpeechRecognitionProxy", "unsubscribe", ['listen'])
                        return word.lower()

    def detect(self, obj, cam=0):
        # update current frame
        self.update_camera()

        # show the frame in a popup window
        cv2.imshow("frame", self.camera_image)
        cv2.waitKey(5)

        # detect object position in frame
        return detect_ocv(self.camera_image, obj)

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

            if center is None or (center is not None and radius < 0.1):
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
