import threading


class HandleEvent(threading.Thread):
    """ Handles receiving events and changes the relative robot variables """
    def __init__(self,  robot):
        self.robot = robot
        self.sub = self.robot.nao_interface.get_events()

        self.running = True
        threading.Thread.__init__(self)

    def run(self):
        """Run as thread - post an event when notifications arrive"""
        while self.running:
            for msg in self.sub.listen():
                if msg['type'] == 'message':
                    message = msg['data'].decode("utf-8")
                    event = message.split(';')[0]
                    value = message.split(';')[1]
                    uid = message.split(';')[2]

                    if event == "SonarLeftDetected" or event == "SonarRightDetected":
                        dir = "left" if event == "SonarLeftDetected" else "right"
                        self.obstacle_detected(eval(value), dir)
                    if event == "SonarLeftNothingDetected" or event == "SonarRightNothingDetected":
                        self.obstacle_detected(0, None)
                    if event == "robotIsFalling" or event == "robotHasFallen":
                        self.robot_fallen()

    def stop(self):
        self.running = False

    def obstacle_detected(self, distance, direction):
        self.robot.obstacle_detected_distance = distance
        self.robot.obstacle_detected_direction = direction

    def robot_fallen(self):
        self.robot.fallen = True

