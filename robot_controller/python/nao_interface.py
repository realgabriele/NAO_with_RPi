import configuration as cfg
import redis
import pickle


class NaoInterface:
    def __init__(self, master):
        self.master = master
        self.camera_id = 0

        self.red = redis.Redis(host=cfg.REDIS_IP, port=cfg.REDIS_PORT)

    def execute_command(self, proxy, command, parameters=[]):
        """ Execute command on NAO robot.

        :param str proxy: NAOqi proxy to be used
        :param str command: command to be executed
        :param list parameters: parameters to be passed as arguments to the command function
        """

        self.red.publish("commands", "" + proxy + "/" + command + ";" + str(parameters) + "")

    def get_events(self):
        """ Get the events Redis channel.

        :return: Redis channel subscribed to all NAO events
        :rtype: Redis PubSub
        """

        sub = self.red.pubsub()
        sub.subscribe("events")
        return sub

    def get_response(self):
        """ Get the response Redis channel.

        :return: Redis channel subscribed to all NAO responses
        :rtype: Redis PubSub
        """

        sub = self.red.pubsub()
        sub.subscribe("response")
        return sub

    def get_camera(self, camera=0):
        """ Get a frame from selected camera.

        :return: current frame
        :rtype: OpenCV image (numpy ndarray)
        """

        '''while self.red.get("cameraid") == self.camera_id:
            pass
        self.camera_id = self.red.get("cameraid")

        # load OpenCV image
        image = pickle.loads(self.red.get("camera" + str(camera)), encoding='bytes')
        return image'''

        sub = self.red.pubsub()
        sub.subscribe("camera_notice")
        # sub.subscribe("camera" + str(camera))
        for message in sub.listen():
            if message['type'] == 'message':
                # load OpenCV image
                image = pickle.loads(self.red.blpop("camera" + str(camera))[1], encoding='bytes')
                return image

    def move(self, x, y, z):
        """ Move NAO robot on the 3-axis.

        :param float x: front-back velocity in m/s
        :param float y: left-right velocity in m/s
        :param float z: velocity vertical axis in rad/s
        """

        self.execute_command("ALMotionProxy", "move", [x, y, z])

