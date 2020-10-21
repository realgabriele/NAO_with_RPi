import configuration as cfg
import redis
import pickle


def execute_command(proxy, command, parameters=[]):
    """ Execute command on NAO robot.

    :param str proxy: NAOqi proxy to be used
    :param str command: command to be executed
    :param list parameters: parameters to be passed as arguments to the command function
    """

    red = redis.Redis(host=cfg.REDIS_IP, port=cfg.REDIS_PORT)
    red.publish("commands", "" + proxy + "/" + command + ";" + str(parameters) + "")


def get_events():
    """ Get the events Redis channel.

    :return: Redis channel subscribed to all NAO events
    :rtype: Redis PubSub
    """

    red = redis.Redis(host=cfg.REDIS_IP, port=cfg.REDIS_PORT)
    sub = red.pubsub()
    sub.subscribe("events")
    return sub


def get_camera(camera=0):
    """ Get a frame from selected camera.

    :return: current frame
    :rtype: OpenCV image (numpy ndarray)
    """

    red = redis.Redis(host=cfg.REDIS_IP, port=cfg.REDIS_PORT)
    sub = red.pubsub()
    sub.subscribe("camera" + str(camera))
    while True:
        message = sub.get_message()
        if message and message['type'] == 'message':
            # load OpenCV image from message data
            image = pickle.loads(message['data'], encoding='bytes')
            return image


def move(x, y, z):
    """ Move NAO robot on the 3-axis.

    :param float x: front-back velocity in m/s
    :param float y: left-right velocity in m/s
    :param float z: velocity vertical axis in rad/s
    """

    execute_command("ALMotionProxy", "move", [x, y, z])

