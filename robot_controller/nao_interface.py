import configuration as cfg
import redis


def execute_command(proxy, command, parameters=[]):
    red = redis.Redis(host=cfg.REDIS_IP, port=cfg.REDIS_PORT)
    red.publish("commands", "" + proxy + "/" + command + ";" + str(parameters) + "")


def get_events():
    red = redis.Redis(host=cfg.REDIS_IP, port=cfg.REDIS_PORT)
    sub = red.pubsub()
    sub.subscribe("events")
    return sub


def get_camera(camera=0):
    """
    returns frame from selected camera
    """
    # ToDo
    pass


def move(x, y, z):
    cmd = f"ALMotionProxy/move;[{x}, {y}, {z}]"
    execute_command("ALMotionProxy", "move", [x, y, z])

