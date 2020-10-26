import redis
from PIL import Image

red = redis.Redis(host='localhost', port=6379)
sub0 = red.pubsub()
sub0.subscribe('camera0')
sub1 = red.pubsub()
sub1.subscribe('camera1')

message = sub0.get_message()
if message and message['type'] == 'message':
	naoImage = eval(message['data'])
	image = Image.frombytes("RGB", (naoImage[0], naoImage[1]), naoImage[6])
	image.save('image0.png')

message = sub1.get_message()
if message and message['type'] == 'message':
	naoImage = eval(message['data'])
	image = Image.frombytes("RGB", (naoImage[0], naoImage[1]), naoImage[6])
	image.save('image1.png')