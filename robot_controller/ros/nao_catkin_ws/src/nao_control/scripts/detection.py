from imageai.Detection import ObjectDetection
import os
import redis
import time


red = redis.Redis(host='localhost', port=6379)
sub = red.pubsub()
sub.subscribe("object")
while True:
	mess = sub.get_message()
	if mess and mess['type'] == 'message':
		tosearch = mess['data']
		if tosearch == b'palla': tosearch = 'sports ball'
		if tosearch == b'macchina': tosearch = 'car'
		break

execution_path = os.getcwd()
#imm="..\images\image.jpg"
detector = ObjectDetection()
detector.setModelTypeAsRetinaNet()
detector.setModelPath(os.path.join(execution_path , "resnet50_coco_best_v2.0.1.h5"))
detector.loadModel()

image = 'image0.png'
switch = 0
trovato=0;

while True:
	x=0;
	y=0;

	detections = detector.detectObjectsFromImage(input_image=os.path.join(execution_path, image), output_image_path=os.path.join(execution_path, "imagenew.jpg"))
	
	for eachObject in detections:
		perso = True
		#print(eachObject["name"] , " : " , eachObject["percentage_probability"], " : ", eachObject["box_points"])
		if eachObject["name"] == tosearch:
			print ("Trovato")
			trovato=1
			x1=eachObject["box_points"][0]
			y1=eachObject["box_points"][1]
			x2=eachObject["box_points"][2]
			y2=eachObject["box_points"][3]
			x=(x2-x1)/2+x1
			y=(y2-y1)/2+y1
			perso = False
	if trovato == 0 and perso:
		image = 'image1.png'
		switch = 1
	
	red.publish("objectdetected",str(trovato)+","+str(x)+","+str(y)+','+str(switch))
