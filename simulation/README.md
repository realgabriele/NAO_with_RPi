simulation of NAO into V-REP

for this simulation we relied on the [Project_NAO_Control](https://github.com/PierreJac/Project-NAO-Control)

## Requirements

* NAOqi  (Version 2.1.4) : obtainable with the Choregraphe suite
* PyNAOqi (Version 2.1.4) : python library obtainable with the Python NAOqi SDK from Aldebaran
* V-REP  (Version 3.5.0) : robot simulation program from Coppelia Robotics.  
note: the program changed name to CoppeliaSim


## Quickstart

- Launch v-rep and load the scene _scene/NAO.ttt_
- Execute `$choregraphe_path$/bin/naoqi-bin -p port-number` to launch a simulated NAOqi.
- optional : You can launch Choregraphe.
- Start the v-rep simulation.
- Execute `python scripts/single_nao_control.py`.  
    This program represent the NAOqi robot inside the v-rep simulation.  
- Execute `python scripts/vision_sensor.py`  
    This program exports the v-rep camera into a redis channel.
