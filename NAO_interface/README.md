this module constitutes the interface with ROS

## prerequisites
* python 2.7
* pynaoqi - the python library for NAO interaction.
You can download it [here]

## execution
`python main.py`

## functions
It serves an interface to "talk" with NAO.  
It uses Redis channels to export NAO data (sensors, events and memory values)
and to execute command on the robot itself.

