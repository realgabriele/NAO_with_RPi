# Robot Controller

This module constitutes the controller of the robot.  

## Goal
The goal of the project is to search for an object and take it.

In particular the main goal can be divided in the following steps:
- listen to the object to take: provide a human interface with speech-recognition and speech-to-text functions.
- look for the object: given the images provided by the robot camera, make object detection to see where the object is
- move: we have to deal with the movement of the robot towards the object, and in general in complex environment
- take the object: once reached, grab the object with the hands.

## Assumptions
0. Simple environment: we assume the robot in a plan room where the object can be directly seen.
We didn't implement functions to navigate through a labyrinth. 
0. Given the complexity to take a generic object from the ground without falling,
we assume that the object is raised from the ground (e.g. some shelf or small table)

## Description
For this scope we use the interface developed in the NAO_interface module.
Said module provides an interface with NAO through a Redis database,
making use of its channel and keys to provide perceptions and execute commands.

![module_image](../docs/assets/robot_controller.png)


## Controllers
We provided several different sub-modules (architectures) that use different
technologies or languages.

Every module is described below.
From more basic to most advanced
* simple
* sonar
* ros
* qulog


## simple
This controller is the first and simple version of the robot controller.

