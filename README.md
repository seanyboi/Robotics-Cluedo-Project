# Robotics Cluedo

## **INTRODUCTION**

The following is a final year project with aim of making a robot (in this case a Turtlebot) to play the famous board game **Cluedo**. In fact the robot has to search the environment (a room) trying to find the murderer and the weapon of the crime.

The task that the robot should perform are the following:

1. Object Search.
2. Object Detection.
3. Object Detection.

### *Object Search*

The object search enables the robot to perform an exploration of the environment given, with the aim of detecting the images on the wall during the former’s motion. The search has to be such that all the sections of the surroundings are visited, allowing the robot to detect the images even if these are placed in difficult positions.


### *Object Detection*

The image detections process has two components to it. The first one is the detection part, which is possible thanks to the usage of AR-markers. However, once the image is detected and its position in the map is retrieved a correct positioning in front of the object is required to obtain a successful recognition result.

### *Object Recognition*

The final step in the chain is recognising the image and store its map position as well as its snapshot with a boundary around the image. The recognition has been implemented such that it could be as robust as possible using computer vision techniques that enable a powerful feature realisation.

## **RESULTS**

Object Avoidance:
https://media.giphy.com/media/3oxHQewMafsJwPYnkY/giphy.gif

Object Detections & Recognition:
https://media.giphy.com/media/3ohs7TDZMDM4ZjHPRC/giphy.gif

Full working solution:
https://media.giphy.com/media/3ohs7V5iZVpXLg6vOU/giphy.gif
