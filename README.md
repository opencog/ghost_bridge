ghost_bridge
============
ghost_bridge is a ROS package that connects the Hanson Robotics stack and [GHOST](https://github.com/opencog/opencog/tree/master/opencog/ghost).
It provides a set of actions that can be called from GHOST and a set of perceptions that are sent to the OpenCog
Atomspace, an overview of the actions and perceptions are provided below.

#### Actions:
* **say**: make the robot vocalize text.
* **gaze-at**: turn the robot's eyes towards the given target point.
* **face-toward**: turn the robot's face towards the given target point.
* **blink**: set the robot's blink cycle.
* **saccade**: set the robot's eye saccade cycle, i.e. how the eye's twitch and move around automatically.
* **emote**: set the robot's emotional state.
* **gesture**: set a pose on the robot's face.
* **soma**: sets the robot's background facial expressions.

#### Perceptions:
* **perceive-emotion**: perceive an emotion.
* **perceive-eye_state**: perceive the state of a person's eyes.
* **perceive-face-talking**: the probability of whether a particular face is talking or not.
* **perceive-word**: perceive an individual word that is a part of the sentence a person is currently speaking.
* **perceive-sentence** (ghost): perceive the whole sentence after the user has finished speaking.

Setup
-------
TODO: this needs testing

Initial setup, only needed to be run once:
```bash
rosrun ghost_bridge setup.sh
```

Running
-------
Ensure that the HEAD stack is started with the below commands. <robot-name> could be `sophia4` or `han`.
```bash
hr run --nogui --dev --tracker none --sttcontinuous --nomuxtts <robot-name>
```

To run:
```bash
rosrun ghost_bridge run.sh
```

To stop:
```bash
rosrun ghost_bridge stop.sh
```

Design Goals
------------
TODO

Architecture
-------------------------------
TODO