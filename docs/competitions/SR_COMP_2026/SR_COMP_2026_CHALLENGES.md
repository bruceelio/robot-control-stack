# Student Robotics — Challenges 2026

Source: [Student Robotics — Challenges 2026](https://studentrobotics.org/docs/resources/2026/challenges.html)

## Table of Contents

1. [Introduction](#introduction)
2. [Vision Challenge](#vision-challenge)
3. [Movement Challenge](#movement-challenge)
4. [Mechanics Challenge](#mechanics-challenge)
5. [Sensing Challenge](#sensing-challenge)
6. [Simulator Challenge](#simulator-challenge)
7. [Transportation Challenge](#transportation-challenge)
8. [Stopping Challenge](#stopping-challenge)

## Introduction

There are seven challenges which teams may optionally complete during the competition year. League points are available for up to four of your choosing. See the main [rulebook](https://studentrobotics.org/docs/resources/2026/rulebook.html#comp-structure) for details on how these relate to the overall competition and when the deadlines for completion are.

Throughout these challenges a “robot” need not be fully constructed, nor is it limited by the size limits which would apply in the competition arena. Robots may use whatever sensing mechanisms they like, as long as those mechanisms would be permitted in the competition arena.

However, robots must be **safe** (as specified in the [regulations](https://studentrobotics.org/docs/resources/2026/rulebook.html#safety-regulations)) when completing these challenges.

Only the Simulator challenge may be completed in the simulator. The rest must be completed with a physical robot. When constructing components of the arena for use in challenges, they must match the specifications defined in the rules.

Submissions for each challenge should be made as a video on the web (e.g. on YouTube, Instagram, etc.) and linking this video in your Discord channel. When linking the video please use `@Challenges` so that your submission is seen. If a team’s challenge submission is not deemed successful, the team may attempt the challenge again. Feedback will be provided about why the submission was not successful.

Remember that you can also post in [Discord](https://studentrobotics.org/docs/tutorials/discord) if you want some help.

## Vision Challenge

This challenge is to think about how your robot can use the camera to interpret the world around it.

To complete this challenge you will need to use the camera from your kit. The onboard LEDs will be used to indicate the robot’s state.

Certain sensors are more useful in certain situations, either due to their range, accuracy, or the information they provide. It is worth considering how you can use the sensors we’ve provided to you.

This challenge has two parts which can be submitted as separate videos, but both parts must be completed to earn the points.

To complete this challenge the two parts are:

1. **Distance sensing.** Perform the following steps:
   - Start with your robot centred on the wall marker about 2000 mm from the wall.
   - Move towards the wall until you are less than 100 mm from the wall, then move back to 2000 mm.
   - During this, illuminate the LEDs based on the following conditions:
     1. LED B in red when the robot is more than 1500 mm from the wall.
     2. LED B in blue when the robot is between 300 mm and 1500 mm from the wall.
     3. LED B in green when the robot is less than 300 mm from the wall.
   - **Note:** distance should be measured from the centre of the sensor or camera.

2. **Angle sensing.** Perform the following steps:
   - Start with your robot centred on the wall marker and between 1 and 3 metres from the wall.
   - Turn your robot to the left, to the centre, then to the right, then reverse this.
   - During this, illuminate the LEDs based on the following conditions:
     1. LED A in blue when the marker is more than 15° left from square on.
     2. LED B in red when the marker is approximately square on (within a 30° arc).
     3. LED C in blue when the marker is more than 15° right from square on.

The robot may move autonomously or may be moved manually to complete this challenge.

**Note:** if moving the robot manually then it is recommended that the Motor and Servo boards be disconnected from the Power Board as well as any mechanical components secured for the duration of the demonstration.

## Movement Challenge

This challenge is designed to test the robot’s ability to perform repeated movements accurately.

This is a key feature for a robot as, due to manufacturing tolerances, no two motors are exactly the same. As such, your robot must account for the differences in the motors to move in a straight line. This is often achieved by adjusting the power of the motors or, in some cases, by using external references.

To complete this challenge, your robot must:

- Autonomously complete 3 continuous circuits of a triangular path, returning its starting position to within 300 mm.
- The path must be an isosceles right-angled triangle with shorter side length of 1500±200 mm.
- The direction of travel around the path and orientation of the robot are inconsequential.

Teams are encouraged to include in their submission video objects which establish the scale of the path traversed by the robot, for example a metre ruler.

## Mechanics Challenge

This challenge is designed to test the robot’s ability to manipulate objects by lifting one of the objects from the game.

To move samples around, it is useful to be able to lift them. Using a mechanism that physically grabs the sides of the box to be able to pick it up is the most common approach, but they can also be lifted using vacuum suction or scoops that slide under the box.

To complete this challenge, your robot must:

- Lift a sample (as described in [the rulebook](https://studentrobotics.org/docs/resources/2026/rulebook.html#sample-specification)) at least 130 mm off of the ground and hold it there for 5 seconds.

## Sensing Challenge

This challenge is designed to test the robot’s ability to use sensors other than its camera to detect objects in its environment.

To complete this challenge you will need to use a sensor (e.g., ultrasonic distance sensor, infrared distance sensor, bump switch, etc. but not the camera) to detect the presence—or absence—of an object. The object may be a sample box without its markers, a similar sized box, or a heavy book, etc. It must not have a marker on it.

You will need to show that the sensor can detect when the object is present and when it is absent. Light LED B in blue when an object is detected, and in red when an object is not.

To complete this challenge, complete the following steps:

- Hold the object near (within 200 mm of) the sensor.
- Show that LED B lights up in blue.
- Move the object away from the sensor.
- Show that LED B lights up in red.
- Move the object back near (within 200 mm of) the sensor.
- Show that LED B lights up again in blue.

## Simulator Challenge

This challenge is designed to test your ability to program the robot, within the simulator environment.

The simulator reproduces the arena and many of the components of a physical robot. The simulated robots can play the same game as the real robots. In this challenge, you will demonstrate playing the game within the simulator.

To complete this challenge, your simulated robot must successfully bring two samples of the same type back to your laboratory in less than 150 seconds.

You can demonstrate this by either a screen capture video of the simulator running, or videoing the screen if that is not possible.

## Transportation Challenge

This challenge is designed to test the robot’s ability to move samples from place to place.

To score points in the game, robots will need to be able to move samples into their laboratories. This challenge tests that ability.

To complete this challenge, you must mark out two locations on the ground at least 3 m apart. One location is the starting location of a sample, and the other is the target location. Place your robot at least 1 m away from both locations. Your robot must autonomously move a sample from the starting location to within 200 mm of the target location, and then stop.

## Stopping Challenge

This challenge is designed to test the robot’s ability to move precisely.

While it is possible to slowly move from one place to another adjusting as you go, it is very beneficial for a robot to be able to move both accurately and quickly.

To complete this challenge, place your robot 3 m away from an arena wall marker. Your robot must drive towards the wall marker and stop within 100 mm of it without touching it, and it must do so within 5 seconds of starting to move.
