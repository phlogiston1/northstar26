# HI NORTHSTAR
Lets make some cool stuff!

# COORDINATE SYSTEM!!
## NEW COORDINATE SYSTEM:
+Y = forward, in the direction of rotor 2
+X = rightward, in direction of rotor 3
+Z = upward

         2 (Y axis)
         |
     1---O---3 (X axis)
         |
         4

+pitch = quadcopter angles to the left
+roll = quadcopter angles to the back
+yaw = quadcopter rotates counterclockwise

IMPORTANT: SOFTWARE ASSUMES THAT ROTORS 1 AND 3 SPIN CLOCKWISE.


## ORIGINAL COORDINATE SYSTEM:
+X = forward motion
+Y = left motion
+Z = downward motion

yaw is ccw+

# Other dependencies:
- considering using Foxglove for visualization


# Code structure:
simulate.cpp - runs a physics simulation using forward and inverse kinematics to sanity-check control software
test.cpp - arbitrary testing code

Eventually:
flight_control.cpp - the flight controller loop for the onboard linux processor
hardware_control.cpp - the main control loop for the onboard real-time processor
base_station.cpp - the control loop for the base computer

## QCLib - does all the math to control the quadcopter
 - Configuration: All of the configurable/tunable numbers go here
 - Quadcopter: This file has classes that represent the entire frame of the quadcopter
   - In my dream world this is the only file that should be interacted with outside of the QCLib folder.
   - It should have 4 functions that can control the entire system:
     - update(IMU data, motor speeds, timestamp) - update forward odometry
     - addPositionObservation(Position, timestamp) - provide a position observation from the groud station
     - setGoal(Position/Velocity) - set either the target position or velocity of the copter
     - calculateMotorVelocities() - get the desired velocity of the motors
 - Kinematics: This file is responsible for calculating the acceleration of the quadcopter given the known information about it's state (the quadcopter's angle and motor speeds)
 - InverseKinematics: Does the opposite of Kinematics - calculates the quadcopter's state needed to attain a desired acceleration.
 - MotionController: This file is responsible for figuring out what the desired acceleration of the quadcopter is based on the desired velocity or position of the quadcopter.
 - 

## Eventually:
## FlightController - eventually connects QCLib to the hardware on the quadcopter
## MissionControl - software that runs on the control computer
## Misisons - defines the actual autonomous/manual behaviors of the quadcopter

