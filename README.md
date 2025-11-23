# NORTHSTAR SUPER SECRET QUADCOPTER PROJECT (work in progress)
This is a custom fully autonomous quadcopter control stack based on
infrared motion capture, inspired by https://github.com/jyjblrd/Low-Cost-Mocap.
The motion capture runs on an offboard computer and is streamed via UDP
to an Arduino Uno Q on the quadcopter, which handles all onboard tasks (stabilization, additional mechs, etc.)


# PROJECT ARCHITECTURE:
The control system is divided between three different processors: the offboard computer,
the linux processor on the Uno Q, and the MCU on the Uno Q.

## Here's the basic pipeline:
1. Base computer captures camera feeds, triangulates quadcopter location.
2. Linux processor recieves MoCap pose & instructions for what autonomous actions to take
3. Linux processor loop runs Kinematics predictions to compute translation in between MoCap updates
4. Linux processor calculates motion profiling for quadcopter and passes the current & target pose and velocity to the MCU
5. MCU loop uses Linear Quadratic Regulator and IMU data to stabilize drone at target pose and velocity
6. MCU loop passes IMU data and motor speed back to linux processor for kinematics.

## Directories:
### src/FlightController
This is the software that runs on the Arduino Uno Q. To run it, first create an App Lab project on the Uno. Then, compile QCLib using cmake to get a libquadcopter.so file in the build directory. Next, copy both the libquadcopter.so file and the contents of src/FlightController/py to the "py" folder of the App Lab project. Also, copy the Arduino sketch from the src/FlightController/cpp directory to the App Lab project. Then, run the project using arudino-app-cli app start "/path/to/your/app" and view logs using arduino-app-cli app logs "/path/to/your/app" --all.

**IMPORTANT**: The Bridge communication library runs via serial and is quite slow by default. In order to run the flight controller software, you need to increase the baud rate of the bridge library as per this video (in a different language with 20 views): https://www.youtube.com/watch?v=vWpq636f1Z4
Specifically, increase the baud rate in the /etc/systemd/system/arduino-bridge.system file.

### src/QCLib
This is a helpful C++ library I wrote that handles a majority of the quadcopter control. It is primarily used by the flight controller linux program. Because App Lab only supports python on the Linux processor (as of right now), QCLib is used through ctypes.

QCLib has theses files:
- Configuration: just what it sounds like.
- Kinematics: predicts the quadcopter's motion based on it's current motor velocities
- InverseKinematics: originally did more but now it just determines the quadcopter's ideal orientation to attain a desired acceleration
- LQR: Linear Quadratic Regulator used for simulation but not on final drone. Functionally identical to the LQR in the flight controller cpp code.
- MotionController: Responsible for calculating the reference position and velocity of the drone for the control loop, based off of either a target position, target height, or target velocity.
- Path: Calculates 2D paths through waypoints based on Bezier curves, time-parameterized to respect velocity, acceleration, and jerk constraints
- Physics: Resposible for estimating the change in the quadcopter's position/velocity based on the accelerations calculated by kinematics.
- Quadcopter: the primary interface with QCLib. Designed to fully handle motion control of the quadcopter based on a goal from the offboard computer.
- Util: just what it sounds like

### src/GroundControl
This doesn't have anything yet, but it will have mocap and sequencing of autonomous plans.


# OTHER IMPORTANT INFO:
## COORDINATE SYSTEM:
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


## Dependencies:
- Foxglove for visualization. Handled by CMake - vurrently, use CMakeLists_simulation to build the simulation with foxglove. Note, this doesn't build the shared library needed by the flight control code.

