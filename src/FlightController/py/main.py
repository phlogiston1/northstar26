# THIS FILE IS THE MAIN LOOP THAT RUNS ON THE UNO Q's LINUX PROCESSOR.
# It links QCLib with the MCU on the Uno Q.

# TO RUN:
# Create an App Lab project. Compile QCLib as a dynamic library (I might be annoying and just have multiple
# CMakeLists.txt so just use the right one).
# Copy the libquadcopter.so file from the build folder to the python folder in the app lab project.
# Make sure that the correct library loading method is uncommented in this file (just below the imports)
# As of right now, only run using arduino-app-cli because if you run it in App Lab the application freezes.

from ctypes import *
from arduino.app_utils import * # type: ignore
import time
from quadcopter import *
from communication import *

##### CONFIGURATION STUFF:
LOOP_TIME = 0.05

GROUND_STATION_IP = "10.12.34.1"
GROUND_STATION_PORT = 9000
QUADCOPTER_PORT = 9100



##### START OF ACTUAL CONTROL CODE

# These variables are updated BY THE MCU
# The functions below are called via. the Bridge
# To keep them up to date
front_vel = 0
right_vel = 0
rear_vel = 0
left_vel = 0
imu_yaw = 0
imu_pitch = 0
imu_roll = 0
imu_yaw_rate = 0
imu_pitch_rate = 0
imu_roll_rate = 0

def recieve_data(left, front, right, rear, y,p,r,yr,pr,rr):
    global front_vel
    global right_vel
    global rear_vel
    global left_vel
    global imu_yaw
    global imu_pitch
    global imu_roll
    global imu_yaw_rate
    global imu_pitch_rate
    global imu_roll_rate
    left_vel = left
    front_vel = front
    right_vel = right
    rear_vel = rear
    imu_yaw = y
    imu_pitch = p
    imu_roll = r
    imu_yaw_rate = yr
    imu_pitch_rate = pr
    imu_roll_rate = rr


Bridge.provide("r", recieve_data) # type: ignore

coms = Communication(QUADCOPTER_PORT, GROUND_STATION_PORT, GROUND_STATION_IP)
coms.begin()

# wait to make sure arduino loop is running
time.sleep(5)

# Initialization
qc = Quadcopter()
qc.setHeight(1)
qc.addWaypoint(0,0)
qc.addWaypoint(1,1)
qc.addWaypoint(2,1)
qc.addWaypoint(0,0)
qc.beginPath()
led_state = False

def main():
    while(True):
        start_time = time.perf_counter()
        # toggle led for debugging
        global led_state
        led_state = not led_state
        # Bridge.call("set_led_state", led_state)
        # time.sleep(0.5)

        # Pass MCU data to C++ Loop
        # qc.addIMUMeasurement(Quaternion(imu_yaw,imu_pitch,imu_roll), Vector3D(imu_roll_rate, imu_pitch_rate, imu_yaw_rate))
        qc.setMotorVelocities(left_vel, front_vel, right_vel, rear_vel)

        # Call C++ Main Loop
        qc.updateSimulation();


        # Pass C++ data to MCU
        request = qc.getRequest()

        ref_pos = request.position().translation()
        ref_vel = request.velocity().translation()
        ref_ang_pos = request.position().rotation()
        ref_ang_vel = request.velocity().rotation()
        Bridge.call( # type: ignore
            "p",
            ref_pos.x(),
            ref_pos.y(),
            ref_pos.z(),
            ref_ang_pos.getRoll(),
            ref_ang_pos.getPitch(),
            ref_ang_pos.getYaw())
        Bridge.call( # type: ignore
            "v",
            ref_vel.x(),
            ref_vel.y(),
            ref_vel.z(),
            ref_ang_vel.getRoll(),
            ref_ang_vel.getPitch(),
            ref_ang_vel.getYaw())

        cur_pos = qc.getTranslation()
        cur_vel = qc.getVelocity()
        Bridge.call( # type: ignore
            "s",
            cur_pos.x(),
            cur_pos.y(),
            cur_pos.z(),
            cur_vel.x(),
            cur_vel.y(),
            cur_vel.z())
        


        #send data to ground station:
        message = {
            "motor_speeds": [left_vel, front_vel, right_vel, rear_vel],
            "busy": qc.busy(),
            "target_translation": ref_pos,
            "target_velocity": ref_vel,
            "target_angle": ref_ang_pos,
            "target_angular_rate": ref_ang_vel,
            "actual_angle": {
                "x": imu_roll,
                "y": imu_pitch,
                "z": imu_yaw
            },
            "message": "hi",
            "packet_index": 0 #todo
        }

        coms.send_message(message)



        #logging for debug:
        print("REQUEST POSITION:")
        print(request.position().translation().x())
        print(request.position().translation().y())
        print(request.position().translation().z())
        print("MOTOR SPEED FROM PY:")
        print("front: ", front_vel)
        print("right: ", right_vel)
        print("rear: ", rear_vel)
        print("left: ",left_vel)
        print("message:")
        print(coms.get_message())

        # manage loop time:
        end_time = time.perf_counter()
        elapsed_time = end_time - start_time
        sleep_duration = LOOP_TIME - elapsed_time

        if sleep_duration > 0:
            time.sleep(sleep_duration)
        else:
            print("PYTHON LOOP OVERRUN. LOOP TOOK AN ADDITIONAL ", -sleep_duration, " SECONDS.")
            pass


if __name__ == "__main__":
    main()
