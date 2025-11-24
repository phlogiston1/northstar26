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
GROUND_STATION_PORT = 8100
QUADCOPTER_PORT = 9100

HAS_MOCAP = False
FULL_POSITION_SIM = True
HAS_IMU = True



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

ground_station = Communication(QUADCOPTER_PORT, GROUND_STATION_PORT, GROUND_STATION_IP)
ground_station.begin()

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



def main():

    led_state = False
    
    
    last_waypoints = []
    last_height = 0
    last_packet_in_index = 0
    arm = False
    
    total_dropped_packets = 0
    packet_out_index = 0
    while(True):
        start_time = time.perf_counter()
        # toggle led for debugging
        led_state = not led_state
        # Bridge.call("set_led_state", led_state)
        # time.sleep(0.5)

        # Pass request from Ground Station to quadcopter control loop:
        (command, new) = ground_station.get_command()
        if new:
            arm = command.arm;
            if command.use_manual:
                if not qc.isManual(): qc.beginManualControl()
                qc.setVelocity(command.velocity)

            if command.has_waypoints:
                if command.waypoints != last_waypoints:
                    for i in command.waypoints:
                        qc.addWaypoint(i["x"], i["y"])
                    qc.beginPath()
                    last_waypoints = command.waypoints

            if command.height != last_height:
                qc.setHeight(command.height)
                last_height = command.height

            if command.pose is not None and HAS_MOCAP:
                qc.addVisionMeasurement(command.pose.translation, qc.getTime() - command.pose_latency)

            index = command.packet_index
            dropped = index - last_packet_in_index
            if last_packet_in_index > index: dropped = (100-last_packet_in_index) + index
            print("Dropped ", dropped, " packets")
            total_dropped_packets += dropped


        # Pass MCU data to C++ Loop
        if HAS_IMU: qc.addIMUMeasurement(Quaternion(imu_yaw,imu_pitch,imu_roll), Vector3D(imu_roll_rate, imu_pitch_rate, imu_yaw_rate))
        qc.setMotorVelocities(left_vel, front_vel, right_vel, rear_vel)

        if FULL_POSITION_SIM:
            qc.updateSimulation();
        else:
            qc.updateKinematics();


        # Pass C++ data to MCU
        request = qc.getRequest()

        ref_pos = request.position().translation()
        ref_vel = request.velocity().translation()
        ref_ang_pos = request.position().rotation()
        ref_ang_vel = request.velocity().rotation()




        cur_pos = qc.getTranslation()
        cur_vel = qc.getVelocity()

        try:
            Bridge.call(
                "a",
                arm)
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
            Bridge.call( # type: ignore
                "s",
                cur_pos.x(),
                cur_pos.y(),
                cur_pos.z(),
                cur_vel.x(),
                cur_vel.y(),
                cur_vel.z())
        except:
            print("WARNING: BRIDGE CALL FAILED")
            pass



        #send data to ground station:
        message = {
            "motor_speeds": [left_vel, front_vel, right_vel, rear_vel],
            "busy": qc.busy(),
            #"target_translation": ref_pos,
            #"target_velocity": {
            #    "x",ref_vel.x(),
            #    "y",ref_vel.y(),
            #    "z",ref_vel.z()
            #},
            #"target_angle": {
            #    "x",ref_ang_pos.x(),
            #    "y",ref_ang_pos.y(),
            #    "z",ref_ang_pos.z()
            #},
            #"target_angular_rate": {
            #    "x",ref_ang_vel.x(),
            #    "y",ref_ang_vel.y(),
            #    "z",ref_ang_vel.z()
            #},
            #"actual_angle": {
            #    "x": imu_roll,
            #    "y": imu_pitch,
            #    "z": imu_yaw
            #},
            "message": "hi",
            "packet_index": packet_out_index
        }

        ground_station.send_message(message)

        packet_out_index += 1
        if packet_out_index > 100: packet_out_index = 0


        #logging for debug:
        print("REQUEST POSITION - x:", request.position().translation().x(), " y: ", request.position().translation().y(), " z: ", request.position().translation().z())
        print("MESSAGE FROM GS:")
        print(ground_station.get_message_json())

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
