# THIS FILE IS THE MAIN LOOP THAT RUNS ON THE UNO Q's LINUX PROCESSOR.
# It links QCLib with the MCU on the Uno Q.

# TO RUN:
# Create an App Lab project. Compile QCLib as a dynamic library (I might be annoying and just have multiple
# CMakeLists.txt so just use the right one).
# Copy the libquadcopter.so file from the build folder to the python folder in the app lab project.
# Make sure that the correct library loading method is uncommented in this file (just below the imports)
# As of right now, only run using arduino-app-cli because if you run it in App Lab the application freezes.

from ctypes import *
from arduino.app_utils import *
import ctypes
import os
import time

##### TO RUN ON UNO Q:
lib_path = os.path.join(os.path.dirname(__file__), "libquadcopter.so")
lib = ctypes.CDLL(lib_path)

##### TO RUN LOCALLY:
#lib = cdll.LoadLibrary('/home/seanb/Documents/quadsquad2026/build/libquadcopter.so')

##### CONFIGURATION STUFF:
LOOP_TIME = 0.05



##### Ensure ctypes knows the argument and return types for every function
# or it won't work on the Uno Q (but may work locally)
lib.Vector3D_new.argtypes = [c_double, c_double, c_double]
lib.Vector3D_new.restype = c_void_p
lib.Vector3D_x.argtypes = [c_void_p]
lib.Vector3D_x.restype = c_double
lib.Vector3D_y.argtypes = [c_void_p]
lib.Vector3D_y.restype = c_double
lib.Vector3D_z.argtypes = [c_void_p]
lib.Vector3D_z.restype = c_double
lib.Quaternion_new.argtypes = [c_double,c_double,c_double]
lib.Quaternion_new.restype = c_void_p
lib.Quaternion_w.argtypes = [c_void_p]
lib.Quaternion_w.restype = c_double
lib.Quaternion_x.argtypes = [c_void_p]
lib.Quaternion_x.restype = c_double
lib.Quaternion_y.argtypes = [c_void_p]
lib.Quaternion_y.restype = c_double
lib.Quaternion_z.argtypes = [c_void_p]
lib.Quaternion_z.restype = c_double
lib.Quaternion_getYaw.argtypes = [c_void_p]
lib.Quaternion_getYaw.restype = c_double
lib.Quaternion_getPitch.argtypes = [c_void_p]
lib.Quaternion_getPitch.restype = c_double
lib.Quaternion_getRoll.argtypes = [c_void_p]
lib.Quaternion_getRoll.restype = c_double
lib.Pose3D_new.argtypes = [c_void_p, c_void_p]
lib.Pose3D_new.restype = c_void_p
lib.Pose3D_translation.argtypes = [c_void_p]
lib.Pose3D_translation.restype = c_void_p
lib.Pose3D_rotation.argtypes = [c_void_p]
lib.Pose3D_rotation.restype = c_void_p
lib.QCRequest_new.argtypes = [c_void_p, c_void_p]
lib.QCRequest_new.restype = c_void_p
lib.QCRequest_position.argtypes = [c_void_p]
lib.QCRequest_position.restype = c_void_p
lib.QCRequest_velocity.argtypes = [c_void_p]
lib.QCRequest_velocity.restype = c_void_p
lib.Quadcopter_new.argtypes = []
lib.Quadcopter_new.restype = c_void_p
lib.Quadcopter_setHeight.argtypes = [c_void_p, c_double]
lib.Quadcopter_setHeight.restype = None
lib.Quadcopter_update_simulation.argtypes = [c_void_p]
lib.Quadcopter_update_simulation.restype = None
lib.Quadcopter_printState.argtypes = [c_void_p]
lib.Quadcopter_printState.restype = None
lib.Quadcopter_frontMotorVel.argtypes = [c_void_p]
lib.Quadcopter_frontMotorVel.restype = c_double
lib.Quadcopter_leftMotorVel.argtypes = [c_void_p]
lib.Quadcopter_leftMotorVel.restype = c_double
lib.Quadcopter_rearMotorVel.argtypes = [c_void_p]
lib.Quadcopter_rearMotorVel.restype = c_double
lib.Quadcopter_rightMotorVel.argtypes = [c_void_p]
lib.Quadcopter_rightMotorVel.restype = c_double
lib.Quadcopter_getTranslation.argtypes = [c_void_p]
lib.Quadcopter_getTranslation.restype = c_void_p
lib.Quadcopter_getVelocity.argtypes = [c_void_p]
lib.Quadcopter_getVelocity.restype = c_void_p
lib.Quadcopter_getTime.argtypes = [c_void_p]
lib.Quadcopter_getTime.restype = c_double
lib.Quadcopter_getRequest.argtypes = [c_void_p]
lib.Quadcopter_getRequest.restype = c_void_p
lib.Quadcopter_addWaypoint.argtypes = [c_void_p, c_double, c_double]
lib.Quadcopter_addWaypoint.restype = None
lib.Quadcopter_beginPath.argtypes = [c_void_p]
lib.Quadcopter_beginPath.restype = None
lib.Quadcopter_beginManualControl.argtypes = [c_void_p]
lib.Quadcopter_beginManualControl.restype = None
lib.Quadcopter_setVelocity.argtypes = [c_void_p, c_void_p]
lib.Quadcopter_setVelocity.restype = None
lib.Quadcopter_updateKinematics.argtypes = [c_void_p]
lib.Quadcopter_updateKinematics.restype = None
lib.Quadcopter_setMotorVelocities.argtypes = [c_void_p, c_double, c_double, c_double, c_double]
lib.Quadcopter_setMotorVelocities.restype = None
lib.Quadcopter_addVisionMeasurement.argtypes = [c_void_p, c_void_p, c_double]
lib.Quadcopter_addVisionMeasurement.restype = None
lib.Quadcopter_addIMUMeasurement.argtypes = [c_void_p, c_void_p, c_void_p]
lib.Quadcopter_addIMUMeasurement.restype = None
lib.Quadcopter_busy.argtypes = [c_void_p]
lib.Quadcopter_busy.restype = c_bool


##### BINDING CLASSES
# These classes bind to the equivalent C++ classes.
class Vector3D(object):
    def __init__(self, x=0, y=0, z=0, ptr=None):
        if ptr is not None:
            self.ptr = ptr
        else:
            self.ptr = lib.Vector3D_new(x,y,z)

    def x(self):
        return lib.Vector3D_x(self.ptr)

    def y(self):
        return lib.Vector3D_y(self.ptr)

    def z(self):
        return lib.Vector3D_z(self.ptr)


class Quaternion(object):
    def __init__(self, yaw=0, pitch=0, roll=0, ptr=None):
        if ptr is not None:
            self.ptr = ptr
        else:
            self.ptr = lib.Quaternion_new(yaw,pitch,roll)

    def w(self):
        return lib.Quaternion_w(self.ptr)
    def x(self):
        return lib.Quaternion_x(self.ptr)
    def y(self):
        return lib.Quaternion_y(self.ptr)
    def z(self):
        return lib.Quaternion_z(self.ptr)
    def getYaw(self):
        return lib.Quaternion_getYaw(self.ptr)
    def getPitch(self):
        return lib.Quaternion_getPitch(self.ptr)
    def getRoll(self):
        return lib.Quaternion_getRoll(self.ptr)

class Pose3D(object):
    def __init__(self, translation: Vector3D, rotation: Quaternion, ptr=None):
        if ptr is not None:
            self.ptr = ptr
        else:
            self.ptr = lib.Pose3D_new(translation.ptr, rotation.ptr)

    def translation(self):
        return Vector3D(ptr=lib.Pose3D_translation(self.ptr))

    def rotation(self):
        return Quaternion(ptr=lib.Pose3D_rotation(self.ptr))

class QCRequest(object):
    def __init__(self, position: Pose3D, velocity: Pose3D, ptr=None):
        if ptr is not None:
            self.ptr = ptr
        else:
            self.ptr = lib.QCRequest_new(position.ptr, velocity.ptr)

    def position(self):
        return Pose3D(None, None, lib.QCRequest_position(self.ptr))

    def velocity(self):
        return Pose3D(None, None, lib.QCRequest_velocity(self.ptr))


class Quadcopter(object):
    def __init__(self):
        self.ptr = lib.Quadcopter_new()

    def setHeight(self, height):
        lib.Quadcopter_setHeight(self.ptr, height)

    def updateSimulation(self):
        lib.Quadcopter_update_simulation(self.ptr)

    def printState(self):
        lib.Quadcopter_printState(self.ptr)

    def getTime(self):
        return lib.Quadcopter_getTime(self.ptr)

    def frontMotorVel(self):
        return lib.Quadcopter_frontMotorVel(self.ptr)

    def leftMotorVel(self):
        return lib.Quadcopter_leftMotorVel(self.ptr)

    def rightMotorVel(self):
        return lib.Quadcopter_rightMotorVel(self.ptr)

    def rearMotorVel(self):
        return lib.Quadcopter_rearMotorVel(self.ptr)

    def getTranslation(self):
        return Vector3D(ptr=lib.Quadcopter_getTranslation(self.ptr))

    def getVelocity(self):
        return Vector3D(ptr=lib.Quadcopter_getVelocity(self.ptr))

    def getRequest(self):
        return QCRequest(None, None, lib.Quadcopter_getRequest(self.ptr))

    def addWaypoint(self, x, y):
        lib.Quadcopter_addWaypoint(self.ptr, x, y)

    def beginPath(self):
        lib.Quadcopter_beginPath(self.ptr)

    def beginManualControl(self):
        lib.Quadcopter_beginManualControl(self.ptr)

    def setVelocity(self, velocity: Vector3D):
        lib.Quadcopter_setVelocity(self.ptr, velocity.ptr)

    def updateKinematics(self):
        lib.Quadcopter_updateKinematics(self.ptr)

    def setMotorVelocities(self, left, front, right, rear):
        lib.Quadcopter_setMotorVelocities(self.ptr, left, front, right, rear)

    def addVisionMeasurement(self, translation: Vector3D, timestamp):
        lib.Quadcopter_addVisionMeasurement(self.ptr, translation.ptr, timestamp)

    def addIMUMeasurement(self, angle: Quaternion, angular_rate: Vector3D):
        lib.Quadcopter_addIMUMeasurement(self.ptr, angle.ptr, angular_rate.ptr)


    def busy(self):
        return lib.Quadcopter_busy(self.ptr)



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

def recieve_front_vel(velocity):
    global front_vel
    front_vel = velocity

def recieve_right_vel(velocity):
    global right_vel
    right_vel = velocity

def recieve_rear_vel(velocity):
    global rear_vel
    rear_vel = velocity

def recieve_left_vel(velocity):
    global left_vel
    left_vel = velocity

def recieve_imu(y,p,r,yr,pr,rr):
    global imu_yaw
    global imu_pitch
    global imu_roll
    global imu_yaw_rate
    global imu_pitch_rate
    global imu_roll_rate
    imu_yaw = y
    imu_pitch = p
    imu_roll = r
    imu_yaw_rate = yr
    imu_pitch_rate = pr
    imu_roll_rate = rr

Bridge.provide("recieve_front_vel", recieve_front_vel)
Bridge.provide("recieve_right_vel", recieve_right_vel)
Bridge.provide("recieve_rear_vel", recieve_rear_vel)
Bridge.provide("recieve_left_vel", recieve_left_vel)


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
        Bridge.call(
            "set_reference",
            ref_pos.x(),
            ref_pos.y(),
            ref_pos.z(),
            ref_vel.x(),
            ref_vel.y(),
            ref_vel.z(),
            ref_ang_pos.getPitch(),
            ref_ang_pos.getRoll(),
            ref_ang_pos.getYaw(),
            ref_ang_vel.getPitch(),
            ref_ang_vel.getRoll(),
            ref_ang_vel.getYaw())

        cur_pos = qc.getTranslation()
        cur_vel = qc.getVelocity()
        Bridge.call(
            "set_current",
            cur_pos.x(),
            cur_pos.y(),
            cur_pos.z(),
            cur_vel.x(),
            cur_vel.y(),
            cur_vel.z())

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
