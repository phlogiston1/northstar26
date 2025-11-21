from ctypes import *
import ctypes
import os
import time

##### TO RUN ON UNO Q:
# lib_path = os.path.join(os.path.dirname(__file__), "libquadcopter.so")
# lib = ctypes.CDLL(lib_path)

##### TO RUN LOCALLY:
lib = cdll.LoadLibrary('/home/seanb/Documents/quadsquad2026/build/libquadcopter.so')

##### Ensure every data type for every function is set, or it won't work on the Uno Q (but may work locally)
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
lib.Quadcopter_new.argtypes = [c_double,c_double,c_double]
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
lib.Quadcopter_getTime.argtypes = [c_void_p]
lib.Quadcopter_getTime.restype = c_double
lib.Quadcopter_getRequest.argtypes = [c_void_p]
lib.Quadcopter_getRequest.restype = c_void_p


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
    def __init__(self, max_velocity, max_acceleration, max_jerk):
        self.ptr = lib.Quadcopter_new(max_velocity, max_acceleration, max_jerk)

    def setHeight(self, height):
        lib.Quadcopter_setHeight(self.ptr, height)

    def update_simulation(self):
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

    def getRequest(self):
        return QCRequest(None, None, lib.Quadcopter_getRequest(self.ptr))


def main():
    qc = Quadcopter(1,1,0.5)
    qc.setHeight(3)
    while True:
        qc.update_simulation()
        req = qc.getRequest()
        print("REQUEST POSITION:")
        print(req.position().translation().x())
        print(req.position().translation().y())
        print(req.position().translation().z())
        time.sleep(0.01)


if __name__ == "__main__":
    main()
