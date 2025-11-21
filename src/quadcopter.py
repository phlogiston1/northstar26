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
lib.Quadcopter_setHeight.argtypes = [c_void_p, c_double]
lib.Quadcopter_setHeight.restype = None

lib.Quadcopter_frontMotorVel.argtypes = [c_void_p]
lib.Quadcopter_frontMotorVel.restype = c_double

lib.Quadcopter_new.argtypes = []
lib.Quadcopter_new.restype = c_void_p

lib.Quadcopter_update_simulation.argtypes = [c_void_p]
lib.Quadcopter_update_simulation.restype = None

lib.Quadcopter_printState.argtypes = [c_void_p]
lib.Quadcopter_printState.restype = None

class Quadcopter(object):
    def __init__(self):
        self.obj = lib.Quadcopter_new()

    def setHeight(self, height):
        lib.Quadcopter_setHeight(self.obj, height)

    def update_simulation(self):
        lib.Quadcopter_update_simulation(self.obj)

    def printState(self):
        lib.Quadcopter_printState(self.obj)

    def getTime(self):
        return lib.Quadcopter_getTime(self.obj)

    def frontMotorVel(self):
        return lib.Quadcopter_frontMotorVel(self.obj)

    def leftMotorVel(self):
        return lib.Quadcopter_leftMotorVel(self.obj)

    def rightMotorVel(self):
        return lib.Quadcopter_rightMotorVel(self.obj)

    def rearMotorVel(self):
        return lib.Quadcopter_rearMotorVel(self.obj)


def main():
    print("1")
    qc = Quadcopter()
    print(2)
    # qc.setHeight(1.0)
    # while(True):
    print(qc.frontMotorVel())
    qc.update_simulation()
    qc.printState()
    time.sleep(0.5)


if __name__ == "__main__":
    main()
