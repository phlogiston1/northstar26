from ctypes import *
lib = cdll.LoadLibrary('/home/seanb/Documents/quadsquad2026/build/libquadcopter.so')

lib.Quadcopter_setHeight.argtypes = [c_void_p, c_double]

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

qc = Quadcopter()

qc.setHeight(1.0)
while(True):
    qc.update_simulation()
    qc.printState()