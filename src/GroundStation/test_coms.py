from communication import *

QUADCOPTER_IP = "192.168.4.2"
GROUND_STATION_PORT = 9000
QUADCOPTER_PORT = 9001

coms = Communication(GROUND_STATION_PORT, QUADCOPTER_PORT, QUADCOPTER_IP)
coms.begin()

while True:
    coms.send_message({"message": "hi"})
    print(coms.get_message())