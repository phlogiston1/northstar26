from communication import *

QUADCOPTER_IP = "10.12.34.2"
GROUND_STATION_PORT = 9000
QUADCOPTER_PORT = 9100

coms = Communication(GROUND_STATION_PORT, QUADCOPTER_PORT, QUADCOPTER_IP)
coms.begin()
msg = {"message": "hello","why":10}

while True:
    print(coms.get_message())
    coms.send_message(msg)