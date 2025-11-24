from communication import *
import time

QUADCOPTER_IP = "10.12.34.2"
GROUND_STATION_PORT = 8100
QUADCOPTER_PORT = 9100

coms = Communication(GROUND_STATION_PORT, QUADCOPTER_PORT, QUADCOPTER_IP)
coms.begin()
msg = {
    "arm": True,
    "pose": {
        "x":0,
        "y":0,
        "z":0,
        "roll":0,
        "pitch":0,
        "yaw":0,
        "latency":0
    },
    "manual_velocity": {
        "target": {
            "x":1,
            "y":0,
            "z":0
        },
        "use": False
    },
    "height": 2,
    "path_waypoints": [],
    "packet_index": 0 # increases every packet to detect losses
}

while True:
    print(coms.get_message())
    coms.send_message(msg)
    time.sleep(0.1)