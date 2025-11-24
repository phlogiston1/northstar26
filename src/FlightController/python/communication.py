import socket, json, time, threading
from quadcopter import Pose3D, Vector3D, Quaternion

# QC_PORT = 9000
# GS_PORT = 9001
# GS_IP = "192.168.4.2"

"""
FORMAT OF MESSAGE FROM QUADCOPTER:
{
    "motor_speeds": [le,fr,ri,re],
    "busy": true/false,
    "target_translation": {"x":#,"y":#,"z":#},
    "target_velocity": {"x":#,"y":#,"z":#},
    "target_angle": {"x":#,"y":#,"z":#},
    "target_angular rate": {"x":#,"y":#,"z":#},
    "actual_angle": {"x":#,"y":#,"z":#},
    "message": "",
    "packet_index": <0-100>
}

FORMAT OF MESSAGE FROM GROUND STATION:
{
    "arm": true/false,
    "pose": {
        "x":#,
        "y":#,
        "z":#,
        "roll":#,
        "pitch":#,
        "yaw":#
        "latency": seconds
    }
    "manual_velocity": {
        "target": {"x":#,"y":#,"z":#},
        "use": true/false
    },
    "height": #,
    "path_waypoints": [{"x":#,"y":#},...],
    "packet_index": <0-100> increases every packet to detect losses
}
"""



class QuadcopterCommand:
    def __init__(self, json):
        try:
            self.arm = json["arm"]
            self.pose = Pose3D(
                Vector3D(
                    json["pose"]["x"],
                    json["pose"]["y"],
                    json["pose"]["z"]
                ),
                Quaternion(
                    json["pose"]["yaw"],
                    json["pose"]["pitch"],
                    json["pose"]["roll"]
                )
            )
            self.pose_latency = json["pose"]["latency"]
            self.velocity = Vector3D(
                json["manual_velocity"]["target"]["x"],
                json["manual_velocity"]["target"]["y"],
                json["manual_velocity"]["target"]["z"]
            )
            self.use_manual = json["manual_velocity"]["use"]
            self.height = json["height"]
            self.waypoints = json["path_waypoints"]
            self.has_waypoints = self.waypoints.__len__ > 0
            self.packet_index = json["packet_index"]
            self.valid = True
        except KeyError:
            print("COMS ERROR: Invalid packet from ground station: ")
            print(json)
            print("Using default command")
            self.arm = False
            self.pose = None
            self.pose_latency = 0
            self.velocity = Vector3D(0,0,0)
            self.use_manual = True
            self.height = 0
            self.waypoints = []
            self.has_waypoints = False
            self.packet_index = -1
            self.valid = False
            pass




class Communication:
    def __init__(self, rx_port, tx_port, tx_ip):
        self.rx_port = rx_port
        self.tx_port = tx_port
        self.tx_ip = tx_ip
        self.rx_message = {}
        self.command = QuadcopterCommand({})
        self.has_new_json = False
        self.has_new_command = False
        self.message_lock = threading.Lock()
        self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def rx_thread(self):
        sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock_rx.bind(("0.0.0.0", self.rx_port))

        while True:
            msg, addr = sock_rx.recvfrom(1024)
            data = json.loads(msg)

            with self.message_lock:
                self.rx_message = data
                self.command = QuadcopterCommand(data)
                self.has_new_json = True
                self.has_new_command = self.command.valid

    def begin(self):
        threading.Thread(target=self.rx_thread, daemon=True).start()


    def send_message(self, msg):
        self.sock_tx.sendto(json.dumps(msg).encode(), (self.tx_ip, self.tx_port))

    def get_message_json(self):
        with self.message_lock:
            has_new_cpy = self.has_new_json
            self.has_new_json = False
            return (self.rx_message, has_new_cpy)

    def get_command(self):
        with self.message_lock:
            has_new_cpy = self.has_new_command
            self.has_new_command = False
            return (self.command, has_new_cpy)
