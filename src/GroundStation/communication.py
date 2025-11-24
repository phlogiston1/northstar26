import socket, json, time, threading

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
    "actual_angle": {"x":#,"y":#,"z":#}
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
        "latency": ms
    }
    "velocity": {
        "target": {"x":#,"y":#,"z":#},
        "use_velocity": true/false
    },
    "height": #,
    "path": {
        "waypoints": [{"x":#,"y":#},...],
        "begin": true/false
    },
    "packet_index": <0-100> increases every packet to detect losses
}
"""

class Communication:
    def __init__(self, rx_port, tx_port, tx_ip):
        self.rx_port = rx_port
        self.tx_port = tx_port
        self.tx_ip = tx_ip
        self.rx_message = {}
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

    def begin(self):
        threading.Thread(target=self.rx_thread, daemon=True).start()


    def send_message(self, msg):
        self.sock_tx.sendto(json.dumps(msg).encode(), (self.tx_ip, self.tx_port))

    def get_message(self):
        with self.message_lock:
            return self.rx_message
