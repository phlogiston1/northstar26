import socket
import threading
import subprocess
import json

# ---- CONFIG ----
HOST_IN_PORT = 9100       # Port the ground station sends to (host side)
# CONTAINER_IP = "172.18.0.2"  # Replace with your App Lab container IP
CONTAINER_PORT = 9100     # Port your Python app listens on

CONTAINER_IN_PORT = 8100  # Port Python app sends from
GROUND_IP = "10.12.34.1"  # Ground station
GROUND_PORT = 8100        # Port ground station listens on

# sudo docker ps -a
# sudo docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' <container-id>

def find_container_ip():
    container_ids = subprocess.check_output([
        "sudo", "docker", "ps",
        "--filter", "name=flightcontroller",
        "--format", "{{.ID}}"
    ]).decode().strip().splitlines()

    if not container_ids:
        raise RuntimeError("No App Lab container found")

    info = subprocess.check_output(
        ["sudo", "docker", "inspect", container_ids[0]]
    )
    return json.loads(info)[0]["NetworkSettings"]["IPAddress"]

CONTAINER_IP = find_container_ip()
print("Found container IP: ", CONTAINER_IP)

def forward_host_to_container():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", HOST_IN_PORT))
    print(f"[relay] host → container on {HOST_IN_PORT}")

    while True:
        data, addr = sock.recvfrom(4096)
        sock.sendto(data, (CONTAINER_IP, CONTAINER_PORT))


def forward_container_to_ground():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", CONTAINER_IN_PORT))
    print(f"[relay] container → ground on {CONTAINER_IN_PORT}")

    while True:
        data, addr = sock.recvfrom(4096)
        sock.sendto(data, (GROUND_IP, GROUND_PORT))


t1 = threading.Thread(target=forward_host_to_container, daemon=True)
t2 = threading.Thread(target=forward_container_to_ground, daemon=True)

t1.start()
t2.start()

print("[relay] running...")
t1.join()
t2.join()
