import socket
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("host", type=str, help="UR5 IP Address")
parser.add_argument("port", type=int, help="UR5 Port")

args = parser.parse_args()

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(2)
s.connect((args.host, args.port))
print(s.recv(4096).decode(), end="")

while True:
    try:
        cmd = input(">>> ")
        cmd += "\n"

        s.send(cmd.encode())
        print(s.recv(4096).decode(), end="")
    except KeyboardInterrupt:
        break
