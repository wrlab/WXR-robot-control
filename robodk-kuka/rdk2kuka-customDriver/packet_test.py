import sys
import struct
import random
import socket
import time
from rdk_config import Config_host

__version__ = '1.1.0'
ENCODING = 'UTF-8'

# Message Tpyes (flag)
# 0 - CommandReadVariableAscii
# 1 - CommandWriteVariableAscii
# 11 - CommandMotion

class kukaClient:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.msg_id = random.randint(1, 100)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def move(self, motion_type, position, debug=True):
        if not isinstance(position, str):
            raise TypeError('Position values must be STRING.')
        print("Position (original): ", position)
        self.position = position.encode('utf-16le')  # Wide format (UTF-16LE)
        self.motion_type = motion_type
        print("Position (encoded): ", self.position.hex())  # Print encoded position in hex for debugging
        return self._move_robot(debug)

    def _move_robot(self, debug):
        req = self._pack_move_req()
        print("req ready!")
        print("req: ", req.hex())
        self._send_req(req)
        print("send ok")
        _value = self._read_rsp_move(debug)
        if debug:
            print(_value)
        return _value

    def _pack_move_req(self):
        position_len = len(self.position) // 2
        flag = 11  # Wide format version of CommandMotion
        req_len = 3 + 2 + len(self.position)

        return struct.pack(
            '!HHBBH' + str(len(self.position)) + 's',
            self.msg_id,  # Tag ID
            req_len,  # Message Length
            flag,  # Message Type
            self.motion_type,  # Motion Type
            position_len,  # Length of Position String (in characters)
            self.position  # Positon String
        )

    def _send_req(self, req):
        print("_send_req: ", req)
        self.rsp = None
        try:
            self.sock.sendall(req)
            print("sock.sendall(req)")
            self.rsp = self.sock.recv(1024)
            print("self.rsp: ", self.rsp)
        except socket.timeout:
            print("Socket timeout occurred. No response received.")
        except socket.error as e:
            print(f"Socket error: {e}")

    def _read_rsp_move(self, debug=False):
        if self.rsp is None:
            return None
        header_format = '!HHBH'
        header_size = struct.calcsize(header_format)
        body = self.rsp[header_size:-3]  # Exclude the 3-byte end marker
        is_ok = self.rsp[-3:]
        if debug:
            print('Response:', self.rsp)
        if is_ok.endswith(b'\x01'):
            self.msg_id = (self.msg_id + 1) % 65536
            return body.decode('utf-16le')

if __name__ == "__main__":
    ip = '172.31.2.147'  # C3 Bridge Interface 서버 IP 주소
    port = 7000
    print("Sada")
    #cfg_host = {"HOST": "172.31.2.147", "PORT": 7000}
    client = kukaClient(ip, port)
    # 로봇 이동 명령 예제 (PTP 이동)
    #position = '{A1 0, A2 -90, A3 90, A4 0, A5 0, A6 0}'
    position = '{POS: X 0, Y 0, Z 0, A 0, B 0, C 0}'
    # position = '{X 1600.00, Y 70.00, Z 500.00, A 0, B 0, C 0}' #'{POS: X 1600.00, Y 70.00, Z 500.00, A 0, B 0, C 0}'
    client.move(1, position, debug=True)

    client.close()