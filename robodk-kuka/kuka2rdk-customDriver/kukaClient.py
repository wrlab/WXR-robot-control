import sys
import struct
import random
import socket
import time


class kukaClient:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.msg_id = random.randint(1, 100)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((self.ip, self.port))
        except socket.error as e:
            print(f"Socket error: {e}")
            sys.exit(-1)

    def test_connection(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            ret = sock.connect_ex((self.ip, self.port))
            sock.close()
            return ret == 0
        except socket.error as e:
            print(f"Socket error: {e}")
            return False

    @property
    def can_connect(self):
        return self.test_connection()

    def read(self, var, debug=True):
        if not isinstance(var, str):
            raise TypeError('Var name must be a string')
        self.varname = var.encode('utf-8')
        return self._read_var(debug)

    def _send_req(self, req):
        self.rsp = None
        self.sock.sendall(req)
        self.rsp = self.sock.recv(256)

    def _pack_read_req(self):
        req_len = len(self.varname) + 3
        return struct.pack('!HHBH' + str(len(self.varname)) + 's',
                           self.msg_id,
                           req_len,
                           0,
                           len(self.varname),
                           self.varname)

    def _read_var(self, debug):
        # 패킷 생성
        req = self._pack_read_req()
        # 패킷 전송
        self._send_req(req)
        return self._read_rsp(debug)

    def _read_rsp(self, debug=False):
        if self.rsp is None: return None
        header_format = '!HHBH'
        header_size = struct.calcsize(header_format)
        body = self.rsp[header_size:-3]  # Exclude the 3-byte end marker
        is_ok = self.rsp[-3:]
        if debug:
            print('Response:', self.rsp)
        # else:
        #     self.msg_id = (self.msg_id + 1) % 65536
        #     return self.rsp.decode('utf-8')
        if is_ok.endswith(b'\x01'):
            self.msg_id = (self.msg_id + 1) % 65536
            return body.decode('utf-8')

    def loop_read_var(self, var, interval=1):
        try:
            while True:
                response = self.read(var, debug=False)
                if response is not None:
                    print(f"서버로부터 받은 응답: {response}")
                time.sleep(interval)  # 다음 요청 전에 지연
        except KeyboardInterrupt:
            print("지속적인 읽기 중지.")

    def close(self):
        self.sock.close()
