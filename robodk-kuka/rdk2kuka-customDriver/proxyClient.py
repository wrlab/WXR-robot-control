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
        else:
            self.varname = var.encode(ENCODING)
        return self._read_var(debug)

    def _read_var(self, debug):
        # 패킷 생성
        req = self._pack_read_req()
        # 패킷 전송
        self._send_req(req)
        _value = self._read_rsp(debug)
        if debug:
            print(_value)
        return _value

    def _send_req(self, req):
        self.rsp = None
        self.sock.sendall(req)
        self.rsp = self.sock.recv(256)

    def _pack_read_req(self):
        var_name_len = len(self.varname)
        flag = 0
        req_len = var_name_len + 3

        return struct.pack('!HHBH' + str(var_name_len) + 's',
                           self.msg_id,
                           req_len,
                           flag,
                           var_name_len,
                           self.varname)

    def _read_rsp(self, debug=False):
        if self.rsp is None:
            return None
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

#############################################################################
## 2024.04.16 ##
#############################################################################
    def write(self, var, value, debug=True):
        if not (isinstance(var, str) and isinstance(value, str)):
            raise Exception('Var name and its value should be string')
        self.varname = var.encode(ENCODING)
        self.value = value.encode(ENCODING)
        return self._write_var(debug)

    def _write_var(self, debug):
        req = self._pack_write_req()
        self._send_req(req)
        _value = self._read_rsp(debug)
        if debug:
            print(_value)
        return _value

    def _pack_write_req(self):
        var_name_len = len(self.varname)
        flag = 1
        value_len = len(self.value)
        req_len = var_name_len + 3 + 2 + value_len

        return struct.pack(
            '!HHBH' + str(var_name_len) + 's' + 'H' + str(value_len) + 's',
            self.msg_id,
            req_len,
            flag,
            var_name_len,
            self.varname,
            value_len,
            self.value
        )

#############################################################################
## 2024.05.20 ##
#############################################################################

    def move(self, motion_type, position, debug=True):
        if not isinstance(position, str):
            raise TypeError('포지션 값은 문자열이어야 합니다.')
        self.position = position.encode('utf-16le')
        # Motion Type:
            # 1 - PTP
            # 2 - LIN
            # 3 - PTP_REL
            # 4 - LIN_REL
        self.motion_type = motion_type
        return self._move_robot(debug)

    def _move_robot(self, debug):
        req = self._pack_move_req()
        self._send_req(req)
        _value = self._read_rsp(debug)
        if debug:
            print(_value)
        return _value


    def _pack_move_req(self):
        position_len = len(self.position) // 2 # UTF-16LE는 2바이트씩 사용
        flag = 11
        req_len = 3 + 2 + position_len

        return struct.pack(
            '!HHBHB' + str(position_len * 2) + 's',
            self.msg_id,
            req_len,
            flag,
            self.motion_type,
            position_len,
            self.position
        )

    def close(self):
        self.sock.close()


# 사용 예제
if __name__ == "__main__":
    ip = '192.168.1.100'  # C3 Bridge Interface 서버 IP 주소
    port = 7000

    client = kukaClient(ip, port)

    if client.can_connect:
        print("서버에 연결 성공")

        # 변수 읽기 예제
        response = client.read('$POS_ACT', debug=True)
        print(f"현재 위치: {response}")

        # 변수 쓰기 예제
        client.write('$OV_PRO', '50', debug=True)

        # 로봇 이동 명령 예제 (PTP 이동)
        position = "{A1 0, A2 -90, A3 90, A4 0, A5 0, A6 0}"
        client.move(1, position, debug=True)

        # 로봇 이동 명령 예제 (PTP_REL 이동)
        position = "{X 10, Y 0, Z 0, A 0, B 0, C 0}"
        client.move(3, position, debug=True)
    else:
        print("서버에 연결 실패")

    client.close()
