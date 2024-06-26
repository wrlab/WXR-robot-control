import sys
import struct
import random
import socket
import time
import json

__version__ = '1.1.1'
ENCODING = 'UTF-8'


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
        if self.rsp is None: return None
        header_format = '!HHBH'
        header_size = struct.calcsize(header_format)
        body = self.rsp[header_size:-3]  # Exclude the 3-byte end marker
        is_ok = self.rsp[-3:]
        debug = True
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

    # Test Code
    # 24.04.19 작성
    # RoboDKsync562.src KRL 프로그램과 연동하여 사용하기 위해 제작
    # $config.dat 에서 선언한 변수와 RoboDKsync562.src KRL 프로그램의 Switch Case를 이용하기 위해 사용
    # 명령 CASE와 명령 값을 매개변수로 하는 메소드
    def write_multiple(self, commands, debug=True):
        """
        여러 변수와 그 값을 한 번에 전송합니다.

        :param commands: 변수 이름과 값을 매핑한 딕셔너리
        :param debug: 디버그 모드 활성화 여부
        """
        if not isinstance(commands, dict):
            raise Exception('Commands must be a dictionary')

        # commands 딕셔너리를 JSON 문자열로 인코딩
        commands_json = json.dumps(commands).encode(ENCODING)
        var_name = "MULTIPLE_COMMANDS".encode(ENCODING)
        value = commands_json
        var_name_len = len(var_name)
        value_len = len(value)
        flag = 1  # write 요청을 나타냄
        req_len = var_name_len + 3 + 2 + value_len

        # 패킷 생성
        req = struct.pack('!HHBH' + str(var_name_len) + 's' + 'H' + str(value_len) + 's',
                          self.msg_id, req_len, flag, var_name_len, var_name, value_len, value)

        # 패킷 전송
        self._send_req(req)
        print("req: ", req)
        print("self._send_req: ", self._send_req(req))
        _value = self._read_rsp(debug)
        if debug:
            print('Debug mode')
            print(_value)
        return _value


    def close(self):
        self.sock.close()
