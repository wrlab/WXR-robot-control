import sys
import struct
import random
import socket
import time

__version__ = '1.1.0'
ENCODING = 'UTF-8'
ASCII = 'ascii'
LATIN = 'latin-1'

# 인코딩 설정 (문서상 string=ascii/latin-1, wstring=utf-16le)
ENCODING = 'UTF-8'
ASCII = 'ascii'
LATIN = 'latin-1'

# Message Types
COMMAND_READ_VARIABLE = 0     # CommandReadVariableAscii
COMMAND_WRITE_VARIABLE = 1    # CommandWriteVariableAscii
COMMAND_PROGRAM_CONTROL = 10  # ProgramControl
COMMAND_MOTION = 11           # CommandMotion

# ProgramControl Command Codes
CMD_RESET = 1
CMD_START = 2
CMD_STOP = 3
CMD_CANCEL = 4

# Interpreter Types
SUBMIT_INTERPRETER = 0
ROBOT_INTERPRETER = 1

# Motion Types
PTP = 1
LIN = 2
PTP_REL = 3
LIN_REL = 4

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
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            ret = sock.connect_ex((self.ip, self.port))
            sock.close()
            return ret == 0
        except socket.error as e:
            print(f"Socket error: {e}")
            return False

    @property
    def can_connect(self):
        return self.test_connection()

    def close(self):
        self.sock.close()

#############################################################################
## Read System Variables ##
#############################################################################
    def read(self, var, debug=True):
        """KUKA 시스템 변수를 읽는다. var는 문자열."""
        if not isinstance(var, str):
            raise TypeError('Var name must be a string')
        self.varname = var.encode(ENCODING)
        return self._read_var(debug)

    def _read_var(self, debug):
        """실제 read 프로토콜 송수신."""
        req = self._pack_read_req()
        self._send_req(req)
        _value = self._read_rsp(debug)
        if debug:
            print("[read] Value:", _value)
        return _value

    def _send_req(self, req):
        self.rsp = None
        try:
            self.sock.sendall(req)
            #print("sock.sendall(req)")
            self.rsp = self.sock.recv(1024)
            #self.rsp = self.sock.recv(256)
            #print("self.rsp: ", self.rsp)
        except socket.timeout:
            print("Socket timeout occurred. No response received.")
        except socket.error as e:
            print(f"Socket error: {e}")

    def _pack_read_req(self):
        """Read 요청 패킷 생성."""
        var_name_len = len(self.varname)
        flag = COMMAND_WRITE_VARIABLE # 0
        req_len = var_name_len + 3

        return struct.pack(
            '!HHBH' + str(var_name_len) + 's',
            self.msg_id,  # tag id
            req_len,  # message length
            flag,  # message type (0)
            var_name_len,  # length of var name
            self.varname
        )

    def _read_rsp(self, debug=False):
        """read 명령에 대한 응답 해석."""
        if self.rsp is None:
            return None
        header_format = '!HHBH'
        header_size = struct.calcsize(header_format)
        body = self.rsp[header_size:-3]  # Exclude the 3-byte end marker
        is_ok = self.rsp[-3:]
        if debug:
            print('[read_rsp] Raw Response:', self.rsp)
        # else:
        #     self.msg_id = (self.msg_id + 1) % 65536
        #     return self.rsp.decode('utf-8')
        if is_ok.endswith(b'\x01'):
            self.msg_id = (self.msg_id + 1) % 65536
            return body.decode('utf-8', errors='ignore')
        return None

    # def loop_read_var(self, var, interval=1):
    #     try:
    #         while True:
    #             response = self.read(var, debug=False)
    #             if response is not None:
    #                 print(f"Response: {response}")
    #             time.sleep(interval)  # 다음 요청 전에 지연
    #     except KeyboardInterrupt:
    #         print("Stop.")

#############################################################################
## Write System Variables ##
#############################################################################
    def write(self, var, value, debug=True):
        """KUKA 시스템 변수를 쓰기."""
        if not (isinstance(var, str) and isinstance(value, str)):
            raise TypeError('Var name and value should be string')
        self.varname = var.encode(ENCODING)
        self.value = value.encode(ENCODING)
        return self._write_var(debug)

    def _write_var(self, debug):
        req = self._pack_write_req()
        self._send_req(req)
        _value = self._read_rsp(debug)
        if debug:
            print("[write] Result:", _value)
        return _value

    def _pack_write_req(self):
        """Write 요청 패킷 생성."""
        var_name_len = len(self.varname)
        flag = COMMAND_WRITE_VARIABLE  # 1
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
## Execute Motion Commands ##
#############################################################################
    def move(self, motion_type, position, debug=True):
        """모션 명령 송신. position은 '{AXIS: ...}', '{POS: ...}' 등의 문자열."""
        if not isinstance(position, str):
            raise TypeError('Position values must be STRING.')
        if debug:
            print("[move] Position (original):", position)

        # wstring (UTF-16LE)
        self.position = position.encode('utf-16le')
        self.motion_type = motion_type

        if debug:
            print("[move] Position (encoded, hex):", self.position.hex())
        return self._move_robot(debug)

    def _move_robot(self, debug):
        req = self._pack_move_req()
        if debug:
            print("[move_robot] Packed request ready!")
        self._send_req(req)
        if debug:
            print("[move_robot] Send OK")
        _value = self._read_rsp_move(debug)
        if debug:
            print("[move_robot] Response decoded:", _value)
        return _value

    def _pack_move_req(self):
        """문서에 맞춰 Motion 패킷 생성."""
        position_len = len(self.position) // 2  # wstring이므로 문자 개수
        msg_type = COMMAND_MOTION  # 11
        # message length = 1(msg_type) + 1(motion_type) + 2(LP) + (LP*2)
        req_len = 4 + len(self.position)  # 4 + position bytes

        return struct.pack(
            '!HHBBH' + str(len(self.position)) + 's',
            self.msg_id,  # tag id
            req_len,  # message length
            msg_type,  # 11
            self.motion_type,  # motion type (ex. 1=PTP)
            position_len,  # LP (문자 길이)
            self.position  # UTF-16LE 바이트
        )

    def _read_rsp_move(self, debug=False):
        if self.rsp is None:
            return None
        header_format = '!HHBH'
        header_size = struct.calcsize(header_format)
        body = self.rsp[header_size:-3]
        is_ok = self.rsp[-3:]
        if debug:
            print('[read_rsp_move] Raw Response:', self.rsp)
        if is_ok.endswith(b'\x01'):
            self.msg_id = (self.msg_id + 1) % 65536
            return body.decode('utf-16le', errors='ignore')
        return None

    # def move(self, motion_type, position, debug=True):
    #     if not isinstance(position, str):
    #         raise TypeError('Position values must be STRING.')
    #     print("Position (original): ", position)
    #     self.position = position.encode('utf-16le')  # Wide format (UTF-16LE)
    #     self.motion_type = motion_type
    #     print("Position (encoded): ", self.position.hex())  # Print encoded position in hex for debugging
    #     return self._move_robot(debug)

    # def _move_robot(self, debug):
    #     req = self._pack_move_req()
    #     print("req ready!")
    #     self._send_req(req)
    #     print("send ok")
    #     _value = self._read_rsp_move(debug)
    #     if debug:
    #         print(_value)
    #     return _value



    # def _pack_move_req(self):
    #     # 2025.01.03
    #     # UTF-16LE 인코딩된 self.position의 길이(바이트)
    #     # => wstring이므로, 실제 문자열 길이는 (바이트 길이 / 2)
    #     position_len = len(self.position) // 2
    #
    #     msg_type = 11  # CommandMotion
    #     # 문서에 따르면: message length = 1(msg_type) + 1(motion_type) + 2(LP) + (LP*2)
    #     #              = 4 + len(self.position)
    #     req_len = 4 + len(self.position)
    #
    #     # struct.pack: "!HHBBH + (N)s"
    #     #  - H: uint16 (tag id)
    #     #  - H: uint16 (message length)
    #     #  - B: uint8  (message type)
    #     #  - B: uint8  (motion type)
    #     #  - H: uint16 (LP)
    #     #  - s: 바이트열 (position string)
    #     return struct.pack(
    #         '!HHBBH' + str(len(self.position)) + 's',
    #         self.msg_id,  # (H) tag id
    #         req_len,  # (H) message length
    #         msg_type,  # (B) 11
    #         self.motion_type,  # (B) motion type (문서상 1=PTP 등)
    #         position_len,  # (H) LP (문자 길이)
    #         self.position  # (s) position string (바이트)
    #     )
    #
    #     # 이전 코드 (임시 backup)
    #     # position_len = len(self.position) // 2
    #     # flag = 11  # Wide format version of CommandMotion
    #     # req_len = 3 + 2 + len(self.position)
    #     # return struct.pack(
    #     #     '!HHBBH' + str(len(self.position)) + 's',
    #     #     self.msg_id,  # Tag ID
    #     #     req_len,  # Message Length
    #     #     flag,  # Message Type
    #     #     self.motion_type,  # Motion Type
    #     #     position_len,  # Length of Position String (in characters)
    #     #     self.position  # Positon String
    #     # )

    # def _read_rsp_move(self, debug=False):
    #     if self.rsp is None:
    #         return None
    #     header_format = '!HHBH'
    #     header_size = struct.calcsize(header_format)
    #     body = self.rsp[header_size:-3]  # Exclude the 3-byte end marker
    #     is_ok = self.rsp[-3:]
    #     if debug:
    #         print('Response:', self.rsp)
    #     if is_ok.endswith(b'\x01'):
    #         self.msg_id = (self.msg_id + 1) % 65536
    #         return body.decode('utf-16le')

#############################################################################
## Program Control Commands ##
#############################################################################
    def start_program(self, interpreter=ROBOT_INTERPRETER, debug=True):
        """Program Control(Start) 명령 전송."""
        # command_code=2 (문서에 따라 Start가 2라고 가정)
        # → 만약 문서에서 '1=Reset, 2=Start, 3=Stop, 4=Cancel'이면 Start=2
        command_code = 2  # CMD_START

        req = self._pack_program_control_req(command_code, interpreter)
        self._send_req(req)
        return self._read_rsp(debug)

    def _pack_program_control_req(self, command_code, interpreter_type):
        """
        Program Control 패킷 (message type=10)
          - tag id (uint16)
          - message length (uint16) = 4 (msg_type + cmd_code + interpreter_type(2))
          - message type (uint8) = 10
          - command code (uint8) = ?
          - interpreter type (uint16)
        """
        msg_type = COMMAND_PROGRAM_CONTROL  # 10
        req_len = 1 + 1 + 2  # 4
        return struct.pack(
            '!HHBBH',
            self.msg_id,         # tag id
            req_len,             # message length
            msg_type,            # 10
            command_code,        # e.g., 2=Start
            interpreter_type     # 0=Submit, 1=Robot
        )

#########################################################################
# Move + 계속 Start (until Robot Stopped)
#########################################################################
    def move_with_start_loop(self, motion_type, position, interval=0.2, debug=True):
        """
        1) move(motion_type, position) 한 번 보냄
        2) 로봇 멈출 때까지:
           - read('$ROB_STOPPED') → 'TRUE'이면 종료, 'FALSE'면 start_program() 반복
        """
        if debug:
            print("[move_with_start_loop] >>> Sending Move command")
        self.move(motion_type, position, debug=debug)

        while True:
            time.sleep(interval)
            rob_stopped = self.read('$ROB_STOPPED', debug=False)
            if rob_stopped and 'TRUE' in rob_stopped.upper():
                if debug:
                    print("[move_with_start_loop] Robot has stopped!")
                break
            if debug:
                print("[move_with_start_loop] Robot is moving... Send Start command")
            self.start_program(ROBOT_INTERPRETER, debug=False)

        if debug:
            print("[move_with_start_loop] >>> Move sequence complete.")

if __name__ == "__main__":
    ip = 'xxx.xx.x.xxx'  # C3 Bridge Interface Server IP address
    port = 7000

    client = kukaClient(ip, port)
    if client.can_connect:
        print("Connecting Success.")

        stop_message = client.read('$STOPMESS')
        print(f"Can read $STOPMESS variabe in kuka system: {stop_message}")
        if stop_message:
            print(f"Robot is Stop Status: {stop_message}")
        else:
            print("Robot is Not Stop Status.")

        # read system variable test
        ov = client.read('$AXIST_ACT', debug=True)
        print(ov)

        # write sysytem variable test
        client.write('$OV_PRO', '30', debug=True)

        # 1) 단순 Move 예시 (한 번만 명령)
        print("\n--- Single Move test ---")
        position = '{AXIS: A1 0, A2 -90, A3 90, A4 0, A5 0, A6 0}'
        client.move(PTP, position, debug=True)

        # 2) Move + 지속적으로 Start 명령 주기 예시
        # 로봇이 멈출 때까지 $ROB_STOPPED를 모니터링하며 Start 전송
        print("\n--- Move with continuous Start ---")
        position2 = '{AXIS: A1 90, A2 -90, A3 0, A4 0, A5 0, A6 0}'
        client.move_with_start_loop(PTP, position2, interval=0.2, debug=True)

        # # motion command example (PTP)
        # position = '{AXIS: A1 0, A2 -90, A3 90, A4 0, A5 0, A6 0}'
        # #position = '{E6AXIS: A1 0.0, A2 0.0, A3 0.0, A4 0.0, A5 0.0, A6 0.0, E1 0.0, E2 0.0, E3 0.0, E40.0, E5 0.0, E6 0.0}'
        # # position = '{POS: X 0, Y 0, Z 0, A 0, B 0, C 0}'
        # client.move(1, position, debug=True)
    else:
        print("Server Connecting Fail.")

    client.close()