import socket
import struct
import threading

# 로컬 프록시 서버가 바인딩할 IP/포트
# RoboDK 드라이버가 원래 로봇에 접속하려는 IP, PORT와 동일하게 맞추거나,
# OS에서 포트포워딩(netsh, iptables)로 이 주소로 리다이렉션
LOCAL_HOST = '192.168.1.162'
LOCAL_PORT = 8001

# 일부 메시지 상수 (iiwa 드라이버 참고)
MSG_ACKNOWLEDGE = 128


def recv_until_null(sock):
    """로보DK 드라이버는 문자열을 '\0'로 종료하므로 '\0'까지 읽어들여 문자열 반환."""
    data = b''
    while True:
        ch = sock.recv(1)
        if ch == b'':
            # 연결 끊김
            return None
        if ch == b'\0':
            break
        data += ch
    return data.decode('utf-8', errors='replace')


def handle_client(client_socket, addr):
    print(f"[INFO] Connected from {addr}")

    # 1) 드라이버가 최초로 버전 정보를 보냄 (recv_line)
    driver_version = recv_until_null(client_socket)
    if driver_version is None:
        print("[ERROR] No data received for driver version. Connection closed.")
        client_socket.close()
        return

    print("[DRIVER->PROXY] Driver version line:", driver_version)

    # 2) 드라이버는 로봇 응답을 기대하므로 가짜 응답 전송 (Welcome)
    # '\0'로 끝나는 문자열
    welcome_msg = "Welcome"
    client_socket.sendall(welcome_msg.encode('utf-8') + b'\0')
    print("[PROXY->DRIVER] Sent welcome message")

    # 이후 드라이버는 명령을 MSG 형식으로 전송할 것임.
    # 각 명령: 정수 cmd 코드를 먼저 전송(4바이트 Big-endian), 필요하면 double 배열 전송.
    # 여기서는 단순히 수신한 raw 데이터를 확인하고, 이후 ACK만 보내주면 됨.

    # 무한 루프 돌며 명령 패킷 수신
    try:
        while True:
            # 명령코드(정수4바이트)를 먼저 수신
            cmd_bytes = client_socket.recv(4)
            if not cmd_bytes:
                break
            if len(cmd_bytes) < 4:
                print("[ERROR] Incomplete command code received")
                break
            cmd = struct.unpack('>i', cmd_bytes)[0]
            print(f"[DRIVER->PROXY] Received CMD: {cmd}")

            # 명령에 따라 추가 데이터(배열) 수신 필요할 수 있음.
            # iiwa 드라이버는 SendCmd 호출 시 다음 패턴:
            # 1) cmd (int)
            # 2) array length (int)
            # 3) if length>0, double 배열 (8*length bytes)
            # 여기서는 단순히 로깅 목적이므로 이 패턴을 따라 데이터 수신 후 출력
            length_bytes = client_socket.recv(4)
            if not length_bytes or len(length_bytes) < 4:
                print("[ERROR] Incomplete length for array")
                break
            arr_len = struct.unpack('>i', length_bytes)[0]
            print(f"[DRIVER->PROXY] Array length: {arr_len}")

            array_data = b''
            if arr_len > 0:
                expected_size = 8 * arr_len
                array_data = client_socket.recv(expected_size)
                if not array_data or len(array_data) < expected_size:
                    print("[ERROR] Incomplete array data")
                    break
                # double 배열일텐데 여기선 그냥 헥사/바이너리로만 출력
                print(f"[DRIVER->PROXY] Received array data ({arr_len} doubles): {array_data.hex()}")

            # 드라이버는 명령 전송 후 recv_acknowledge를 기다림.
            # recv_acknowledge는 int를 수신하며 MSG_ACKNOWLEDGE(=128)나 MSG_MONITOR(=127) 등을 기대.
            # 여기서는 항상 MSG_ACKNOWLEDGE만 보내서 드라이버를 만족시키자.
            ack_bytes = struct.pack('>i', MSG_ACKNOWLEDGE)
            client_socket.sendall(ack_bytes)
            print("[PROXY->DRIVER] Sent ACK")

    except ConnectionResetError:
        print("[INFO] Connection reset by driver")

    print("[INFO] Closing connection")
    client_socket.close()


def start_proxy():
    proxy_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    proxy_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    proxy_socket.bind((LOCAL_HOST, LOCAL_PORT))
    proxy_socket.listen(5)
    print(f"[INFO] Proxy listening on {LOCAL_HOST}:{LOCAL_PORT}")

    while True:
        client_socket, addr = proxy_socket.accept()
        client_thread = threading.Thread(target=handle_client, args=(client_socket, addr))
        client_thread.start()


if __name__ == "__main__":
    start_proxy()