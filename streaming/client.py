from config import Config
import ast
import socket


def _get_bytes_stream(sock):
    length_data = sock.recv(5).decode()
    length = int(length_data)

    sock.recv(1)

    buf = sock.recv(length)
    return buf


if __name__ == "__main__":
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect((Config.HOST, Config.PORT))
        while True:
            data = _get_bytes_stream(client_socket)
            if data is None:
                print("Connection is closed")

            data = data.decode()
            print(*ast.literal_eval(data))
