import tkinter as tk
from opcua import Server, ua
import time
from threading import Thread

# 변수 값을 토글하고 일정 시간 후 False로 되돌리는 함수
# 변수 값을 토글하고 일정 시간 후 False로 되돌리는 함수

# def toggle_variable(variable):
#     variable.set_value(True)
#     print(f"{variable.get_browse_name()} True로 설정됨")
#     time.sleep(0.1)  # 1초 후 False로 되돌림
#     variable.set_value(False)
#     print(f"{variable.get_browse_name()} False로 설정됨")

# xPlus와 xMinus 변수를 주기적으로 전환하는 함수
def toggle_variables(xVar1, xVar2):
    while True:
        # xPlus를 True로 설정
        xVar1.set_value(True)
        print(f"{xVar1.get_browse_name()} True로 설정됨")
        time.sleep(5)  # 5초 대기
        xVar1.set_value(False)
        print(f"{xVar1.get_browse_name()} False로 설정됨")

        # xMinus를 True로 설정
        xVar2.set_value(True)
        print(f"{xVar2.get_browse_name()} True로 설정됨")
        time.sleep(5)  # 5초 대기
        xVar2.set_value(False)
        print(f"{xVar2.get_browse_name()} False로 설정됨")

# OPC UA 서버 시작 함수
def start_opcua_server():
    server = Server()
    url = "opc.tcp://192.168.1.171:4840"  # 로컬에서 테스트하기 위해 IP를 로컬 호스트로 변경
    server.set_endpoint(url)

    name = "OPCUA_Test_SERVER"
    idx = server.register_namespace(name)

    objects = server.get_objects_node()

    myobj = objects.add_object(idx, "X_coordinate")
    xVar1 = myobj.add_variable(idx, "xPlus", False, ua.VariantType.Boolean)
    xVar2 = myobj.add_variable(idx, "xMinus", False, ua.VariantType.Boolean)
    xVar1.set_writable()  # 쓰기 가능하도록 설정
    xVar2.set_writable()  # 쓰기 가능하도록 설정

    try:
        server.start()
        print("OPC UA 서버 시작됨")

        # GUI에서 OPC UA 변수를 조작하는 함수
        # def on_x_plus():
        #     Thread(target=toggle_variable, args=(xVar1,)).start()
        #
        # def on_x_minus():
        #     Thread(target=toggle_variable, args=(xVar2,)).start()

        # 스레드에서 xPlus와 xMinus 변수를 주기적으로 토글하는 함수 실행
        toggle_thread = Thread(target=toggle_variables, args=(xVar1, xVar2))
        toggle_thread.daemon = True
        toggle_thread.start()

        # 간단한 Tkinter GUI 생성
        root = tk.Tk()
        root.title("OPC UA 토글 테스트")

        # btn_x_plus = tk.Button(root, text="X Plus", command=on_x_plus)
        # btn_x_plus.pack(pady=10)
        #
        # btn_x_minus = tk.Button(root, text="X Minus", command=on_x_minus)
        # btn_x_minus.pack(pady=10)

        # 서버 종료는 GUI 이벤트 루프가 종료된 후에만 이루어지도록 함
        root.mainloop()

    except Exception as e:
        print(f"OPC UA 서버 오류: {e}")

    finally:
        print("OPC UA 서버 종료 중...")
        server.stop()
        print("OPC UA 서버 종료됨")

# 서버를 별도의 스레드에서 실행
start_opcua_server()
#opcua_thread = Thread(target=start_opcua_server)
#opcua_thread.start()