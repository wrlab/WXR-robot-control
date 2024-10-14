from opcua import Server, ua
import time

# OPC UA 서버 생성 및 설정
server = Server()
url = "opc.tcp://192.168.1.171:4840" # 서버 URL 설정
server.set_endpoint(url)

# 네임스페이스 추가
name = "OPCUA_Test_SERVER"
idx = server.register_namespace(name)

# 객체 노드 추가
objects = server.get_objects_node()

# kuka_obj = objects.add_object(idx, "Frame")
# frame_data = "[1900.0, 866.0, 1212.0, 1.0]"
# frame_var = kuka_obj.add_variable(idx, "FRAME_DATA", frame_data, ua.VariantType.Float)
# frame_var.set_writable()

myobj = objects.add_object(idx, "X_coordinate")
myobj2 = objects.add_object(idx, "MyObject_boolean")
xVar1 = myobj.add_variable(idx, "xPlus", False, ua.VariantType.Boolean)
xVar2 = myobj.add_variable(idx, "xMinus", False, ua.VariantType.Boolean)
xVar1.set_writable() # 쓰기 가능하도록 설정
xVar2.set_writable() # 쓰기 가능하도록 설정

# 서버 시작
try:
    server.start()
    print("OPC UA 서버 시작")

    current_value = 10
    direction = 1

    while True:
        #myvar.set_value(current_value)
        print(f"현재 MyVariable 값: {current_value}")

        # 값 증가 또는 감소
        current_value += 5 * direction

        # 10에서 40 사이를 넘으면 방향 반대로
        if current_value > 40:
            current_value = 40
            direction = -1  # 감소로 변경
        elif current_value < 10:
            current_value = 10
            direction = 1  # 증가로 변경

        # 1초 대기
        time.sleep(1)

except Exception as e:
    print(f"OPC UA 서버 오류: {e}")

finally:
    server.stop()
    print("OPC UA 서버 종료")


