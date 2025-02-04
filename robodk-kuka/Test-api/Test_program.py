# create_program.py
from robodk.robolink import Robolink
from robodk.robomath import transl, rotx, roty, rotz
import os
import re, math
from parse_krl2robodk import parse_krl_file  # 별도 모듈에서 파싱 함수 임포트

# RoboDK에 연결
RDK = Robolink()

robot = RDK.Item('KUKA KR 70 R2100-Meltio') # Get robot item.
positioner = RDK.Item('KUKA KP2-HV500')
reference = RDK.Item('base-frame')
tool = robot.Tool()

# 🔹 3. 그룹 (폴더 역할을 하는 프레임) 생성
# hidden_targets_group = RDK.AddFrame("HiddenTargets", reference)
# hidden_targets_group.setVisible(False)  # 전체 폴더 숨기기
hidden_targets_group = reference

# 🔹 4. 새로운 프로그램 생성
program = RDK.AddProgram("GeneratedToolpath", robot)
program.setVisible(False)
program.ShowInstructions(False)

# 파일 읽기 – 여기서는 toolpath.krl 파일에 전체 KRL 코드가 저장되어 있다고 가정합니다.
krl_file = "toolpath/ssample_1-prototype.src"
with open(krl_file, 'r') as f:
    lines = f.readlines()

# 초기값 설정
current_mode = None       # 현재 모드: "manipulator" 혹은 "turntable"
current_TCP = None        # 최근에 설정된 TCP 좌표 (X, Y, Z, A, B, C)
# targets 리스트에 (타입, pose, (E1, E2)) 튜플을 저장합니다.
targets = []

# 정규표현식: LIN 명령어에서 X, Y, Z, A, B, C, E1, E2 값을 추출
# (나머지 E3~E6는 모두 0인 것으로 가정)
lin_pattern = re.compile(
    r"LIN\s*\{X\s*([\d\-.]+),\s*Y\s*([\d\-.]+),\s*Z\s*([\d\-.]+),\s*A\s*([\d\-.]+),\s*B\s*([\d\-.]+),\s*C\s*([\d\-.]+),\s*E1\s*([\d\-.]+),\s*E2\s*([\d\-.]+)"
)

# KRL 파일의 각 줄을 순차적으로 처리
for line in lines:
    line = line.strip()
    if not line:
        continue
    # 모드 변경: $VEL.CP = value
    if line.startswith("$VEL.CP"):
        try:
            val = float(line.split('=')[1].strip())
        except:
            continue
        if abs(val - 0.060) < 1e-5:
            current_mode = "manipulator"
        elif abs(val - 0.0085) < 1e-5:
            current_mode = "turntable"
        else:
            # 다른 속도값은 무시
            current_mode = None
        continue

    # 동기화 신호 등은 무시
    if line.startswith("$OUT") or line.startswith("WAIT"):
        continue

    # LIN 명령어 처리
    if line.startswith("LIN"):
        m = lin_pattern.search(line)
        if not m:
            continue
        # 추출한 값: X, Y, Z, A, B, C, E1, E2
        x, y, z, a, b, c, e1, e2 = list(map(float, m.groups()))
        # 여기서 TCP의 b값은 무조건 0으로 설정
        b = 0.0
        # 모드에 따라 처리
        if current_mode == "manipulator":
            # 매니퓰레이터 이동 모드: TCP도 갱신 (b는 0)
            current_TCP = (x, y, z, a, b, c)
            # 변환: 각도는 radian으로 변환
            a_rad = math.radians(a)
            b_rad = math.radians(b)  # b_rad는 항상 0
            c_rad = math.radians(c)
            pose = transl(x, y, z) * rotx(a_rad) * roty(b_rad) * rotz(c_rad)
            targets.append(("manipulator", pose, (e1, e2)))
        elif current_mode == "turntable":
            # turntable 모드: 이전 TCP(current_TCP) 사용 (b가 이미 0)
            if current_TCP is None:
                continue  # TCP가 지정되지 않은 상태면 건너뜁니다.
            x0, y0, z0, a0, b0, c0 = current_TCP
            a0_rad = math.radians(a0)
            b0_rad = math.radians(b0)  # b0는 0
            c0_rad = math.radians(c0)
            pose = transl(x0, y0, z0) * rotx(a0_rad) * roty(b0_rad) * rotz(c0_rad)
            targets.append(("turntable", pose, (e1, e2)))
        continue

# ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
# RoboDK 프로그램 구성 – targets 리스트 순서대로 타깃 생성 및 이동 명령 추가
if not targets:
    raise Exception("파싱된 타깃이 없습니다.")

for i, (t_type, pose, ext_axes) in enumerate(targets):
    if t_type == "manipulator":
        # 새 타깃 생성 (매니퓰레이터의 TCP 목표)
        target = RDK.AddTarget(f"Target_{i}", hidden_targets_group)
        target.setPose(pose)
        target.setVisible(False)
        print(f"Target_{i}가 생성되었습니다.")
        program.MoveL(target)
        # 동시에 포지셔너(턴테이블)도 외부 축(E1,E2) 값으로 이동시키기
        positioner.MoveJ(list(ext_axes))
    elif t_type == "turntable":
        # turntable 명령: TCP는 그대로 두고, 포지셔너만 외부 축 업데이트
        print(f"Turntable 명령에서 Target_{i} (외부 축 업데이트)가 적용됩니다.")
        positioner.MoveJ(list(ext_axes))

RDK.Render(True)
RDK.Update()

#program.RunProgram()