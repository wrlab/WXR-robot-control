# parse_krl.py
import re

def parse_krl(krl_code: str):
    """
    KRL 코드 문자열에서 LIN 명령어를 파싱하여 좌표값 리스트를 반환합니다.
    각 좌표값은 [X, Y, Z, A, B, C] 형식의 리스트입니다.
    """
    waypoints = []
    # LIN 명령어에서 X, Y, Z, A, B, C 값만 추출하는 정규표현식 (소수점과 음수 지원)
    pattern = r"LIN\s*\{X\s*([\d\-.]+),\s*Y\s*([\d\-.]+),\s*Z\s*([\d\-.]+),\s*A\s*([\d\-.]+),\s*B\s*([\d\-.]+),\s*C\s*([\d\-.]+)"
    for line in krl_code.splitlines():
        match = re.search(pattern, line)
        if match:
            x, y, z, a, b, c = map(float, match.groups())
            waypoints.append([x, y, z, a, b, c])
    return waypoints

def parse_krl_file(file_path: str):
    """
    지정된 파일에서 KRL 코드를 읽어 파싱한 후 좌표값 리스트를 반환합니다.
    """
    with open(file_path, 'r') as file:
        krl_code = file.read()
    return parse_krl(krl_code)