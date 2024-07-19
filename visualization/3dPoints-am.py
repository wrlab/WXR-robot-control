import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re

# 로그 파일 경로
log_file_path = "C:/GitProjects/visualization/samplecode/MyProgram_001().src"

# X, Y, Z 좌표를 저장할 리스트
x_coords = []
y_coords = []
z_coords = []

# 로그 파일에서 데이터를 읽어오기
with open(log_file_path, 'r') as file:
    lines = file.readlines()
    for line in lines:
        if line.startswith("LIN"):
            # 정규식을 이용해 X, Y, Z 좌표 추출
            match = re.search(r"X (-?\d+\.\d+),Y (-?\d+\.\d+),Z (-?\d+\.?\d*)", line)
            if match:
                x_coords.append(float(match.group(1)))
                y_coords.append(float(match.group(2)))
                z_coords.append(float(match.group(3)))

# 3차원 그래프 생성
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 점 그리기
ax.scatter(x_coords, y_coords, z_coords, c='r', marker='o')

# 축 레이블 설정
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

# 그래프 표시
plt.show()
