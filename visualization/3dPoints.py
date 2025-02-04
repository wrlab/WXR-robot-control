import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from itertools import product, combinations
import pandas as pd
import numpy as np

# 로그 파일에서 데이터 로드
#log_file = "C:/GitProjects/robodk-kuka/rdkapi-a/points/pose_log_67.txt"
log_file = "/Users/jaehongyoo/Documents/GitHub/WXR-robot-control/robodk-kuka/rdkapi-a-1/points/pose_log_53.txt"
data = pd.read_csv(log_file)

#total_log_file = "C:/GitProjects/robodk-kuka/rdkapi-a/points/total_pose_log_66.txt"
total_log_file = "/Users/jaehongyoo/Documents/GitHub/WXR-robot-control/robodk-kuka/rdkapi-a-1/points/total_pose_log_53.txt"
total_data = pd.read_csv(total_log_file)

#######################################################
# time 컬럼을 datetime으로 변환
data['time'] = pd.to_datetime(data['time'])

# 시간 차이 계산 (소수점 두 자리 이하 포함)
data['time_diff'] = data['time'].diff().dt.total_seconds()

# 소수점 두 자리 이하로 출력하기 위해 포맷 지정
data['time_diff'] = data['time_diff'].apply(lambda x: np.round(x, 3))

# 가장 작은 시간 차이 찾기
min_time_diff = data['time_diff'].min()

# 시간 차이의 평균 계산
mean_time_diff = data['time_diff'].mean()

print(f"가장 빠른 점 찍기 속도: {min_time_diff} 초에 1개의 점")
print(f"평균 점 찍기 속도: {mean_time_diff} 초에 1개의 점")

# 확인을 위해 가장 빠른 시간 간격과 해당 인덱스 출력
min_time_diff_index = data['time_diff'].idxmin()
print(f"가장 빠른 시간 간격의 인덱스: {min_time_diff_index}")
print(data.iloc[min_time_diff_index - 1:min_time_diff_index + 1])

# time 컬럼을 datetime으로 변환
total_data['time'] = pd.to_datetime(total_data['time'])

# 시간 차이 계산 (소수점 두 자리 이하 포함)
total_data['time_diff'] = total_data['time'].diff().dt.total_seconds()

# 소수점 두 자리 이하로 출력하기 위해 포맷 지정
total_data['time_diff'] = total_data['time_diff'].apply(lambda x: np.round(x, 3))

# 가장 작은 시간 차이 찾기
total_min_time_diff = total_data['time_diff'].min()

# 시간 차이의 평균 계산
total_mean_time_diff = total_data['time_diff'].mean()

print(f"가장 빠른 점 찍기 속도: {total_min_time_diff} 초에 1개의 점")
print(f"평균 점 찍기 속도: {total_mean_time_diff} 초에 1개의 점")

# 확인을 위해 가장 빠른 시간 간격과 해당 인덱스 출력
total_min_time_diff_index = total_data['time_diff'].idxmin()
print(f"가장 빠른 시간 간격의 인덱스: {total_min_time_diff_index}")
print(total_data.iloc[total_min_time_diff_index - 1:total_min_time_diff_index + 1])

##########################################################

# 3차원 그래프 생성
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

total_fig = plt.figure()
total_ax = total_fig.add_subplot(111, projection='3d')

# 점 그리기
ax.scatter(data['x'], data['y'], data['z'], c='r', marker='o')
total_ax.scatter(total_data['x'], total_data['y'], total_data['z'], c='r', marker='o')
# 축 레이블 설정
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

# 축 레이블 설정
total_ax.set_xlabel('X Label')
total_ax.set_ylabel('Y Label')
total_ax.set_zlabel('Z Label')

# 축 한계 설정
ax.set_xlim(922.5, 1922.5)
ax.set_ylim(-500, 500)
ax.set_zlim(125, 1125)

total_ax.set_xlim(922.5, 1922.5)
total_ax.set_ylim(-500, 500)
total_ax.set_zlim(125, 1125)

# 정육면체 생성 및 추가
center = np.array([1442.5, 0, 625])
side_length = 600
r = side_length / 2

# 각 축의 경계 설정
x = [center[0] - r, center[0] + r]
y = [center[1] - r, center[1] + r]
z = [center[2] - r, center[2] + r]

# 투명도 설정
alpha = 0.7

# 정육면체의 각 면 그리기
for s, e in combinations(np.array(list(product(x, y, z))), 2):
    if np.sum(np.abs(s-e)) == side_length:
        ax.plot3D(*zip(s, e), color="b", alpha=alpha)
        total_ax.plot3D(*zip(s, e), color="b", alpha=alpha)

# 그래프 표시
plt.show()
