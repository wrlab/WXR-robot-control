import pandas as pd
import numpy as np

# 로그 파일에서 데이터 로드
log_file = "C:/GitProjects/robodk-kuka/rdkapi-a/points/pose_log_44.txt"
#log_file = "C:/GitProjects/robodk-kuka/rdkapi-a/joints/joints_log_4.txt"
#export = "C:/GitProjects/robodk-kuka/log2krl/krl/krl1.txt"
#export = "C:/GitProjects/robodk-kuka/log2krl/krl/krl2.txt"
export = "C:/GitProjects/robodk-kuka/log2krl/krl/krl7.txt"

data = pd.read_csv(log_file)

def generate_ptp(data):
    commands = []
    for index, row in data.iterrows():
        x, y, z = row['x'], row['y'], row['z']
        command = f"PTP {{X {x}, Y {y}, Z {z}}} C_PTP"
        commands.append(command)
    return commands

def generate_lin(data):
    commands = []
    for index, row in data.iterrows():
        x, y, z = row['x'], row['y'], row['z']
        command = f"LIN {{X {x}, Y {y}, Z {z}}} C_DIS"
        commands.append(command)
    return commands

def generate_ptp_joints(data):
    commands = []
    for index, row in data.iterrows():
        a1, a2, a3, a4, a5, a6 = round(row['a1'], 4), round(row['a2'], 4), round(row['a3'], 4), round(row['a4'], 4), round(row['a5'], 4), round(row['a6'], 4)
        command = f"PTP {{A1 {a1}, A2 {a2}, A3 {a3}, A4 {a4}, A5 {a5}, A6 {a6}}} C_PTP"
        commands.append(command)
    return commands

def generate_sptp_joints(data):
    commands = []
    for index, row in data.iterrows():
        a1, a2, a3, a4, a5, a6 = round(row['a1'], 4), round(row['a2'], 4), round(row['a3'], 4), round(row['a4'], 4), round(row['a5'], 4), round(row['a6'], 4)
        command = f"SPTP {{A1 {a1}, A2 {a2}, A3 {a3}, A4 {a4}, A5 {a5}, A6 {a6}}} C_PTP"
        commands.append(command)
    return commands

def generate_slin_joints(data):
    commands = []
    for index, row in data.iterrows():
        x, y, z = row['x'], row['y'], row['z']
        command = f"SLIN {{X {x}, Y {y}, Z {z}}} C_SPL"
        commands.append(command)
    return commands

ptp_commands = generate_lin(data)

# 생성된 명령어 출력
# 생성된 명령어 출력 및 파일로 저장
with open(export, 'w') as file:
    for command in ptp_commands:
        file.write(command + '\n')
        print(command)


