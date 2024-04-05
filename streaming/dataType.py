from enum import Enum


class MessageType(Enum):
    DISCONNECT = -1
    ROBOT_STATE = 16
    ROBOT_MESSAGE = 20
    HMC_MESSAGE = 22
    MODBUS_INFO_MESSAGE = 5
    SAFETY_SETUP_BROADCAST_MESSAGE = 23
    SAFETY_COMPLIANCE_TOLERANCES_MESSAGE = 24
    PROGRAM_STATE_MESSAGE = 25


class RobotModes(Enum):
    NO_CONTROLLER = -1
    DISCONNECTED = 0
    CONFIRM_SAFETY = 1
    BOOTING = 2
    POWER_OFF = 3
    POWER_ON = 4
    IDLE = 5
    BACKDRIVE = 6
    RUNNING = 7
    UPDATING_FIRMWARE = 8


class JointModes(Enum):
    RESET = 235
    SHUTTING_DOWN = 236
    BACKDRIVE = 238
    POWER_OFF = 239
    READY_FOR_POWER_OFF = 240
    NOT_RESPONDING = 245
    MOTOR_INITIALISATION = 246
    BOOTING = 247
    VIOLATION = 251
    FAULT = 252
    RUNNING = 253
    IDLE = 255


class MessageSource(Enum):
    MESSAGE_SOURCE_JOINT_0_FPGA = 100
    MESSAGE_SOURCE_JOINT_0_A = 110
    MESSAGE_SOURCE_JOINT_0_B = 120
    MESSAGE_SOURCE_JOINT_1_FPGA = 101
    MESSAGE_SOURCE_JOINT_1_A = 111
    MESSAGE_SOURCE_JOINT_1_B = 121
    MESSAGE_SOURCE_JOINT_2_FPGA = 102
    MESSAGE_SOURCE_JOINT_2_A = 112
    MESSAGE_SOURCE_JOINT_2_B = 122
    MESSAGE_SOURCE_JOINT_3_FPGA = 103
    MESSAGE_SOURCE_JOINT_3_A = 113
    MESSAGE_SOURCE_JOINT_3_B = 123
    MESSAGE_SOURCE_JOINT_4_FPGA = 104
    MESSAGE_SOURCE_JOINT_4_A = 114
    MESSAGE_SOURCE_JOINT_4_B = 124
    MESSAGE_SOURCE_JOINT_5_FPGA = 105
    MESSAGE_SOURCE_JOINT_5_A = 115
    MESSAGE_SOURCE_JOINT_5_B = 125
    MESSAGE_SOURCE_TOOL_FPGA = 106
    MESSAGE_SOURCE_TOOL_A = 116
    MESSAGE_SOURCE_TOOL_B = 126
    MESSAGE_SOURCE_EUROMAP_FPGA = 107
    MESSAGE_SOURCE_EUROMAP_A = 117
    MESSAGE_SOURCE_EUROMAP_B = 127
    MESSAGE_SOURCE_TEACH_PENDANT_A = 108
    MESSAGE_SOURCE_TEACH_PENDANT_B = 118
    MESSAGE_SOURCE_SCB_FPGA = 40
    MESSAGE_SAFETY_PROCESSOR_UA = 20
    MESSAGE_SAFETY_PROCESSOR_UB = 30
    MESSAGE_SOURCE_ROBOTINTERFACE = -2
    MESSAGE_SOURCE_RTMACHINE = -3
    MESSAGE_SOURCE_SIMULATED_ROBOT = -4
    MESSAGE_SOURCE_GUI = -5
    MESSAGE_SOURCE_CONTROLLER = 7
    MESSAGE_SOURCE_RTDE = 8
