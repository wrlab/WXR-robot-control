import socket
import struct
import logging
import argparse
from dataType import MessageType
from collections import defaultdict


def parse(buffer):
    fixed_header_format = "!iB"
    fixed_header_size = struct.calcsize(fixed_header_format)
    message_size, message_type = struct.unpack(
        fixed_header_format, buffer[:fixed_header_size]
    )
    buffer = buffer[fixed_header_size:message_size]

    parsed_data = defaultdict(lambda: {})

    if message_type == MessageType.ROBOT_STATE.value:
        while buffer:
            package_size, package_type = struct.unpack("!iB", buffer[:5])
            buffer = buffer[5:]

            if package_type == 0:
                fmt = "!Q???????BBdddB"
                fmt_size = struct.calcsize(fmt)
                parsed_data["robot_mode_data"] = struct.unpack(fmt, buffer[:fmt_size])

            if package_type == 1:
                fmt = "!dddffffB"
                fmt_size = struct.calcsize(fmt)
                parsed_data["joint_data"] = [
                    struct.unpack(fmt, buffer[i * fmt_size : (i + 1) * fmt_size])
                    for i in range(6)
                ]

            if package_type == 2:
                fmt = "!BBddfBffB"
                fmt_size = struct.calcsize(fmt)
                parsed_data["tool_data"] = struct.unpack(fmt, buffer[:fmt_size])

            if package_type == 3:
                fmt = "!iiBBddbbddffffBBbIBBBB"
                fmt_size = struct.calcsize(fmt)
                parsed_data["masterboard_data"] = struct.unpack(fmt, buffer[:fmt_size])

            if package_type == 4:
                fmt = "!ddd ddd ddd ddd"
                fmt_size = struct.calcsize(fmt)
                parsed_data["cartesian_info"] = struct.unpack(fmt, buffer[:fmt_size])

            if package_type == 5:
                fmt = "!6I6d6d6d6dI"
                fmt_size = struct.calcsize(fmt)
                parsed_data["kinematics_info"] = struct.unpack(fmt, buffer[:fmt_size])

            if package_type == 6:
                parsed_data["configuration_data"] = ()

            if package_type == 7:
                pass

            if package_type == 8:
                pass

            if package_type == 9:
                pass

            buffer = buffer[package_size - 5 :]

        if message_type == MessageType.ROBOT_MESSAGE.value:
            header_fmt = "!Qbbb"
            header_size = struct.calcsize(header_fmt)
            header = struct.unpack(header_fmt, buffer[:header_size])

            project_name_fmt = f"!{header[-1]}s"
            project_name_size = struct.calcsize(project_name_fmt)
            project_name_start = header_size
            project_name = struct.unpack(
                project_name_fmt,
                buffer[project_name_start : project_name_start + project_name_size],
            )[0].decode()

            version_info_fmt = "!BBii"
            version_info_size = struct.calcsize(version_info_fmt)
            version_info_start = project_name_start + project_name_size
            version_info = struct.unpack(
                version_info_fmt,
                buffer[version_info_start : version_info_start + version_info_size],
            )

            build_date_start = version_info_start + version_info_size
            build_date_size = message_size - build_date_start - 5
            build_date_fmt = f"!{build_date_size}s"
            build_date = struct.unpack(
                build_date_fmt,
                buffer[build_date_start : build_date_start + build_date_size],
            )[0].decode()

            parsed_data["version_message"] = (
                header + (project_name) + version_info + (build_date)
            )

    return parsed_data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("host", type=str, help="UR5 IP Address")
    parser.add_argument("port", type=int, help="UR5 Port")

    args = parser.parse_args()

    logging.basicConfig(
        format="%(asctime)s.%(msecs)06d %(levelname)s:%(message)s",
        level=logging.DEBUG,
        datefmt="%m/%d/%Y %I:%M:%S",
    )

    logging.info(f"Connecting to the primary client [{args.host}:{args.port}]")
    socket_obj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socket_obj.connect((args.host, args.port))

    with socket_obj as s:
        while True:
            buffer = s.recv(4096)
            ur_data = parse(buffer)

            logging.info(
                [
                    q_actual
                    for (
                        q_actual,
                        q_target,
                        qd_actual,
                        l_actual,
                        V_actual,
                        T_motor,
                        T_micro,
                        jointMode,
                    ) in ur_data["joint_data"]
                ]
            )

            if ur_data["cartesian_info"]:
                (
                    X,
                    Y,
                    Z,
                    Rx,
                    Ry,
                    Rz,
                    TCPOffsetX,
                    TCPOffsetY,
                    TCPOffsetZ,
                    TCPOffsetRx,
                    TCPOffsetRy,
                    TCPOffsetRz,
                ) = ur_data["cartesian_info"]

                logging.info(f"{[X, Y, Z]}")


if __name__ == "__main__":
    main()
