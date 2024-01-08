import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import argparse

parser = argparse.ArgumentParser("UR5 Streaming Server")
parser.add_argument("--ur_host", type=str, required=True, help="UR5 IP Address")
args = parser.parse_args()

conn = rtde.RTDE(args.ur_host, 30004)

conn.connect()
conf = rtde_config.ConfigFile("record_configuration.xml")

state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")

setp = conn.send_input_setup(setp_names, setp_types)

if not conn.send_output_setup(state_names, state_types, frequency=125):
    print("Unable to configure output")
    exit()

if not conn.send_start():
    print("Unable to start synchronization")
    exit()

keep_running = True

try:
    while keep_running:
        state = conn.receive()
        print(state.timestamp)
except KeyboardInterrupt:
    keep_running = False

conn.send_pause()
conn.disconnect()
