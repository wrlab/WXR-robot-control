from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from collections import defaultdict

import asyncio, ast
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import math
import json
import uvicorn

import logging


app = FastAPI()
connected = set()

# HOST = "169.254.16.195"
HOST = "192.168.1.51"
PORT = 30004
FREQ = 125

origins = ["http://localhost:8000", "https://localhost", "http://localhost"]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class ConnectionManager:
    def __init__(self):
        self.active_connections = []

    async def connect(self, websocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    async def broadcast(self, message):
        for connection in self.active_connections:
            await connection.send_text(message)

    async def message(self, websocket, message):
        await websocket.send_text(message)

    def disconnect(self, websocket):
        self.active_connections.remove(websocket)


manager = ConnectionManager()


@app.get("/{workspace_id}")
async def main(workspace_id: int):
    global conn, conf, setp
    print("initialize: ", workspace_id)

    conn = rtde.RTDE(HOST, PORT)
    conn.connect()
    conf = rtde_config.ConfigFile("record_configuration.xml")

    state_names, state_types = conf.get_recipe("state")
    setp_names, setp_types = conf.get_recipe("setp")

    setp = conn.send_input_setup(setp_names, setp_types)

    if not conn.send_output_setup(state_names, state_types, frequency=FREQ):
        print("Unable to configure output")
        exit()

    if not conn.send_start():
        print("Unable to start synchronization")
        exit()

    return {"message": "Hello World"}


@app.get("/{workspace_id}/move")
async def move(workspace_id: int, target_tcp: str, target_rot: str):
    global conn, conf, setp

    state = conn.receive()

    target_tcp = ast.literal_eval(target_tcp)
    target_rot = ast.literal_eval(target_rot)

    for idx, e in enumerate(target_tcp):
        setp.__dict__[f"input_double_register_{idx}"] = e

    for idx, e in enumerate(target_rot, 3):
        setp.__dict__[f"input_double_register_{idx}"] = e * math.pi / 180

    conn.send(setp)

    return {
        "joints": [q * 180 / math.pi for q in state.actual_q],
        "workspace_id": workspace_id,
        "target_tcp": target_tcp,
        "target_rot": target_rot,
    }


async def get_pos(websocket):
    global conn, conf, setp
    data = await websocket.receive_text()

    if "pos" in data:
        data = json.loads(data)

        x, y, z = data["pos"].values()
        u, v, w, order = data["rot"].values()
        u, v, w = [0, -180, 0]

        if x is None or y is None or z is None:
            return

        setp.input_double_register_0 = -x
        setp.input_double_register_1 = z
        setp.input_double_register_2 = y
        setp.input_double_register_3 = u * math.pi / 180
        setp.input_double_register_4 = v * math.pi / 180
        setp.input_double_register_5 = w * math.pi / 180

        conn.send(setp)


async def send_joints(websocket, conn):
    state = conn.receive()
    joints = state.actual_q

    await manager.message(
        websocket,
        json.dumps(
            {
                "len": len(joints),
                "timestamp": state.timestamp,
                "q1": joints[0] * 180 / math.pi,
                "q2": joints[1] * 180 / math.pi,
                "q3": joints[2] * 180 / math.pi,
                "q4": joints[3] * 180 / math.pi,
                "q5": joints[4] * 180 / math.pi,
                "q6": joints[5] * 180 / math.pi,
            }
        ),
    )


@app.websocket("/stream/{workspace_id}")
async def stream(websocket: WebSocket, workspace_id: int):
    global conn, conf
    await manager.connect(websocket)

    keep_running = True

    initial_state = await websocket.receive_text()

    print(initial_state)

    try:
        while keep_running:
            await get_pos(websocket)
            await send_joints(websocket, conn)

    except rtde.RTDEException:
        conn.send_pause()
        conn.disconnect()

    except WebSocketDisconnect:
        conn.send_pause()
        keep_running = False
        manager.disconnect(websocket)
        print("Connection closed from", workspace_id)


if __name__ == "__main__":
    uvicorn.run("server:app", host="0.0.0.0", port=8000, reload=True, workers=2)
