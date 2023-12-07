from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from starlette.websockets import WebSocketDisconnect

import asyncio
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import math
import json


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


@app.get("/{workspace_id}")
async def main(workspace_id: int):
    global conn
    print("initialize: ", workspace_id)

    conn = rtde.RTDE(HOST, PORT)
    conn.connect()

    return {"message": "Hello World"}


@app.websocket("/stream/{workspace_id}")
async def stream(websocket: WebSocket, workspace_id: int):
    global conn
    await websocket.accept()

    conf = rtde_config.ConfigFile("record_configuration.xml")
    output_names, output_types = conf.get_recipe("out")

    try:
        while True:
            if not conn.send_output_setup(output_names, output_types, frequency=FREQ):
                print("Unable to configure output")
                exit()

            if not conn.send_start():
                print("Unable to start synchronization")
                exit()

            state = conn.receive()
            joints = state.actual_q

            await websocket.send_text(
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
                )
            )

    except rtde.RTDEException:
        conn.send_pause()
        conn.disconnect()

    except WebSocketDisconnect:
        conn.send_pause()
        print("Connection closed from", workspace_id)
        await websocket.close()
