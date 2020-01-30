import asyncio
import websockets
import json
import time
import threading
import airsim
import math


class WebSocketClient:
    isContinuousLoop = True
    p = None
    async def printit(self, websocket):

        while True:
            if WebSocketClient.isContinuousLoop:
                rpcinfo = self.client.getMultirotorState()
                gps_location = rpcinfo.gps_location
                kinematics_estimated = rpcinfo.kinematics_estimated
                pitch, roll, yaw  = airsim.to_eularian_angles(rpcinfo.kinematics_estimated.orientation)
                homepoint = self.client.getHomeGeoPoint()
                #print(math.degrees(yaw))
                #print(self.client.getGpsData())
          
                telemetry = {
                    "battery_state": {
                        "percentage": 70.04
                    },
                    "distance_from_home": 1561.4,
                    "gimbal": {
                        "roll": roll,
                        "pitch": pitch,
                        "yaw": yaw
                    },
                    "height_above_takeoff": 55.77,
                    "gps_health": 5,
                    "heading": math.degrees(yaw),
                    "velocity": {
                        "x": kinematics_estimated.linear_velocity.x_val,
                        "y": kinematics_estimated.linear_velocity.y_val,
                        "z": kinematics_estimated.linear_velocity.z_val
                    },
                    "gps_position": {
                        "latitude": gps_location.latitude,
                        "altitude": gps_location.altitude,
                        "longitude": gps_location.longitude
                    },
                    "last_change_time": rpcinfo.timestamp,
                    "lastHome": {
                        "latitude": homepoint.latitude,
                        "operationalAlt": homepoint.altitude,
                        "longitude": homepoint.longitude
                    },
                    "owner": "droneService",
                    "state": {
                        "armed": True
                    },
                    "wayPoints": {
                        "status": 2
                    },
                    "keepAlive": rpcinfo.timestamp
                }
                mesage = json.dumps(telemetry)
                await websocket.send(mesage)
                await asyncio.sleep(1)
                print(f"> {rpcinfo}")

    async def start(self):
        uri = "ws://localhost:8765"
        WebSocketClient.isContinuousLoop = True
        client = airsim.MultirotorClient()
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)

        self.client = client
        async with websockets.connect(uri) as websocket:
            await self.printit(websocket)

    def worker(self, loop):
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.start())
        # asyncio.get_event_loop().run_until_complete(self.start())

    def __init__(self):
        loop = asyncio.new_event_loop()
        p = threading.Thread(target=self.worker, args=(loop,))
        p.start()

    def end(self):
        WebSocketClient.isContinuousLoop = False
        print('thread Stop')


if __name__ == '__main__':
    d = WebSocketClient()
    d.start()
