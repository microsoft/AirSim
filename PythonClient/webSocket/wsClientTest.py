import asyncio
import websockets
import json


async def hello():
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
        name = input("What's your name? ")
        telemetry = {
            "battery_state": {
                "percentage": 70.04
            },
            "distance_from_home": 1561.4,
            "gimbal": {
                "roll": 2.65,
                "pitch": -4.33,
                "yaw": -4.33
            },
            "height_above_takeoff": 55.77,
            "gps_health": 5,
            "heading": 150.00,
            "velocity": {
                "x": 3.3859853744506836,
                "y": 4.3859853744506836,
                "z": -6.4859853744506836
            },
            "gps_position": {
                "latitude": 32.79549864125392,
                "altitude": 120.10076141357422,
                "longitude": 35.07356423292824
            },
            "last_change_time": 1577692382821,
            "lastHome": {
                "latitude": 32.79549864125392,
                "operationalAlt": 50.1007,
                "longitude": 35.07356423292824
            },
            "owner": "droneService",
            "state": {
                "armed": True
            },
            "wayPoints": {
                "status": 2
            },
            "keepAlive": 1577692382821
        }
        mesage = json.dumps(telemetry)
        await websocket.send(mesage)
        print(f"> {name}")

        greeting = await websocket.recv()
        print(f"< {greeting}")


asyncio.get_event_loop().run_until_complete(hello())


