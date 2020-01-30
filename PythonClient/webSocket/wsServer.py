import asyncio
import websockets


async def hello(websocket, path):
    while True:
        try:
            text = await websocket.recv()
            print(text)
        except websockets.ConnectionClosed:
            break

start_server = websockets.serve(hello, 'localhost', 8765)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
