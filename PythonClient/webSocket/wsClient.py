
import airsim
import threading


class WebSocketClient:

    timer = threading.Timer(5.0, None)

    def start(self):
        print("start web sockets")
        client = airsim.MultirotorClient()
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)
        self.client = client
        self.printit()
        return not client.getMultirotorState().collision.has_collided

    def printit(self):
        WebSocketClient.timer = threading.Timer(1.0, self.printit)
        WebSocketClient.timer.start()
        print(self.client.getMultirotorState())

    def end(self):
        if WebSocketClient.timer:
            WebSocketClient.timer.cancel()
         