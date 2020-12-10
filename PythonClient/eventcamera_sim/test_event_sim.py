import numpy as np
import airsim
import time
import cv2
import matplotlib.pyplot as plt

from event_simulator import *


class AirSimEventGen:
    def __init__(self, W, H, debug=False):
        self.ev_sim = EventSimulator(W, H)
        self.W = W
        self.H = H

        self.image_request = airsim.ImageRequest(
            "0", airsim.ImageType.Scene, False, False
        )

        self.client = airsim.VehicleClient()
        self.client.confirmConnection()
        self.init = True
        self.start_ts = None

        self.rgb_image_shape = [H, W, 3]
        self.debug = debug

        if debug:
            self.fig, self.ax = plt.subplots(1, 1)

    def visualize_events(self, event_img):
        event_img = self.convert_event_img_rgb(event_img)
        self.ax.cla()
        self.ax.imshow(event_img, cmap="viridis")
        plt.draw()
        plt.pause(0.01)

    def convert_event_img_rgb(self, image):
        image = image.reshape(self.H, self.W)
        out = np.zeros((self.H, self.W, 3), dtype=np.uint8)
        out[:, :, 0] = np.clip(image, 0, 1) * 255
        out[:, :, 2] = np.clip(image, -1, 0) * -255

        return out


if __name__ == "__main__":
    event_generator = AirSimEventGen(256, 144, debug=True)
    i = 0
    start_time = 0
    t_start = time.time()

    while True:
        image_request = airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)

        response = event_generator.client.simGetImages([event_generator.image_request])
        while response[0].height == 0 or response[0].width == 0:
            response = event_generator.client.simGetImages(
                [event_generator.image_request]
            )

        ts = time.time_ns()

        if event_generator.init:
            event_generator.start_ts = ts
            event_generator.init = False

        img = np.reshape(
            np.fromstring(response[0].image_data_uint8, dtype=np.uint8),
            event_generator.rgb_image_shape,
        )

        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
        img = cv2.add(img, 0.001)

        ts = time.time_ns()
        ts_delta = (ts - event_generator.start_ts) * 1e-3
        start = time.time()
        event_img, events = event_generator.ev_sim.image_callback(img, ts_delta)
        print(f"Time: {time.time() - start}")
        bytestream = []

        if events is not None and events.shape[0] > 0:
            bytestream = events.tolist()
            print(len(bytestream))

            event_generator.visualize_events(event_img)

