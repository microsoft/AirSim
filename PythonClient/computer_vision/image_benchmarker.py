from argparse import ArgumentParser
import airsim
import time
import threading
import numpy as np
import cv2
import tempfile
import os

class ImageBenchmarker():
    def __init__(self, 
            img_benchmark_type = 'simGetImages', 
            viz_image_cv2 = False,
            save_images = False):
        self.airsim_client = airsim.VehicleClient()
        self.airsim_client.confirmConnection()
        self.image_benchmark_num_images = 0
        self.image_benchmark_total_time = 0.0
        self.image_callback_thread = None
        self.viz_image_cv2 = viz_image_cv2
        self.save_images = save_images

        if img_benchmark_type == "simGetImage":
            self.image_callback_thread = threading.Thread(target=self.repeat_timer_img, args=(self.image_callback_benchmark_simGetImage, 0.05))
        if img_benchmark_type == "simGetImages":
            self.image_callback_thread = threading.Thread(target=self.repeat_timer_img, args=(self.image_callback_benchmark_simGetImages, 0.05))
        self.is_image_thread_active = False
        self.image_callback_thread.daemon = True

        if self.save_images:
            self.tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_img_bm")
            print ("Saving images to %s" % self.tmp_dir)
            try:
                os.makedirs(self.tmp_dir)
            except OSError:
                if not os.path.isdir(self.tmp_dir):
                    raise

    def start_img_benchmark_thread(self):
        if not self.is_image_thread_active:
            self.is_image_thread_active = True
            self.image_callback_thread.start()
            print("Started img image_callback thread")

    def stop_img_benchmark_thread(self):
        if self.is_image_thread_active:
            self.is_image_thread_active = False
            self.image_callback_thread.join()
            print("Stopped image callback thread.")

    def repeat_timer_img(self, task, period):
        while self.is_image_thread_active:
            task()
            time.sleep(period)

    def print_benchmark_results(self):
        avg_fps = 1.0 / ((self.image_benchmark_total_time) / float(self.image_benchmark_num_images))
        print("result: {} avg_fps for {} num of images".format(avg_fps, self.image_benchmark_num_images))

    def image_callback_benchmark_simGetImage(self):
        self.image_benchmark_num_images += 1
        iter_start_time = time.time()
        image = self.airsim_client.simGetImage("front_center", airsim.ImageType.Scene)
        np_arr = np.frombuffer(image, dtype=np.uint8)
        img_rgb = np_arr.reshape(240, 512, 4)
        self.image_benchmark_total_time += time.time() - iter_start_time
        avg_fps = 1.0 / ((self.image_benchmark_total_time) / float(self.image_benchmark_num_images))
        print("result: {} avg_fps for {} num of images".format(avg_fps, self.image_benchmark_num_images))
        # uncomment following lines to viz image
        if self.viz_image_cv2:
            cv2.imshow("img_rgb", img_rgb)
            cv2.waitKey(1)

    def image_callback_benchmark_simGetImages(self):
        self.image_benchmark_num_images += 1
        iter_start_time = time.time()
        request = [airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)]
        response = self.airsim_client.simGetImages(request)
        np_arr = np.frombuffer(response[0].image_data_uint8, dtype=np.uint8)
        img_rgb = np_arr.reshape(response[0].height, response[0].width, -1)
        self.image_benchmark_total_time += time.time() - iter_start_time
        avg_fps = 1.0 / ((self.image_benchmark_total_time) / float(self.image_benchmark_num_images))
        print("result + {} avg_fps for {} num of images".format(avg_fps, self.image_benchmark_num_images))
        # uncomment following lines to viz image
        if self.viz_image_cv2:
            cv2.imshow("img_rgb", img_rgb)
            cv2.waitKey(1)
        if self.save_images:
            filename = os.path.join(self.tmp_dir, str(self.image_benchmark_num_images))
            cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png

def main(args):
    baseline_racer = ImageBenchmarker(img_benchmark_type=args.img_benchmark_type, viz_image_cv2=args.viz_image_cv2,
                                      save_images=args.save_images)
 
    baseline_racer.start_img_benchmark_thread()
    time.sleep(30)
    baseline_racer.stop_img_benchmark_thread()
    baseline_racer.print_benchmark_results()

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--img_benchmark_type', type=str, choices=["simGetImage", "simGetImages"], default="simGetImages")
    parser.add_argument('--enable_viz_image_cv2', dest='viz_image_cv2', action='store_true', default=False)
    parser.add_argument('--save_images', dest='save_images', action='store_true', default=False)

    args = parser.parse_args()
    main(args)