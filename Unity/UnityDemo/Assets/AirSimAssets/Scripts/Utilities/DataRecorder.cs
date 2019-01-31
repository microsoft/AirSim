using System.IO;
using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

namespace AirSimUnity {
    /*
     * Utility class to save images to Documents\AirSim folder.
     * The Images are queued in the main thread while the file saving is done in background thread.
     * Creates the folder to hold images and it's corresponding data in a text file.
     * The folder creation and data capturing is similar to Unreal AirSim.
     */

    public class DataRecorder {
        private const string FLOAT_FORMAT = "0.0000";

        public struct ImageData {
            public CarStructs.CarData carData;
            public AirSimPose pose;
            public byte[] image;
        }

        private Thread encoderThread;
        private Queue<ImageData> imagesQueue;
        private StreamWriter dataWriter;

        private bool isDrone;
        private readonly string docsLocation;
        private string folderLocation;
        private string fileName;
        private string imagesLocation;

        private bool isCapturingImagesRunning;
        private int count = 0;

        public DataRecorder() {
            isDrone = true;
            count = 0;
            docsLocation = IsLinux ? "/Documents/AirSim/" : "/AirSim/";
            docsLocation = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments) + docsLocation;
        }

        public void IsDrone(bool isDrone) {
            this.isDrone = isDrone;
        }

        public static bool IsLinux {
            get {
                int p = (int)Environment.OSVersion.Platform;
                return (p == 4) || (p == 6) || (p == 128);
            }
        }

        //Start the thread to dequeue the capture data and save them in a folder.
        public void StartRecording() {
            imagesQueue = new Queue<ImageData>();
            isCapturingImagesRunning = true;
            encoderThread = new Thread(SaveImagesThread);
            encoderThread.Priority = System.Threading.ThreadPriority.BelowNormal;
            encoderThread.Start();

            folderLocation = docsLocation + DateTime.Now.ToString("yyyy-MM-dd-HH-mm-ss");
            if (!Directory.Exists(folderLocation)) {
                Directory.CreateDirectory(folderLocation);
            }

            imagesLocation = IsLinux ? "/images" : "\\images";
            imagesLocation = folderLocation + imagesLocation;
            if (!Directory.Exists(imagesLocation)) {
                Directory.CreateDirectory(imagesLocation);
            }

            fileName = IsLinux ? "/airsim_rec.txt" : "\\airsim_rec.txt";
            fileName = folderLocation + fileName;
            dataWriter = new StreamWriter(File.Open(fileName, FileMode.OpenOrCreate, FileAccess.Write));
            string heading;
            if (isDrone) {
                heading = "Timestamp\tPosition(x)\tPosition(y)\tPosition(z)\tOrientation(w)\tOrientation(x)\tOrientation(y)\tOrientation(z)\tImageName";
            } else {
                heading = "Timestamp\tSpeed (kmph)\tThrottle\tSteering\tBrake\tGear\tImageName";
            }
            dataWriter.WriteLine(heading);
        }

        public void StopRecording() {
            isCapturingImagesRunning = false;
            if (dataWriter != null) {
                dataWriter.Close();
                dataWriter = null;
            }
        }

        public void AddImageDataToQueue(ImageData data) {
            if (imagesQueue == null) {
                return;
            }
            imagesQueue.Enqueue(data);
        }

        private void SaveImagesThread() {
            string imageName;
            while (true) {
                if (!isCapturingImagesRunning && imagesQueue.Count <= 0) {
                    break;
                } else if (imagesQueue.Count <= 0) {
                    Thread.Sleep(500);
                    continue;
                }

                long timeStamp = DataManager.GetCurrentTimeInMilli();
                ImageData data = imagesQueue.Dequeue();

                imageName = string.Format("img_{0}_{1}.png", count++, timeStamp);
                if (isDrone) {
                    dataWriter.WriteLine(string.Format("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}",
                        timeStamp, data.pose.position.x.ToString(FLOAT_FORMAT),
                        data.pose.position.y.ToString(FLOAT_FORMAT), data.pose.position.z.ToString(FLOAT_FORMAT),
                        data.pose.orientation.w.ToString(FLOAT_FORMAT), data.pose.orientation.x.ToString(FLOAT_FORMAT),
                        data.pose.orientation.y.ToString(FLOAT_FORMAT), data.pose.orientation.z.ToString(FLOAT_FORMAT), imageName));
                } else {
                    dataWriter.WriteLine(string.Format("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}",
                        timeStamp, data.carData.speed, data.carData.throttle.ToString(FLOAT_FORMAT),
                        data.carData.steering.ToString(FLOAT_FORMAT), data.carData.brake.ToString(FLOAT_FORMAT),
                        data.carData.gear, imageName));
                }

                if (IsLinux) {
                    imageName = string.Format("{0}/{1}", imagesLocation, imageName);
                } else {
                    imageName = string.Format("{0}\\{1}", imagesLocation, imageName);
                }

                File.WriteAllBytes(imageName, data.image);
            }
            imagesQueue = null;
        }
    }
}