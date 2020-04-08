# Imitation Learning  

This section is about training a model to steer our Formula car using imitation learning.  
The code in this section is based on the [Autonomous Driving Cookbook](https://github.com/Microsoft/AutonomousDrivingCookbook/tree/master/AirSimE2EDeepLearning) from Airsim and it's highly recommended to read the tutorial first.  

## Prerequisites  
* Operating system: Windows 10  
* GPU: Nvidia GTX 1080 or higher (recommended)  
* Software: Unreal Engine 4.24 and Visual Studio 2019 (see [upgrade instructions](../../docs/unreal_upgrade.md))  
* Development: CUDA 9.0 and python 3.5.  
* Python libraries: Keras 2.1.2, TensorFlow 1.6.0.  
* Note: Newer versions of keras or tensorflow are recommended but can cause syntax errors.  
  
## What's inside  
  
![imitation learning](https://github.com/microsoft/airsim/wiki/images/technion/imitation_learning_example.gif)  
*Driving in simulation using trained imitation learning model, based on recorded data*  

Imitation learning includes the usage of labeled data as input to a training algorithm with the purpose of having the algorithm imitate the actions of people who recorded the data.  

![diagram](https://github.com/microsoft/airsim/wiki/images/technion/imitation_diagram.PNG)

This diagram is represented by these files:  

**cook_data.py**  
This file is responsible for preparing .h5 dataset files for the training procedure.  
The code rely on having two adjacent folders:  
'raw_data' - contains folders of recorded data by airsim's recording method.  
'cooked_data' - empty folder to store the .h5 files.  

The flag "COOK_ALL_DATA" gives the option to choose all subfolders, or exclude some of them.  

**train_model.py**  
This file is responsible to train a model using the .h5 dataset files.  
The code rely on having two adjacent folders:  
'cooked_data' - contains the .h5 dataset files.  
'models' - empty folder to store the generated models.  

The file will preprocess the data, add augmentations and create a neural network model that predicts the next steering angle.  

**drive_model.py**  
This file connects to the simulation in order to upload a trained model and drive using it.  
By using the predicted steering value, the code calculates related control parameters and maintain driving with steady velocities.

## Training Tips  
We recommend on using augmentation and recording techniques.  
We give here an example for two methods:  
- [CycleLight](https://github.com/Microsoft/AirSim/wiki/graphic_features) - Animation of a day light cycle in a changeable, potentially very short period of time.  
- Shifted images - Altering the camera’s position to the right or the left of the car, so that it can record images in extreme conditions. To simulate driving back to the center from those extreme situations, post-process the recorded angle of the steering accordingly (manually).  
