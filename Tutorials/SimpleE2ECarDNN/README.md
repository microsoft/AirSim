# Autonomous Driving using End-to-End Supervised Deep Learning: A tutorial

### Authors:

**[Mitchell Spryn](https://www.linkedin.com/in/mitchell-spryn-57834545/)**, Software Engineer II, Microsoft

**[Aditya Sharma](https://www.linkedin.com/in/adityasharmacmu/)**, Program Manager, Microsoft

*(This tutorial has been developed and maintained by the Microsoft Deep Learning and Robotics Garage Chapter)*

## Overview

End-to-end deep learning is a modeling strategy that is a response to the success of deep neural networks. Unlike traditional Machine Learning and Computer Vision, this strategy is not built on feature engineering. Instead, it leverages the power of deep neural networks, along with recent hardware advances (GPUs, FPGAs etc.) to harness the incredible potential of large amounts of data. It is closer to a human-like learning approach than traditional ML as it lets the neural network map raw input to direct outputs. The only downside to this approach is that it requires a large amount of training data which makes it unsuitable for many common applications. Since simulators can generate an infinite amount of data, they are a perfect data source for end-to-end deep learning algorithms. If you wish to learn more about end-to-end deep learning, [this video](https://www.coursera.org/learn/machine-learning-projects/lecture/k0Klk/what-is-end-to-end-deep-learning) by Andrew Ng provides a nice overview of the topic.

Autonomous driving is a field that can highly benefit from the power of end-to-end deep learning. In order to achieve SAE Level 4 Autonomy, cars need to be trained on data worth millions of miles driven, something that is virtually impossible without a simulator. Additionally, a majority of predictions tasks needed for autonomous driving are regression based, making it a perfect use case for end-to-end learning.

In this tutorial, you will train a model to learn how to steer the car through a portion of the Landscape map using only one of the front facing webcams as inputs. Our strategy will be to perform some basic data analysis to get a feel for the dataset, and then train an end-to-end deep learning model to predict the correct steering control signals (a.k.a. "steering angle") given a frame from the webcam.  Such a task is usually considered the "hello world" of autonomous driving, but after finishing this tutorial you will have enough background to start exploring new ideas. Here's a short sample of the model in action. :

![car-driving](car_driving.gif)



## Structure of this tutorial

The code presented in this tutorial is written in [Keras](https://keras.io/), a high-level deep learning Python API capable of running on top of [CNTK](https://www.microsoft.com/en-us/cognitive-toolkit/), [TensorFlow](https://www.tensorflow.org/) or [Theano](http://deeplearning.net/software/theano/index.html). The fact that Keras lets you work with the deep learning framework of your choice, along with its simplicity of use, makes it an ideal choice for beginners, eliminating the learning curve that comes with most popular frameworks.

This tutorial is presented to you in the form of Python notebooks. Python notebooks make it easy for you to read instructions and explanations, and write and run code in the same file, all with the comfort of working in your browser window. You will go through the following notebooks in order:

**DataExplorationAndPreparation**

**TrainModel**

**TestModel**

If you have never worked with Python notebooks before, we highly recommend [checking out the documentation](http://jupyter-notebook-beginner-guide.readthedocs.io/en/latest/what_is_jupyter.html).

## Prerequisites and setup

#### Background needed

At the very least, you need to be familiar with the basics of how neural networks work. You do not need to know advanced concepts like LSTMs or Reinforcement Learning but you should know how Convolutional Networks work. A really good starting point to get a strong background in a very short amount of time is [this amazing book](http://neuralnetworksanddeeplearning.com/) written by Michael Nielsen. It is free, very short and available online. It can provide you a solid foundation in less than a week's time.

You should also be comfortable with Python. No programming expertise is necessary but at the very least, you should be able to read and understand code written in Python. 

#### Environment Setup

1. [Install Anaconda](https://conda.io/docs/user-guide/install/index.html) with Python 3.5 or higher.
2. [Install CNTK](https://docs.microsoft.com/en-us/cognitive-toolkit/Setup-CNTK-on-your-machine) or [install Tensorflow](https://www.tensorflow.org/install/install_windows)
3. [Install h5py](http://docs.h5py.org/en/latest/build.html)
4. [Install Keras](https://keras.io/#installation)
5. [Configure Keras backend](https://keras.io/backend/) to work with TensorFlow (default) or CNTK.

#### Hardware

It is highly recommended that a GPU is available for processing. While it is possible to train the model using just a CPU, it will take days to complete training. This tutorial was developed with a Nvidia GTX970 GPU, which resulted in a training time of ~45 minutes. 

If you do not have a GPU available, you can spin up a [Deep Learning VM on Azure](https://azuremarketplace.microsoft.com/en-us/marketplace/apps/microsoft-ads.dsvm-deep-learning), which comes with all the dependencies and libraries installed (use the provided py35 environment if you decide to use this).

#### Dataset

The dataset for the model is quite large. It is split into two parts, and can be found on the releases page. The first notebook will provide guidance on how to access the data once you have downloaded it. The final uncompressed data set size is approximately 3.25GB (which although is nothing compared to the petabytes of data needed to train an actual self-driving car, should be enough for the purpose of this tutorial).