import os
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
import tensorflow as tf
from keras.preprocessing.image import ImageDataGenerator
from keras.models import Sequential, Model
from keras.layers.convolutional import Convolution2D
from keras.layers import Conv2D, MaxPooling2D, Dropout, Flatten, Dense, Lambda, Input, concatenate
from keras.layers.core import Activation
from keras.layers.normalization import BatchNormalization
from keras.layers.advanced_activations import ELU, LeakyReLU
from keras.optimizers import Adam, SGD, Adamax, Nadam
from keras.callbacks import ReduceLROnPlateau, ModelCheckpoint, CSVLogger, EarlyStopping
import keras.backend as K
from keras.preprocessing import image
from keras_tqdm import TQDMNotebookCallback
import json
import os
import numpy as np
import pandas as pd
from Generator import DriveDataGenerator
import h5py
import math

# Hyper-parameters
batch_size = 32
learning_rate = 0.0001
number_of_epochs = 500

# Activation functions
activation = 'relu'
out_activation = 'sigmoid'

#Stop training if in the last 20 epochs, there was no change of the best recorded validation loss
training_patience = 20


# << The directory containing the cooked data from the previous step >>
COOKED_DATA_DIR = './cooked_data/'

# << The directory in which the model output will be placed >>
MODEL_OUTPUT_DIR = './models/'

train_dataset = h5py.File(os.path.join(COOKED_DATA_DIR, 'train.h5'), 'r')
eval_dataset = h5py.File(os.path.join(COOKED_DATA_DIR, 'eval.h5'), 'r')

num_train_examples = train_dataset['image'].shape[0]
num_eval_examples = eval_dataset['image'].shape[0]

# Use ROI of [78,144,27,227] for FOV 60 with Formula car
data_generator = DriveDataGenerator(rescale=1./255., horizontal_flip=False, brighten_range=0.4)
train_generator = data_generator.flow\
    (train_dataset['image'], train_dataset['previous_state'], train_dataset['label'], batch_size=batch_size, zero_drop_percentage=0.95, roi=[78,144,27,227])
eval_generator = data_generator.flow\
    (eval_dataset['image'], eval_dataset['previous_state'], eval_dataset['label'], batch_size=batch_size, zero_drop_percentage=0.95, roi=[78,144,27,227])

[sample_batch_train_data, sample_batch_test_data] = next(train_generator)

image_input_shape = sample_batch_train_data[0].shape[1:]

pic_input = Input(shape=image_input_shape)

#Network definition
img_stack = Conv2D(24, (5, 5), name="conv1", strides=(2, 2), padding="valid", activation=activation, kernel_initializer="he_normal")(pic_input)
img_stack = Conv2D(36, (5, 5), name="conv2", strides=(2, 2), padding="valid", activation=activation, kernel_initializer="he_normal")(img_stack)
img_stack = Conv2D(48, (5, 5), name="conv3", strides=(2, 2), padding="valid", activation=activation, kernel_initializer="he_normal")(img_stack)

img_stack = Dropout(0.5)(img_stack)

img_stack = Conv2D(64, (3, 3), name="conv4", strides=(1, 1), padding="valid", activation=activation, kernel_initializer="he_normal")(img_stack)
img_stack = Conv2D(64, (3, 3), name="conv5", strides=(1, 1), padding="valid", activation=activation, kernel_initializer="he_normal")(img_stack)

img_stack = Flatten(name = 'flatten')(img_stack)

img_stack = Dense(100, name="fc2", activation=activation,kernel_initializer="he_normal")(img_stack)
img_stack = Dense(50, name="fc3", activation=activation, kernel_initializer="he_normal")(img_stack)
img_stack = Dense(10, name="fc4", activation=activation, kernel_initializer="he_normal")(img_stack)
img_stack = Dense(1, name="output", activation = out_activation, kernel_initializer="he_normal")(img_stack)

adam = Adam(lr=learning_rate, beta_1=0.9, beta_2=0.999, epsilon=1e-08, decay=0.0)

model = Model(inputs=[pic_input], outputs=img_stack)
model.compile(optimizer=adam, loss='mse')

model.summary()

plateau_callback = ReduceLROnPlateau(monitor="val_loss", factor=0.5, patience=3, min_lr=learning_rate, verbose=1)
csv_callback = CSVLogger(os.path.join(MODEL_OUTPUT_DIR, 'training_log.csv'))
checkpoint_filepath = os.path.join(MODEL_OUTPUT_DIR, 'fresh_models', '{0}_model.{1}-{2}.h5'.format('model', '{epoch:02d}', '{val_loss:.7f}'))
checkpoint_callback = ModelCheckpoint(checkpoint_filepath, save_best_only=True, verbose=1)
early_stopping_callback = EarlyStopping(monitor="val_loss", patience=training_patience, verbose=1)
callbacks=[plateau_callback, csv_callback, checkpoint_callback, early_stopping_callback, TQDMNotebookCallback()]

history = model.fit_generator(train_generator, steps_per_epoch=num_train_examples//batch_size, epochs=number_of_epochs, callbacks=callbacks,\
                   validation_data=eval_generator, validation_steps=num_eval_examples//batch_size, verbose=2)
