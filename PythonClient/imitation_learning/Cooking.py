import random
import csv
from PIL import Image
import numpy as np
import pandas as pd
import sys
import os
import errno
from collections import OrderedDict
import h5py
from pathlib import Path
import copy
import re


# This constant is used as an upper bound  for normalizing the car's speed to be between 0 and 1 
MAX_SPEED = 70.0



def checkAndCreateDir(full_path):
    """Checks if a given path exists and if not, creates the needed directories.
            Inputs:
                full_path: path to be checked
    """
    if not os.path.exists(os.path.dirname(full_path)):
        try:
            os.makedirs(os.path.dirname(full_path))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
                
def readImagesFromPath(image_names):
    """ Takes in a path and a list of image file names to be loaded and returns a list of all loaded images after resize.
           Inputs:
                image_names: list of image names
           Returns:
                List of all loaded and resized images
    """
    returnValue = []
    for image_name in image_names:
        im = Image.open(image_name)
        imArr = np.asarray(im)
        
        #Remove alpha channel if exists
        if len(imArr.shape) == 3 and imArr.shape[2] == 4:
            if (np.all(imArr[:, :, 3] == imArr[0, 0, 3])):
                imArr = imArr[:,:,0:3]
        if len(imArr.shape) != 3 or imArr.shape[2] != 3:
            print('Error: Image', image_name, 'is not RGB.')
            sys.exit()            

        returnIm = np.asarray(imArr)

        returnValue.append(returnIm)
    return returnValue
    
    
    
def splitTrainValidationAndTestData(all_data_mappings, split_ratio=(0.7, 0.2, 0.1)):
    """Simple function to create train, validation and test splits on the data.
            Inputs:
                all_data_mappings: mappings from the entire dataset
                split_ratio: (train, validation, test) split ratio

            Returns:
                train_data_mappings: mappings for training data
                validation_data_mappings: mappings for validation data
                test_data_mappings: mappings for test data

    """
    if round(sum(split_ratio), 5) != 1.0:
        print("Error: Your splitting ratio should add up to 1")
        sys.exit()

    train_split = int(len(all_data_mappings) * split_ratio[0])
    val_split = train_split + int(len(all_data_mappings) * split_ratio[1])

    train_data_mappings = all_data_mappings[0:train_split]
    validation_data_mappings = all_data_mappings[train_split:val_split]
    test_data_mappings = all_data_mappings[val_split:]

    return [train_data_mappings, validation_data_mappings, test_data_mappings]
    
def generateDataMapAirSim(folders):
    """ Data map generator for simulator(AirSim) data. Reads the driving_log csv file and returns a list of 'center camera image name - label(s)' tuples
           Inputs:
               folders: list of folders to collect data from

           Returns:
               mappings: All data mappings as a dictionary. Key is the image filepath, the values are a 2-tuple:
                   0 -> label(s) as a list of double
                   1 -> previous state as a list of double
    """

    all_mappings = {}
    for folder in folders:
        print('Reading data from {0}...'.format(folder))
        current_df = pd.read_csv(os.path.join(folder, 'airsim_rec.txt'), sep='\t')
        
        for i in range(1, current_df.shape[0] - 1): 

            
            if current_df.iloc[i-1]['Brake'] != 0:   # Consider only training examples without breaks
                continue


            norm_steering = [ (float(current_df.iloc[i-1][['Steering']]) + 1) / 2.0 ]  # Normalize steering: between 0 and 1
            norm_throttle = [ float(current_df.iloc[i-1][['Throttle']]) ]
            norm_speed = [ float(current_df.iloc[i-1][['Speed (kmph)']]) / MAX_SPEED ]  # Normalize speed: between 0 and 1


            previous_state = norm_steering + norm_throttle + norm_speed   # Append lists


            #compute average steering over 3 consecutive recorded images, this will serve as the label

            norm_steering0 = (float(current_df.iloc[i][['Steering']]) + 1) / 2.0
            norm_steering1 = (float(current_df.iloc[i+1][['Steering']]) + 1) / 2.0

            temp_sum_steering = norm_steering[0] + norm_steering0 + norm_steering1
            average_steering = temp_sum_steering / 3.0

            current_label = [average_steering]
            
            image_filepath = os.path.join(os.path.join(folder, 'images'), current_df.iloc[i]['ImageName']).replace('\\', '/')
            
            if (image_filepath in all_mappings):
                print('Error: attempting to add image {0} twice.'.format(image_filepath))
            
            all_mappings[image_filepath] = (current_label, previous_state)

    mappings = [(key, all_mappings[key]) for key in all_mappings]
    
    random.shuffle(mappings) 
    
    return mappings


def generatorForH5py(data_mappings, chunk_size=32):
    """
    This function batches the data for saving to the H5 file
    """
    for chunk_id in range(0, len(data_mappings), chunk_size):
        # Data is expected to be a dict of <image: (label, previousious_state)>
        data_chunk = data_mappings[chunk_id:chunk_id + chunk_size]
        if (len(data_chunk) == chunk_size):
            image_names_chunk = [a for (a, b) in data_chunk]
            labels_chunk = np.asarray([b[0] for (a, b) in data_chunk])
            previous_state_chunk = np.asarray([b[1] for (a, b) in data_chunk])
            
            #Flatten and yield as tuple
            yield (image_names_chunk, labels_chunk.astype(float), previous_state_chunk.astype(float))
            if chunk_id + chunk_size > len(data_mappings):
                raise StopIteration
    raise StopIteration
    
    
def saveH5pyData(data_mappings, target_file_path, chunk_size):
    """
    Saves H5 data to file
    """
    gen = generatorForH5py(data_mappings,chunk_size)

    image_names_chunk, labels_chunk, previous_state_chunk = next(gen)
    images_chunk = np.asarray(readImagesFromPath(image_names_chunk))
    row_count = images_chunk.shape[0]

    checkAndCreateDir(target_file_path)
    with h5py.File(target_file_path, 'w') as f:

        # Initialize a resizable dataset to hold the output
        images_chunk_maxshape = (None,) + images_chunk.shape[1:]
        labels_chunk_maxshape = (None,) + labels_chunk.shape[1:]
        previous_state_maxshape = (None,) + previous_state_chunk.shape[1:]

        dset_images = f.create_dataset('image', shape=images_chunk.shape, maxshape=images_chunk_maxshape,
                                chunks=images_chunk.shape, dtype=images_chunk.dtype)

        dset_labels = f.create_dataset('label', shape=labels_chunk.shape, maxshape=labels_chunk_maxshape,
                                       chunks=labels_chunk.shape, dtype=labels_chunk.dtype)
        
        dset_previous_state = f.create_dataset('previous_state', shape=previous_state_chunk.shape, maxshape=previous_state_maxshape,
                                       chunks=previous_state_chunk.shape, dtype=previous_state_chunk.dtype)
                                       
        dset_images[:] = images_chunk
        dset_labels[:] = labels_chunk
        dset_previous_state[:] = previous_state_chunk

        for image_names_chunk, label_chunk, previous_state_chunk in gen:
            image_chunk = np.asarray(readImagesFromPath(image_names_chunk))
            
            # Resize the dataset to accommodate the next chunk of rows
            dset_images.resize(row_count + image_chunk.shape[0], axis=0)
            dset_labels.resize(row_count + label_chunk.shape[0], axis=0)
            dset_previous_state.resize(row_count + previous_state_chunk.shape[0], axis=0)
            # Create the next chunk
            dset_images[row_count:] = image_chunk
            dset_labels[row_count:] = label_chunk
            dset_previous_state[row_count:] = previous_state_chunk

            # Increment the row count
            row_count += image_chunk.shape[0]
            
            
def cook(folders, output_directory, train_eval_test_split, chunk_size):
    """ Primary function for data pre-processing. Reads and saves all data as h5 files.
            Inputs:
                folders: a list of all data folders
                output_directory: location for saving h5 files
                train_eval_test_split: dataset split ratio
    """
    output_files = [os.path.join(output_directory, f) for f in ['train.h5', 'eval.h5', 'test.h5']]
    if (any([os.path.isfile(f) for f in output_files])):
       print("Preprocessed data already exists at: {0}. Skipping preprocessing.".format(output_directory))

    else:
        all_data_mappings = generateDataMapAirSim(folders)
        
        split_mappings = splitTrainValidationAndTestData(all_data_mappings, split_ratio=train_eval_test_split)
        
        for i in range(0, len(split_mappings)-1, 1):
            print('Processing {0}...'.format(output_files[i]))
            saveH5pyData(split_mappings[i], output_files[i], chunk_size)
            print('Finished saving {0}.'.format(output_files[i]))