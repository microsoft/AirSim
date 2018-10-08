from keras.preprocessing import image
import numpy as np
import keras.backend as K
import os
import cv2
import PIL
from PIL import Image
from PIL import ImageChops
import cv2


class DriveDataGenerator(image.ImageDataGenerator):
    def __init__(self,
                 featurewise_center=False,
                 samplewise_center=False,
                 featurewise_std_normalization=False,
                 samplewise_std_normalization=False,
                 zca_whitening=False,
                 zca_epsilon=1e-6,
                 rotation_range=0.,
                 width_shift_range=0.,
                 height_shift_range=0.,
                 shear_range=0.,
                 zoom_range=0.,
                 channel_shift_range=0.,
                 fill_mode='nearest',
                 cval=0.,
                 horizontal_flip=False,
                 vertical_flip=False,
                 rescale=None,
                 preprocessing_function=None,
                 data_format=None,
                 brighten_range=0):
        super(DriveDataGenerator, self).__init__(featurewise_center,
                 samplewise_center,
                 featurewise_std_normalization,
                 samplewise_std_normalization,
                 zca_whitening,
                 zca_epsilon,
                 rotation_range,
                 width_shift_range,
                 height_shift_range,
                 shear_range,
                 zoom_range,
                 channel_shift_range,
                 fill_mode,
                 cval,
                 horizontal_flip,
                 vertical_flip,
                 rescale,
                 preprocessing_function,
                 data_format)
        self.brighten_range = brighten_range

    def flow(self, x_images, x_prev_states = None, y=None, batch_size=32, shuffle=True, seed=None,
             save_to_dir=None, save_prefix='', save_format='png', zero_drop_percentage=0.5, roi=None):
        return DriveIterator(
            x_images, x_prev_states, y, self,
            batch_size=batch_size,
            shuffle=shuffle,
            seed=seed,
            data_format=self.data_format,
            save_to_dir=save_to_dir,
            save_prefix=save_prefix,
            save_format=save_format,
            zero_drop_percentage=zero_drop_percentage,
            roi=roi)
    
    def random_transform_with_states(self, x, seed=None):
        """Randomly augment a single image tensor.
        # Arguments
            x: 3D tensor, single image.
            seed: random seed.
        # Returns
            A tuple. 0 -> randomly transformed version of the input (same shape). 1 -> true if image was horizontally flipped, false otherwise
        """
        img_row_axis = self.row_axis
        img_col_axis = self.col_axis
        img_channel_axis = self.channel_axis

        is_image_horizontally_flipped = False

        # use composition of homographies
        # to generate final transform that needs to be applied
        if self.rotation_range:
            theta = np.pi / 180 * np.random.uniform(-self.rotation_range, self.rotation_range)
        else:
            theta = 0

        if self.height_shift_range:
            tx = np.random.uniform(-self.height_shift_range, self.height_shift_range) * x.shape[img_row_axis]
        else:
            tx = 0

        if self.width_shift_range:
            ty = np.random.uniform(-self.width_shift_range, self.width_shift_range) * x.shape[img_col_axis]
        else:
            ty = 0

        if self.shear_range:
            shear = np.random.uniform(-self.shear_range, self.shear_range)
        else:
            shear = 0

        if self.zoom_range[0] == 1 and self.zoom_range[1] == 1:
            zx, zy = 1, 1
        else:
            zx, zy = np.random.uniform(self.zoom_range[0], self.zoom_range[1], 2)

        transform_matrix = None
        if theta != 0:
            rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                        [np.sin(theta), np.cos(theta), 0],
                                        [0, 0, 1]])
            transform_matrix = rotation_matrix

        if tx != 0 or ty != 0:
            shift_matrix = np.array([[1, 0, tx],
                                     [0, 1, ty],
                                     [0, 0, 1]])
            transform_matrix = shift_matrix if transform_matrix is None else np.dot(transform_matrix, shift_matrix)

        if shear != 0:
            shear_matrix = np.array([[1, -np.sin(shear), 0],
                                    [0, np.cos(shear), 0],
                                    [0, 0, 1]])
            transform_matrix = shear_matrix if transform_matrix is None else np.dot(transform_matrix, shear_matrix)

        if zx != 1 or zy != 1:
            zoom_matrix = np.array([[zx, 0, 0],
                                    [0, zy, 0],
                                    [0, 0, 1]])
            transform_matrix = zoom_matrix if transform_matrix is None else np.dot(transform_matrix, zoom_matrix)

        if transform_matrix is not None:
            h, w = x.shape[img_row_axis], x.shape[img_col_axis]
            transform_matrix = image.transform_matrix_offset_center(transform_matrix, h, w)
            x = image.apply_transform(x, transform_matrix, img_channel_axis,
                                fill_mode=self.fill_mode, cval=self.cval)

        if self.channel_shift_range != 0:
            x = image.random_channel_shift(x,
                                     self.channel_shift_range,
                                     img_channel_axis)
        if self.horizontal_flip:
            if np.random.random() < 0.5:
                x = image.flip_axis(x, img_col_axis)
                is_image_horizontally_flipped = True

        if self.vertical_flip:
            if np.random.random() < 0.5:
                x = image.flip_axis(x, img_row_axis)
                
        if self.brighten_range != 0:
            random_bright = np.random.uniform(low = 1.0-self.brighten_range, high=1.0+self.brighten_range)
            
            img = cv2.cvtColor(x, cv2.COLOR_RGB2HSV)
            img[:, :, 2] = np.clip(img[:, :, 2] * random_bright, 0, 255)
            x = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)

        return (x, is_image_horizontally_flipped)



class DriveIterator(image.Iterator):
    """Iterator yielding data from a Numpy array.

    # Arguments
        x: Numpy array of input data.
        y: Numpy array of targets data.
        image_data_generator: Instance of `ImageDataGenerator`
            to use for random transformations and normalization.
        batch_size: Integer, size of a batch.
        shuffle: Boolean, whether to shuffle the data between epochs.
        seed: Random seed for data shuffling.
        data_format: String, one of `channels_first`, `channels_last`.
        save_to_dir: Optional directory where to save the pictures
            being yielded, in a viewable format. This is useful
            for visualizing the random transformations being
            applied, for debugging purposes.
        save_prefix: String prefix to use for saving sample
            images (if `save_to_dir` is set).
        save_format: Format to use for saving sample images
            (if `save_to_dir` is set).
    """

    def __init__(self, x_images, x_prev_states, y, image_data_generator,
                 batch_size=32, shuffle=False, seed=None,
                 data_format=None,
                 save_to_dir=None, save_prefix='', save_format='png', zero_drop_percentage = 0.5, roi = None):
        if y is not None and len(x_images) != len(y):
            raise ValueError('X (images tensor) and y (labels) '
                             'should have the same length. '
                             'Found: X.shape = %s, y.shape = %s' %
                             (np.asarray(x_images).shape, np.asarray(y).shape))

        if data_format is None:
            data_format = K.image_data_format()
        
        self.x_images = x_images
        
        self.zero_drop_percentage = zero_drop_percentage
        self.roi = roi
        
        if self.x_images.ndim != 4:
            raise ValueError('Input data in `NumpyArrayIterator` '
                             'should ave rank 4. You passed an array '
                             'with shape', self.x_images.shape)
        channels_axis = 3 if data_format == 'channels_last' else 1
        if self.x_images.shape[channels_axis] not in {1, 3, 4}:
            raise ValueError('NumpyArrayIterator is set to use the '
                             'data format convention "' + data_format + '" '
                             '(channels on axis ' + str(channels_axis) + '), i.e. expected '
                             'either 1, 3 or 4 channels on axis ' + str(channels_axis) + '. '
                             'However, it was passed an array with shape ' + str(self.x_images.shape) +
                             ' (' + str(self.x_images.shape[channels_axis]) + ' channels).')
        if x_prev_states is not None:
            self.x_prev_states = x_prev_states
        else:
            self.x_prev_states = None

        if y is not None:
            self.y = y
        else:
            self.y = None
        self.image_data_generator = image_data_generator
        self.data_format = data_format
        self.save_to_dir = save_to_dir
        self.save_prefix = save_prefix
        self.save_format = save_format
        self.batch_size = batch_size
        super(DriveIterator, self).__init__(x_images.shape[0], batch_size, shuffle, seed)

    def next(self):
        """For python 2.x.

        # Returns
            The next batch.
        """
        # Keeps under lock only the mechanism which advances
        # the indexing of each batch.
        with self.lock:
            index_array = next(self.index_generator)
        # The transformation of images is not under thread lock
        # so it can be done in parallel

        return self.__get_indexes(index_array)

    def __get_indexes(self, index_array):
        index_array = sorted(index_array)
        if self.x_prev_states is not None:
            batch_x_images = np.zeros(tuple([self.batch_size]+ list(self.x_images.shape)[1:]),
                                      dtype=K.floatx())
            batch_x_prev_states = np.zeros(tuple([self.batch_size]+list(self.x_prev_states.shape)[1:]), dtype=K.floatx())
        else:
            batch_x_images = np.zeros(tuple([self.batch_size] + list(self.x_images.shape)[1:]), dtype=K.floatx())

        if self.roi is not None:
            batch_x_images = batch_x_images[:, self.roi[0]:self.roi[1], self.roi[2]:self.roi[3], :]
            
        used_indexes = []
        is_horiz_flipped = []
        for i, j in enumerate(index_array):
            x_images = self.x_images[j]            
            
            if self.roi is not None:
                x_images = x_images[self.roi[0]:self.roi[1], self.roi[2]:self.roi[3], :]
                
                
            transformed = self.image_data_generator.random_transform_with_states(x_images.astype(K.floatx()))
            x_images = transformed[0]
            is_horiz_flipped.append(transformed[1])
            x_images = self.image_data_generator.standardize(x_images)
            batch_x_images[i] = x_images

            if self.x_prev_states is not None:
                x_prev_states = self.x_prev_states[j]
                
                if (transformed[1]):
                    x_prev_states[0] *= -1.0
                
                batch_x_prev_states[i] = x_prev_states
            
            used_indexes.append(j)

        if self.x_prev_states is not None:
            batch_x = [np.asarray(batch_x_images)]
        else:
            batch_x = np.asarray(batch_x_images)
            
        if self.save_to_dir:
            for i in range(0, self.batch_size, 1):
                hash = np.random.randint(1e4)
               
                img = image.array_to_img(batch_x_images[i], self.data_format, scale=True)
                fname = '{prefix}_{index}_{hash}.{format}'.format(prefix=self.save_prefix,
                                                                        index=1,
                                                                        hash=hash,
                                                                        format=self.save_format)
                img.save(os.path.join(self.save_to_dir, fname))

        batch_y = self.y[list(sorted(used_indexes))]
        idx = []
        num_of_close_samples = 0
        num_of_non_close_samples = 0
        for i in range(0, len(is_horiz_flipped), 1):
            if batch_y.shape[1] == 1:
                
                if (is_horiz_flipped[i]):
                    batch_y[i] *= -1

                if (np.isclose(batch_y[i], 0.5, rtol=0.005, atol=0.005)): 
                    num_of_close_samples += 1
                    if (np.random.uniform(low=0, high=1) > self.zero_drop_percentage):
                        idx.append(True)
                    else:
                        idx.append(False)
                else:
                    num_of_non_close_samples += 1
                    idx.append(True)
            else:
                
                if (batch_y[i][int(len(batch_y[i])/2)] == 1):
                    if (np.random.uniform(low=0, high=1) > self.zero_drop_percentage):
                        idx.append(True)
                    else:
                        idx.append(False)
                else:
                    idx.append(True)
                
                if (is_horiz_flipped[i]):
                    batch_y[i] = batch_y[i][::-1]

        batch_y = batch_y[idx]
        batch_x[0] = batch_x[0][idx]
        
        
        return batch_x, batch_y
        
    def _get_batches_of_transformed_samples(self, index_array):
        return self.__get_indexes(index_array)
        
