###################################################################################################
#
#  Project:  Embedded Learning Library (ELL)
#  File:     wav_reader.py
#  Authors:  Chris Lovett
#
#  Requires: Python 3.x
#
###################################################################################################
import audioop
import math
import wave

import numpy as np
import pyaudio


class WavReader:
    def __init__(self, sample_rate=16000, channels=1, auto_scale=True):
        """ Initialize the wav reader with the type of audio you want returned.
        sample_rate  Rate you want audio converted to (default 16 kHz)
        channels     Number of channels you want output (default 1)
        auto_scale   Whether to scale numbers to the range -1 to 1.
        """
        self.input_stream = None
        self.audio = pyaudio.PyAudio()
        self.wav_file = None
        self.requested_channels = int(channels)
        self.requested_rate = int(sample_rate)
        self.buffer_size = 0
        self.sample_width = 0
        self.read_size = None
        self.dtype = None
        self.auto_scale = auto_scale
        self.audio_scale_factor = 1
        self.tail = None

    def open(self, filename, buffer_size, speaker=None):
        """ open a wav file for reading
        buffersize   Number of audio samples to return on each read() call
        speaker      Optional output speaker to send converted audio to so you can hear it.
        """
        self.speaker = speaker
        # open a stream on the audio input file.
        self.wav_file = wave.open(filename, "rb")
        self.cvstate = None
        self.read_size = int(buffer_size)
        self.actual_channels = self.wav_file.getnchannels()
        self.actual_rate = self.wav_file.getframerate()
        self.sample_width = self.wav_file.getsampwidth()
        # assumes signed integer used in raw audio, so for example, the max for 16bit is 2^15 (32768)
        if self.auto_scale:
            self.audio_scale_factor = 1 / pow(2, (8 * self.sample_width) - 1)

        if self.requested_rate == 0:
            raise Exception("Requested rate cannot be zero")
        self.buffer_size = int(math.ceil((self.read_size * self.actual_rate) / self.requested_rate))

        # convert int16 data to scaled floats
        if self.sample_width == 1:
            self.dtype = np.int8
        elif self.sample_width == 2:
            self.dtype = np.int16
        elif self.sample_width == 4:
            self.dtype = np.int32
        else:
            msg = "Unexpected sample width {}, can only handle 1, 2 or 4 byte audio"
            raise Exception(msg.format(self.sample_width))

        if speaker:
            # configure output stream to match what we are resampling to...
            audio_format = self.audio.get_format_from_width(self.sample_width)
            speaker.open(audio_format, self.requested_channels, self.requested_rate)

    def read_raw(self):
        """ Reads the next chunk of audio (returns buffer_size provided to open)
        It returns the raw data buffers converted to the target rate without any scaling.
        """
        if self.wav_file is None:
            return None

        data = self.wav_file.readframes(self.buffer_size)
        if len(data) == 0:
            return None

        if self.actual_rate != self.requested_rate:
            # convert the audio to the desired recording rate
            data, self.cvstate = audioop.ratecv(data, self.sample_width, self.actual_channels, self.actual_rate,
                                                self.requested_rate, self.cvstate)

        return self.get_requested_channels(data)

    def get_requested_channels(self, data):
        if self.requested_channels > self.actual_channels:
            raise Exception("Cannot add channels, actual is {}, requested is {}".format(
                self.actual_channels, self.requested_channels))

        if self.requested_channels < self.actual_channels:
            data = np.frombuffer(data, dtype=np.int16)
            channels = []
            # split into separate channels
            for i in range(self.actual_channels):
                channels += [data[i::self.actual_channels]]
            # drop the channels we don't want
            channels = channels[0:self.requested_channels]
            # zip the resulting channels back up.
            data = np.array(list(zip(*channels))).flatten()
            # convert back to packed bytes in PCM 16 format
            data = bytes(np.array(data, dtype=np.int16))

        return data

    def read(self):
        """ Reads the next chunk of audio (returns buffer_size provided to open)
        It returns the data converted to floating point numbers between -1 and 1, scaled by the range of
        values possible for the given audio format.
        """

        # deal with any accumulation of tails, if the tail grows to a full
        # buffer then return it!
        if self.tail is not None and len(self.tail) >= self.read_size:
            data = self.tail[0:self.read_size]
            self.tail = self.tail[self.read_size:]
            return data

        data = self.read_raw()
        if data is None:
            return None

        if self.speaker:
            self.speaker.write(data)

        data = np.frombuffer(data, dtype=self.dtype).astype(float)
        if self.tail is not None:
            # we have a tail from previous frame, so prepend it
            data = np.concatenate((self.tail, data))

        # now the caller needs us to stick to our sample_size contract, but when
        # rate conversion happens we can't be sure that 'data' is exactly that size.
        if len(data) > self.read_size:
            # usually one byte extra so add this to our accumulating tail
            self.tail = data[self.read_size:]
            data = data[0:self.read_size]

        if len(data) < self.read_size:
            # might have reached the end of a file, so pad with zeros.
            zeros = np.zeros(self.read_size - len(data))
            data = np.concatenate((data, zeros))

        return data * self.audio_scale_factor

    def close(self):
        if self.wav_file:
            self.wav_file.close()
            self.wav_file = None

    def is_closed(self):
        return self.wav_file is None
