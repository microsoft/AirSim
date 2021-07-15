###################################################################################################
#
#  Project:  Embedded Learning Library (ELL)
#  File:     speaker.py
#  Authors:  Chris Lovett
#
#  Requires: Python 3.x
#
###################################################################################################

import pyaudio


class Speaker:
    def __init__(self):
        self.output_stream = None
        self.audio = pyaudio.PyAudio()

    def open(self, audio_format, num_channels, rate):
        # open speakers so we can hear what it is processing...
        self.output_stream = self.audio.open(format=audio_format,
                                             channels=num_channels,
                                             rate=rate,
                                             output=True)

    def write(self, data):
        if self.output_stream:
            self.output_stream.write(data)

    def close(self):
        if self.output_stream:
            self.output_stream.close()
            self.output_stream = None

    def is_closed(self):
        return self.output_stream is None
