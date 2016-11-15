#!/usr/bin/env python
import pyaudio
import contextlib
import rospy
import math
import struct
import numpy as np
from collections import deque
from std_msgs.msg import Float32
from audio_stream.msg import audiodata

from std_msgs.msg import UInt8MultiArray, MultiArrayDimension

@contextlib.contextmanager
def record_audio(channels, rate, chunk):
    """Opens a recording stream in a context manager."""
    audio_interface = pyaudio.PyAudio()
    audio_stream = audio_interface.open(
        format=pyaudio.paInt16, channels=channels, rate=rate,
        input=True, frames_per_buffer=chunk,
    )

    yield audio_stream

    audio_stream.stop_stream()
    audio_stream.close()
    audio_interface.terminate()



# Converts Stream (which is in byte format) to List of +ve and -ve Integers
def convData(V):
    count = len(V) / 2
    format = "%dh" % (count)
    shorts = struct.unpack(format, V)
    return shorts

# The Energy of Sound per chunk is calculated
def get_decibel(block):
    # get 1 val for each two char(byte)
    count = len(block) / 2
    SQUARE=0.0
    for i in block:
        SQUARE = SQUARE + math.pow(abs(i), 2)
    p = 20* math.log10(math.sqrt((SQUARE / count)))
    return p
    
# Get the frequency of the current Chunk
def getFreq(Maindata): # Purpose: Get the current Pitch of the file using fast fourier transform
    fftData = abs(np.fft.rfft(Maindata)) ** 2  # find the maximum
    which = fftData[1:].argmax() + 1
    # use quadratic interpolation around the max
    if which != len(fftData) - 1:
        y0, y1, y2 = np.log(fftData[which - 1:which + 2:])
        x1 = (y2 - y0) * .5 / (2 * y1 - y2 - y0)
        # find the frequency and output it
        FREQUENCY = (which + x1) * rate / chunk
    else:
        FREQUENCY = which * rate / chunk
    return FREQUENCY

# Return either 1 or 0 for Sudden Change and Similar decibels  respectively
def suddenChanges(dd):
    THRESHOLD = 25 # Test and modification of this constant is required
    if max(dd)-min(dd) > THRESHOLD:
        return 10
    else:
        return 0



if __name__ == '__main__':
    d = deque()
    loop = 0
    global loop
    rospy.init_node('audio_stream')
    audio_publisher = rospy.Publisher('speech_audio', UInt8MultiArray, queue_size=1)
    audio_feature = rospy.Publisher('/opencog/AudioFeature', audiodata, queue_size=1)
  
    rate = rospy.get_param('audio_rate', 16000)
    channels = rospy.get_param('audio_channels', 1)
    chunk = int(rate / 10)  # 100ms
    with record_audio(channels, rate, chunk) as audio_stream:
        while not rospy.is_shutdown():
            data = audio_stream.read(chunk)
            if not data:
                raise StopIteration()
            msg = UInt8MultiArray()
            msg.data = data
            msg.layout.dim = [
                MultiArrayDimension('wave', chunk, channels*2),
                MultiArrayDimension('channel', channels, channels)
            ]
            AudioData = []
            AudioData =convData(msg.data) # can publish it if raw data is needed 22050 Samples/s
            Decibel = get_decibel(AudioData)
            ''' Used Deque to keep the Audio-Energy Trend in the past few milli secs (1 sec)
                [-- <mask_size:2 ---> --> --]
            '''
            event = loop < 4 and d.append(Decibel) or \
                    d.popleft();d.append(Decibel);
            loop = loop + 1
            Frequency=getFreq(AudioData)
            msg2 = audiodata()
            msg2.Decibel = Decibel
            msg2.Frequency = Frequency
            msg2.suddenchange= suddenChanges(d)
            audio_feature.publish(msg2) 
                     
            audio_publisher.publish(msg)
