#!/usr/bin/env python
from collections import deque
import pyaudio
import rospy
import math
import struct
import time
import numpy as np
from std_msgs.msg import Float32
from audiosysneeds.msg import audiodata

'''
   The main purpose of this Node is to publish Input Audio(Numpy array) and some
    of its features such as Decibel and Frequency. It publishes to two Topics 
    namely /opencog/AudioFeature and /opencog/suddenchange. The first one 
    contains Decibel and Frequency of a given chunk as Float. The second one 
    holds 0 for No Sudden Change Events and 10 for events which have sudden change. '''
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
        FREQUENCY = (which + x1) * RATE / CHUNK
    else:
        FREQUENCY = which * RATE / CHUNK
    return FREQUENCY

# Return either 1 or 0 for Sudden Change and Similar decibels  respectively
def suddenChanges(dd):
    THRESHOLD = 25 # Test and modification of this constant is required
    if max(dd)-min(dd) > THRESHOLD:
        return 10
    else:
        return 0

# Entry point to Publish Three Audio Features
if __name__ == '__main__':
    d = deque()
    try:
        Vsource = rospy.Publisher('/opencog/AudioFeature', audiodata, queue_size=1)
        Vchange = rospy.Publisher('/opencog/suddenchange', Float32, queue_size=1)
        rospy.init_node('AudioFeature', anonymous=True)
        rate = rospy.Rate(5)

        loop = 0
        global loop
        RECORD_SECONDS = 10
        initTime = time.time()

        frames = []
        WAVE_OUTPUT_FILENAME = str(time.time()).__add__('.wav')
        audio = pyaudio.PyAudio()
        FORMAT = pyaudio.paInt16
        CHANNELS = 2
        RATE = 44100
        CHUNK = 22050  # 1024
        stream = audio.open(format=FORMAT, channels=CHANNELS,
                    rate=RATE, input=True,
                    frames_per_buffer=CHUNK)
        while not rospy.is_shutdown():
            data = stream.read(CHUNK)
            frames.append(data)
            AudioData = []
            AudioData =convData(data) # can publish it if raw data is needed 22050 Samples/s
            Decibel = get_decibel(AudioData)

            ''' Used Deque to keep the Audio-Energy Trend in the past few milli secs (1 sec)
                [-- <mask_size:2 ---> --> --]
            '''
            event = loop < 4 and d.append(Decibel) or \
                    d.popleft();d.append(Decibel);
            loop = loop + 1
            Frequency=getFreq(AudioData)
            msg = audiodata()
            msg.Decibel = Decibel
            msg.Frequency = Frequency
            
            #feature= (Decibel) #+'_'+(Frequency)
            Vsource.publish(msg)
            Vchange.publish(suddenChanges(d))

            rate.sleep()
    except rospy.ROSInterruptException as e:
        print(e)










