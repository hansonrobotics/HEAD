import numpy as np
from scipy.io.wavfile import write
from scipy.io import wavfile
import os
import pysptk
import numpy
from libs import dspUtil
import scipy

mydir = 'C:/Users/rediet/Documents/Vocie-samples/'
myfile = 'amy.wav'
file = os.path.join(mydir, myfile)
fs, x = wavfile.read(file)

# data = np.random.uniform(-1,1,44100) # 44100 random samples between -1 and 1
scaled = np.int16(x/np.max(np.abs(x)) * 32767)
write('C:/Users/rediet/Documents/Vocie-samples/test.wav', 44100, scaled)