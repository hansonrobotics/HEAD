
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


yfile = []
for i in x:
    # if (i[0] > 30) or (i[0] < -30):
    yfile.append(i)

# scipy.io.wavfile.write('C:/Users/rediet/Documents/Vocie-samples/xenencounter_23sin.wav',fs,numpy.asarray(yfile))
#
yfilenew = []
for i in yfile:
    yfilenew.append(i[1])
print len(yfilenew)
yfilenew = numpy.asarray(yfilenew).astype(numpy.float32)
# hopsize = 10 # 5ms for 16kHz data
f0 = pysptk.swipe(yfilenew.astype(numpy.float64), fs, 20,10,600,0.3,1)
print len(yfilenew)/len(f0)
fnew = []
cnti = []
cnt = 0
for i in f0:
    if i != 0:
        cnti.append(cnt)
        fnew.append(i)
    cnt = cnt + 1
pitch_mean = numpy.mean(fnew)
cent = []
for i in fnew:
    cent.append(dspUtil.hertzToCents(i, pitch_mean))

import matplotlib.pyplot as plt
plt.plot(cnti, cent,'-o', linewidth=2, label="F0 trajectory estimated by SWIPE'")
plt.xlim(0, len(f0))
plt.legend()
plt.show()