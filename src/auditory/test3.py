import matplotlib.pyplot as plt
import numpy
from libs import dspUtil
import os
import ctypes
import pysptk
from scipy.io import wavfile
#
#
# class disable_file_system_redirection:
#     _disable = ctypes.windll.kernel32.Wow64DisableWow64FsRedirection
#     _revert = ctypes.windll.kernel32.Wow64RevertWow64FsRedirection
#
#     def __enter__(self):
#         self.old_value = ctypes.c_long()
#         self.success = self._disable(ctypes.byref(self.old_value))
#
#     def __exit__(self, type, value, traceback):
#         if self.success:
#             self._revert(self.old_value)
#
#
# with disable_file_system_redirection():
#     mydir = 'C:/Users/rediet/Documents/Vocie-samples/'
#     myfile = 'eric.wav'
#     file = os.path.join(mydir, myfile)
# fs, x = wavfile.read(file)
#
# y = []
# for i in x:
#     y.append(i[0])
#
# pitch = pysptk.swipe((numpy.asarray(y)).astype(numpy.float64), fs, 80, otype="f0")  # read in pitch track via swipe

# data = {}
# time = []
# frequency = []
# cent = []
# t = 0
# lenPitch= len(pitch)
# for p in pitch:
#     if p < 600:  # hz
#         # print (t), '*****************', p
#         time.append(t)
#         t = 1/lenPitch + t
#         frequency.append(p)
#
# data['Time'] = time
# data['Frequency'] = frequency
#
# pitch_mean = numpy.mean(pitch)
#
# for i in frequency:
#     cent.append(dspUtil.hertzToCents(i, baseFreq=pitch_mean))
# print (cent)
#
# plt.figure(1)
# plt.xlabel('Time (s)')
# plt.ylabel('pitch (cent)')
# plt.plot(data['Time'], cent)
# plt.gcf().autofmt_xdate()
# plt.title('Cent vs Time')
# plt.show()

import matplotlib.pyplot as plt
from libs import swipe
import numpy
from libs import dspUtil


class disable_file_system_redirection:
    _disable = ctypes.windll.kernel32.Wow64DisableWow64FsRedirection
    _revert = ctypes.windll.kernel32.Wow64RevertWow64FsRedirection

    def __enter__(self):
        self.old_value = ctypes.c_long()
        self.success = self._disable(ctypes.byref(self.old_value))

    def __exit__(self, type, value, traceback):
        if self.success:
            self._revert(self.old_value)


with disable_file_system_redirection():
    mydir = 'C:/Users/rediet/Documents/Vocie-samples/'
    myfile = 'eric.wav'
    file = os.path.join(mydir, myfile)
pitch = swipe.swipe(file, 75, 600) # read in pitch track via swipe
data ={}
time=[]
frequency=[]
cent =[]

print pitch
for (t, pitch) in pitch:
    if pitch < 600:  # hz
        print t, '*****************', pitch
        time.append(t)
        frequency.append(pitch)

data['Time'] = time
data['Frequency'] = frequency

pitch_mean= numpy.mean(pitch)
print pitch_mean
for i in frequency:
    cent.append(dspUtil.hertzToCents(i,pitch_mean))
print cent


plt.figure(1)
plt.xlabel('Time (s)')
plt.ylabel('pitch (cent)')
plt.plot(data['Time'], cent)
plt.gcf().autofmt_xdate()
plt.title('Cent vs Time')
plt.show()
