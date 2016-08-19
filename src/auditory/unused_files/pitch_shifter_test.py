import os

from scipy.io import wavfile
from scipy.io.wavfile import write

from src.auditory.unused_files.pitch_shifter import pitchshift

mydir = 'Voice-samples/'
myfile = 'amy.wav'
file = os.path.join(mydir, myfile)
fs, x = wavfile.read(file)
xnew = []
for i in x:
    xnew.append(i[1])
xnew = pitchshift(xnew,0.5)
write('Voice-samples/amyShifted.wav', 44100, xnew)