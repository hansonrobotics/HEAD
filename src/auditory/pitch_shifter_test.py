from pitch_shifter import pitchshift
from scipy.io import wavfile
import os
from scipy.io.wavfile import write
import pyo

mydir = 'C:/Users/rediet/Documents/Vocie-samples/'
myfile = 'amy.wav'
file = os.path.join(mydir, myfile)
fs, x = wavfile.read(file)
xnew = []
for i in x:
    xnew.append(i[1])
xnew = pitchshift(xnew,0.5)
write('C:/Users/rediet/Documents/Vocie-samples/amyShifted.wav', 44100, xnew)