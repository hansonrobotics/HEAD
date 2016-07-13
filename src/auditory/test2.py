

from pyo import *# s = Server().boot()
import os
import time

s = Server(nchnls=2).boot()
s.start()
mydir = 'C:/Users/rediet/Documents/Vocie-samples/'
myfile = 'kendra.wav'
file = os.path.join(mydir, myfile)
filename = 'C:/Users/rediet/Documents/Vocie-samples/kendraVU500.wav'
s.recstart(filename)
sf = SfPlayer(file, speed=1, loop=False).out()
time.sleep(1)
s.recstop()