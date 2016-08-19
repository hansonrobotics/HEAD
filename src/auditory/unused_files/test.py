__author__ = 'zelalem'
# #
import time
import numpy, pygame.mixer, pygame.sndarray
from scikits.samplerate import resample
import os

pygame.mixer.init(35000, -16, 2, 512)
mydir = 'C:/Users/rediet/Documents/Vocie-samples/'
myfile = 'amy.wav'
file = os.path.join(mydir, myfile)
sound = pygame.mixer.Sound(file)
snd_array = pygame.sndarray.array(sound)
# print snd_array
snd_resample = resample(snd_array, 0.9, "sinc_best").astype(snd_array.dtype) #sinc_fastest
# print snd_resample
# take the resampled array, make it an object and stop playing after 2 seconds.
snd_out = pygame.sndarray.make_sound(snd_resample)
snd_out.play()
time.sleep(10)

