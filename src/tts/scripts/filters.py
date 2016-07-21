import math
import subprocess
import shutil
import scikits.audiolab
import scipy
import numpy
import pylab
from pyo import *

import scipy.fftpack
from scipy import pi

def display_audio(path):
    y_axis, sampling_rate, encoding = scikits.audiolab.wavread(path)
    x_axis = scipy.linspace(0, sampling_rate, len(y_axis))
    pylab.plot(x_axis, y_axis)
    pylab.show()

#def display_fft(path):
#    data, sampling_rate, encoding = scikits.audiolab.wavread(path)
#    pitch = abs(scipy.fft(data))
#    y_axis = 20*scipy.log10(scipy.absolute(pitch))
#    x_axis = scipy.linspace(0, sampling_rate, len(y_axis))
#    freqs = scipy.fftpack.fftfreq(len(y_axis))
#    #idx = numpy.argmax(numpy.abs(pitch))
#    #freq = freqs[idx]
#    #freq_in_hertz = abs(freq * sampling_rate)
#    #print(freq_in_hertz)
#    pylab.plot(freqs, y_axis, 'x')
#    pylab.show()
#
#def dsp(path):
#    signal, sampling_rate, encoding = scikits.audiolab.wavread(path)
#    t = scipy.linspace(0, sampling_rate, len(signal))
#
#    FFT = scipy.absolute(scipy.fft(signal))
#    freqs = scipy.fftpack.fftfreq(len(signal))
#
#    pylab.subplot(211)
#    pylab.plot(t, signal)
#    pylab.subplot(212)
#    pylab.plot(freqs,20*scipy.log10(FFT),'x')
#    pylab.xlim(0, 5000)
#    pylab.show()

def fft(path):
    data, sampling_rate, encoding = scikits.audiolab.wavread(path)
    max_freq = sampling_rate/2
    coef = scipy.absolute(scipy.fft(data))[:max_freq]
    freq = scipy.linspace(0, max_freq, len(coef), endpoint=False)
    return {'coef':coef, 'freq':freq}

def display_fft(path):
    FFT = fft(path)
    x_axis = FFT['freq']
    y_axis = 20*scipy.log10(FFT['coef'])
    pylab.plot(x_axis, y_axis)
    pylab.show()

def freq_mean(path):
    FFT = fft(path)
    return numpy.average(FFT['freq'], weights=FFT['coef'])

def freq_mode(path):
    FFT = fft(path)
    idx = numpy.argmax(numpy.abs(FFT['coef']))
    return FFT['freq'][idx]

#Alters input file to be two-channel and match the direction of the head
def balance_to_match_head(path):
    s = Server(audio="offline")
    
    # retrieve info about the sound
    info = sndinfo(path)
    dur, sr, chnls = info[1], info[2], info[3]
    chnls = 2
    fformat = ['WAVE', 'AIFF', 'AU', 'RAW', 'SD2', 'FLAC', 'CAF', 'OGG'].index(info[4])
    samptype = ['16 bit int', '24 bit int', '32 bit int', '32 bit float', 
                '64 bits float', 'U-Law encoded', 'A-Law encoded'].index(info[5])
    
    # set server parameters
    s.setSamplingRate(sr)
    s.setNchnls(chnls)
    s.boot()
    s.recordOptions(dur=dur, filename="./.balance_tmp.wav", fileformat=fformat, sampletype=samptype)
    direction = subprocess.check_output("rostopic echo /blender_api/get_head_target -n 1 | sed -n -e 's/^.*x: //p'", shell=True)
    f = float(direction) #[-0.9 pi, +0.9 pi]
    unit = f/(pi*0.9) #[-1,+1]
    biased = max(-1,min(1,unit*2)) #make the balance more noticeable #make the balance more noticeable
    balance = (unit+1)/2
    # processing
    sf = SfPlayer(path)
    lf = Sine(freq=0.5, mul=100)
    fs = FreqShift(sf, shift=lf)
    p = Pan(fs, outs=2, pan=balance).out()
    #bp = ButBP(sf, 1000, 2)
    #dt = Disto(bp, drive=0.9, slope=0.8)
    #mx = Interp(sf, dt, interp=0.5, mul=0.5)
    #sf.out()
    
    # start the render
    s.start()
    # cleanup
    s.shutdown()
    #os.rename('./.balance_tmp.wav',path)
    shutil.copy('./.balance_tmp.wav',path)
