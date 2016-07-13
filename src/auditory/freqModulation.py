from pyo import *
import wave, struct,math
import array
import voiced_unvoiced as voi
from scipy.io import wavfile
import scipy
import numpy

#
def FreqModArray(signal,timeArray, carrierFreq, filenameFM):
    signal_fm_array = integratorFM(carrierFreq,100.0,signal,timeArray)
    scipy.io.wavfile.write(filenameFM,44100,numpy.asarray(signal_fm_array))
    return signal_fm_array

#
def integratorFM(carrierFreq, bandwidth, sndarray,timeArray):
    fs = 44100.0
    integ_base = 0
    integ_base2 = 0
    signal_fm_array = []
    cnt = 0
    for n in timeArray:
    # for n in range(0, baseband.getnframes()):
    #     base = array.array('h', baseband.readframes(1))
        # Base signal is integrated (not mandatory, but improves
        # volume at demodulation side)
        integ_base += sndarray[cnt]/3.0
        # integ_base2 += sndarray[cnt]/2.0
        # The FM trick: time (n) is multiplied only by carrier freq;
        # the frequency deviation is added afterwards.00
        # print 2 * math.pi * FM_CARRIER * (n /fs) + 2 * math.pi * FM_BAND * integ_base2 / fs
        nfs = numpy.float32(n)
        signal_fm = math.cos(2 * math.pi * carrierFreq * nfs + 2 * math.pi * bandwidth * integ_base)
        signal_fm_array.append(signal_fm*3000.0)
        cnt = cnt + 1
        # fm.writeframes(struct.pack('h', signal_fm *32767.0))
    return signal_fm_array

def FreqMod(filename,filenameFM):
    # baseband = wave.open(filename, "r")
    # fm = wave.open(filenameFM,"w")
    fs, x = wavfile.read(filename)
    sndarrayOne = voi.get_one_channel_array(x)
    timeArray = numpy.arange(0,len(sndarrayOne),1/fs)
    fs = 44100.0
    FM_BAND = 100.0
    FM_CARRIER = 8.5
    # fm.setnchannels(1)
    # fm.setsampwidth(2)
    # fm.setframerate(fs)

    signal_fm_array = integratorFM(FM_CARRIER,FM_BAND,sndarrayOne,timeArray)
    scipy.io.wavfile.write(filenameFM,fs,numpy.asarray(signal_fm_array))

if __name__ == "__main__":
    import wave, struct, math
    import array
    import voiced_unvoiced as voi
    import os
    from scipy.io import wavfile
    import numpy
    import pysptk
    from libs import dspUtil
    import freqModulation as fm

    filename= "C:/Users/rediet/Documents/Vocie-samples/amy.wav"
    filenameFM = "C:/Users/rediet/Documents/Vocie-samples/amyFM.wav"

    buckets = []
    for i in xrange(0,100000):
        buckets.append(10000.0)
    for i in xrange(0,100000):
        buckets.append(2.0)
    for i in xrange(0,100000):
        buckets.append(4000.0)
    for i in xrange(0,100000):
        buckets.append(9000.0)
    for i in xrange(0,100000):
        buckets.append(3.0)
    timeArray = numpy.arange(0,len(buckets),1)
    # fm.FreqMod(filename,filenameFM)
    fm.FreqModArray(buckets,timeArray,3,filenameFM)

    mydir = 'C:/Users/rediet/Documents/Vocie-samples/'
    myfile = 'amyFM.wav'
    file = os.path.join(mydir, myfile)
    fs, x = wavfile.read(file)
    voi.plot(timeArray,x,len(timeArray))