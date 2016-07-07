import math
import subprocess
from pyo import *

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
    balance = (float(direction) + pi)/(pi*2)#Direction varies [-0.9 pi, +0.9 pi]
    # processing
    sf = SfPlayer(path)
    p = Pan(sf, outs=2, pan=balance).out()
    #bp = ButBP(sf, 1000, 2)
    #dt = Disto(bp, drive=0.9, slope=0.8)
    #mx = Interp(sf, dt, interp=0.5, mul=0.5)
    #sf.out()
    
    # start the render
    s.start()
    # cleanup
    s.shutdown()
    os.rename('./.balance_tmp.wav',path)
