from pyo import *
from scipy.io import wavfile
import matplotlib.pyplot as plt
from scipy.io import wavfile
import os
import pysptk
import numpy
from libs import dspUtil
import scipy
import array

def unvoiced_starting_pts(f0,vSig,chunk_size):
    #register unvoiced signal starting points
    start = False
    cnt = 0
    for i in f0:
        if i == 0 and start == False:
            pt = cnt * chunk_size
            vSig["unvoicedStart"].append(pt)
            start = True
        if i != 0:
            start = False
        cnt = cnt + 1

def voiced_starting_pts(f0,vSig,chunk_size):
    #register voiced signal starting points
    start = False
    cnt = 0
    for i in f0:
        if i != 0 and start == False:
            pt = cnt * chunk_size
            vSig["voicedStart"].append(pt)
            start = True
        if i == 0:
            start = False
        cnt = cnt + 1

def length_voiced_region(lenSndArray, lengthVoiced,vSig):
    #length of each voiced region
    cnt = 0
    for i in vSig["voicedStart"]:
        if vSig["voicedStart"][0] != 0:
            if cnt+1 == len(vSig["unvoicedStart"]):
                lengthVoiced.append(lenSndArray-i)
            else:
                lengthVoiced.append(numpy.abs(vSig["unvoicedStart"][cnt+1]-i))
        else:
            if cnt == len(vSig["unvoicedStart"]):
                lengthVoiced.append(lenSndArray-i)
            else:
                lengthVoiced.append(numpy.abs(vSig["unvoicedStart"][cnt]-i))
        cnt = cnt + 1

def length_unvoiced_region(lenSndArray,lengthUnvoiced,vSig):
    #length of each unvoiced region
    cnt = 0
    for i in vSig["unvoicedStart"]:
        if vSig["unvoicedStart"][0] == 0:
            if cnt == len(vSig["voicedStart"]):
                lengthUnvoiced.append(lenSndArray-i)
            else:
                lengthUnvoiced.append(numpy.abs(vSig["voicedStart"][cnt]-i))
        else:
            if cnt+1 == len(vSig["voicedStart"]):
                lengthUnvoiced.append(lenSndArray-i)
            else:
                lengthUnvoiced.append(numpy.abs(vSig["voicedStart"][cnt+1]-i))
        cnt  = cnt + 1

#returns concatenated voiced regions array of floats
def get_voiced_region_array(sndarrayOne,vSig,lengthVoiced):
    xVoiced = []
    cnt = 0
    cntOne = 0
    for i in sndarrayOne:
        if cnt+1 != len(vSig["voicedStart"]) and cntOne == vSig["voicedStart"][cnt+1]:
            cnt = cnt + 1
        if cntOne >= vSig["voicedStart"][cnt] and cntOne <= vSig["voicedStart"][cnt]+ lengthVoiced[cnt]:
            xVoiced.append(i)
        cntOne = cntOne + 1
    return xVoiced

#returns concatenated unvoiced regions array of floats
def get_unvoiced_region_array(sndarrayOne,vSig,lengthUnvoiced):
    xUnvoiced = []
    cnt = 0
    cntOne = 0
    for i in sndarrayOne:
        if cnt+1 != len(vSig["unvoicedStart"]) and cntOne == vSig["unvoicedStart"][cnt+1]:
            cnt = cnt + 1
        if cntOne >= vSig["unvoicedStart"][cnt] and cntOne <= vSig["unvoicedStart"][cnt]+ lengthUnvoiced[cnt]:
            xUnvoiced.append(i)
        cntOne = cntOne + 1
    return xUnvoiced

#returns an array of lists of starting and ending points of voiced chunks
def get_voiced_region_chunks(vSig,lengthVoiced):
    voiced_regions = []
    for i in range(0,len(vSig["voicedStart"])):
        start = vSig["voicedStart"][i]
        end = start + lengthVoiced[i] - 1
        voiced_region = []
        voiced_region.append(start)
        voiced_region.append(end)
        voiced_regions.append(voiced_region)
    return voiced_regions

def get_voiced_region_chunks_two(sndarray,chunk_size):
    len_snd = len(sndarray)
    voiced_regions = []
    unvoiced_regions = []
    freq_array = []
    steps = int(numpy.ceil(len_snd/chunk_size))
    for i in range(0,steps):
        chunk_start = i * chunk_size
        if chunk_start >= len_snd-1:
            chunk_start = len_snd-1
        chunk_end = chunk_start + chunk_size - 1
        if chunk_end >= len_snd-1:
            chunk_end = len_snd-1
        if chunk_start == chunk_end:
            break
        f0 = get_freq_array(numpy.asarray(sndarray[chunk_start:chunk_end+1]).tolist(),44100,chunk_size)
        freq = f0[0]
        if freq != 0:
            voiced_regions.append([chunk_start,chunk_end])
            freq_array.append(freq)
        else:
            unvoiced_regions.append([chunk_start,chunk_end])
    return voiced_regions,unvoiced_regions,freq_array

#returns an array of lists of starting and ending points of unvoiced chunks
def get_unvoiced_region_chunks(vSig,lengthUnvoiced):
    unvoiced_regions = []
    for i in range(0,len(vSig["unvoicedStart"])):
        start = vSig["unvoicedStart"][i]
        end = start + lengthVoiced[i] -1
        unvoiced_region = []
        unvoiced_region.append(start)
        unvoiced_region.append(end)
        unvoiced_regions.append(unvoiced_region)
    return unvoiced_regions

def get_non_zero_freq_array(f0):
    non_zero_freq_array_info={"cnt":[],"f":[]}
    cnt = 0
    for i in f0:
        if i != 0:
            non_zero_freq_array_info["cnt"].append(cnt)
            non_zero_freq_array_info["f"].append(i)
        cnt = cnt + 1
    return non_zero_freq_array_info

def hertzToCents(f0):
    pitch_mean = numpy.mean(f0)
    cent = []
    for i in f0:
        cent.append(dspUtil.hertzToCents(i, pitch_mean))
    return cent

def find_mean(f0):
    return numpy.mean(f0)

def plot(x,y,total_len,desc):
    import matplotlib.pyplot as plt

    if isinstance(y[0],numpy.ndarray):
        y = get_one_channel_array(y)
    plt.plot(y,'-')
    plt.xlim(0, total_len)
    plt.legend()
    plt.show()

def get_signal_voiced_unvoiced_starting_info(x,f0,fs,chunk_size):
    vSig = {"unvoicedStart":[],"voicedStart":[]}
    unvoiced_starting_pts(f0,vSig,chunk_size)
    voiced_starting_pts(f0,vSig,chunk_size)
    return vSig

def get_signal_voiced_length_info(sndarrayOne,vSig):
    lengthVoiced = []
    length_voiced_region(len(sndarrayOne),lengthVoiced,vSig)
    return lengthVoiced

def get_signal_unvoiced_length_info(sndarrayOne,vSig):
    lengthUnvoiced = []
    length_unvoiced_region(len(sndarrayOne),lengthUnvoiced,vSig)
    return lengthUnvoiced

def get_one_channel_array(sndarray):
    xOne = []
    for i in sndarray:
        xOne.append(i[1])
    return xOne

def get_freq_array(sndarray,fs, chunk_size):
    new_sndarray = []
    for i in sndarray:
        new_sndarray.append(numpy.float64(i))
    sTime = time.time()
    f0 = pysptk.swipe(numpy.asarray(new_sndarray), fs, chunk_size, 65,500,0.001,1)
    eTime = time.time()
    # print "time taken by freq detector two is " + str(eTime-sTime)
    return f0

def merge_voiced_unvoiced_regions(xVoiced,xUnvoiced,vSig):
    total_len = len(xVoiced) + len(xUnvoiced)
    cnt = 0
    voicedCnt = 0
    unvoicedCnt = 0
    xMerged = []
    if vSig["voicedStart"][0] == 0:
        start = "voiced"
        switch = "voiced"
    else:
        start = "unvoiced"
        switch = "unvoiced"
    for i in range(total_len):
        if start == "unvoiced":
            if  switch == "voiced" and cnt+1 == len(vSig["unvoicedStart"]):
                # print "I am here 0 " + str(i) + " " + str(cnt) + " " + str(voicedCnt)
                xMerged.append(xVoiced[voicedCnt])
                voicedCnt = voicedCnt + 1
            elif switch == "unvoiced" and cnt == len(vSig["voicedStart"]):
                # print "I am here 1 " + str(i) + " " + str(cnt) + " " +  str(unvoicedCnt)
                xMerged.append(xUnvoiced[unvoicedCnt])
                unvoicedCnt = unvoicedCnt + 1
            elif switch == "unvoiced" and i < vSig["voicedStart"][cnt] :
                # print "I am here 2 " + str(i) + " " + str(cnt) + " " +  str(unvoicedCnt)
                xMerged.append(xUnvoiced[unvoicedCnt])
                unvoicedCnt = unvoicedCnt + 1
            elif switch == "voiced" and i < vSig["unvoicedStart"][cnt+1]:
                # print "I am here 3 " + str(i) + " " + str(cnt) + " " + str(voicedCnt)
                xMerged.append(xVoiced[voicedCnt])
                voicedCnt = voicedCnt + 1
            elif switch == "voiced":
                # print "I am here 0 " + str(i) + " " + str(cnt) + " " + str(unvoicedCnt)
                switch = "unvoiced"
                xMerged.append(xUnvoiced[unvoicedCnt])
                cnt = cnt + 1
            else:
                # print "I am here 1 " + str(i) + " " + str(cnt) + " " + str(voicedCnt)
                switch = "voiced"
                xMerged.append(xVoiced[voicedCnt])
        else:
            if cnt+1 == len(vSig["voicedStart"]):
                xMerged.append(xUnvoiced[unvoicedCnt])
                unvoicedCnt = unvoicedCnt + 1
            if cnt == len(vSig["unvoicedStart"]):
                xMerged.append(xVoiced[voicedCnt])
                voicedCnt = voicedCnt + 1
            if switch == "voiced" and i <= vSig["unvoicedStart"][cnt] :
                xMerged.append(xUnvoiced[unvoicedCnt])
                unvoicedCnt = unvoicedCnt + 1
            elif switch == "unvoiced" and i <= vSig["voicedStart"][cnt+1]:
                xMerged.append(xVoiced[voicedCnt])
                voicedCnt = voicedCnt + 1
            elif switch == "voiced":
                switch = "unvoiced"
                xMerged.append(xUnvoiced[unvoicedCnt])
                unvoicedCnt = unvoicedCnt + 1
            else:
                switch = "voiced"
                xMerged.append(xVoiced[voicedCnt])
                voicedCnt = voicedCnt + 1
                cnt += cnt
    return xMerged


def make_two_channels(sndarrayOne):
    sndarrayTwo = []
    for i in sndarrayOne:
        bz=[]
        for j in range(0,2):
            bz.append(i)
        sndarrayTwo.append(bz)
    return sndarrayTwo

def write_to_new_file(filename,sndarray):
    sndarray = numpy.asarray(sndarray)
    scipy.io.wavfile.write(filename,44100,sndarray)

if __name__ == "__main__":
    mydir = 'Voice-samples/'
    myfile = 'eric.wav'
    file = os.path.join(mydir, myfile)
    filenameVoiced = 'Voice-samples/ericVoiced.wav'
    filenameVoicedResampled = 'Voice-samples/kendraVoicedResampled.wav'
    fs, x = wavfile.read(file)
    chunk_size = 1024
    xOne = get_one_channel_array(x)

    #vSig is a dictonary of voiced and unvoiced regions starting points
    vSig = get_signal_voiced_unvoiced_starting_info(xOne,fs,chunk_size)
    lengthVoiced = get_signal_voiced_length_info(xOne,vSig)
    # lengthUnvoiced = get_signal_unvoiced_length_info(xOne,vSig)

    voiced_regions = get_voiced_region_chunks(vSig,lengthVoiced)
    # unvoiced_regions = get_unvoiced_region_chunks(vSig,lengthUnvoiced)
    # print unvoiced_regions

    #make a signal array with only voiced regions
    # xVoiced = get_voiced_region_array(xOne,vSig,lengthVoiced)
    # xUnvoiced = get_unvoiced_region_array(xOne,vSig,lengthUnvoiced)
    # print voiced_regions
    voiced_region_hi = []
    vr_start = 45056
    vr_stop = vr_start + (chunk_size*20) - 1
    for i in range(vr_start,vr_stop):
        voiced_region_hi.append(x[i])
    write_to_new_file(filenameVoiced,voiced_region_hi)

