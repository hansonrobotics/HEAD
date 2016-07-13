import wave, struct, math
import array
import voiced_unvoiced as voi
import os
from scipy.io import wavfile
import numpy
import pysptk
from libs import dspUtil
import freqModulation as fm
from pyo import *
import pitch_shifter as ps
import matplotlib as plt
import heapq

def get_freq_regions_from_voiced_region(voiced_region_range, chunk_size):
    freq_regions = []
    rng = (voiced_region_range[1] - voiced_region_range[0] + 1)/chunk_size
    for i in range(0,rng):
        start = i * chunk_size
        end = start + chunk_size - 1
        freq_region = []
        freq_region.append(start)
        freq_region.append(end)
        freq_regions.append(freq_region)
    return freq_regions

def get_freq_chunk(sndarray,chunk_start,chunk_size):
    f0 = voi.get_freq_array(sndarray,44100,chunk_size)
    spot = chunk_start/chunk_size
    return f0[spot]

def get_zero_derivative_indices_fast(arr):
    indices = []
    pitch_points = []
    cnt = 0
    for i in arr:
        if cnt == 0:
            if len(arr) > 1 and arr[1] <= arr[0]:
                pitch_points.append(cnt)
        if cnt != 0:
            x = indices.pop()
            if i >= x:
                indices.append(i)
            else:
                pitch_points.append(cnt)
        if cnt == len(arr)-1 and arr[cnt] >= arr[cnt-1]:
            pitch_points.append(cnt)
        cnt = cnt + 1
        indices.append(i)
    return pitch_points


def get_zero_derivative_indices(arr):
    indices = []
    i = 0
    while i <=  len(arr)-3:
        # print arr[i:i+3]
        argMax = numpy.argmax(arr[i:i+3])
        if argMax == 1:
            indices.append(i+1)
        if i == 0 and argMax == 0:
            indices.append(i)
        if i == len(arr)-3 and argMax == 2:
            indices.append(i+2)
        i = i + 1
    return indices

#a chunk is an array of samples of size chunk_size
def get_pitch_marks_freq_chunk(sndarray,freq,chunk_start,chunk_end, numPitchMarks):
    if chunk_end >= len(sndarray):
        chunk_end = len(sndarray)
    # x = sndarray[chunk_start:chunk_end]
    # y = numpy.arange(0,len(x),1)
    # voi.plot(y,x,len(x),"signal amplitude")
    pitch_marks = []
    window_pitch_obj = {"windows":[],"pitch_marks":[]}
    factor = 0.7
    fctr = 1 - factor
    if freq == 0:
        return pitch_marks, window_pitch_obj
    period = float(1)/float(freq)

    # print "period " + str(period)

    periodToSamps = period * 44100
    periodToSamps = numpy.int(periodToSamps)
    # print "freq " + str(freq)
    # print "period To Samps " + str(periodToSamps)

    # print "periodToSamps " + str(periodToSamps)
    tm = numpy.argmax(sndarray[chunk_start:chunk_end+1])
    tm = tm + chunk_start
    originalTM = tm


    # print "freq " + str(freq)
    # print "chunk_start " + str(chunk_start)
    # print "chunk_end " + str(chunk_end)

    shift = numpy.int(fctr*periodToSamps)

    # print "shift " + str(shift)
    # print "tm " + str(tm)

    window_start = tm - shift

    # print "window_start0 "  + str(window_start)

    if window_start <= chunk_start:
        window_start = chunk_start
    window_end = tm + shift

    # print "window_start1 "  + str(window_start)
    # print "window_end0 "  + str(window_end)

    cnt = 0
    while tm <= chunk_end-120:
        print "window_start and window end 1 "  + str(window_start) + "-" + str(window_end) + "tm " + str(tm)
        if window_end >= chunk_end:
            window_end = chunk_end
        if window_end <= window_start:
            break
        # if window_end >= chunk_end-50:
        #     break
        # print "window_start and window end 1  "  + str(window_start) + "-" + str(window_end)
        # print "window_end1 "  + str(window_end)
        window_pitch_obj["windows"].append([window_start,window_end])
        window_pitch_obj["pitch_marks"].append([tm])
        pitch_marks.append(tm)
        ind = get_zero_derivative_indices(sndarray[window_start:window_end+1])
        # print ind
        vals = []

        for i in ind:
            vals.append(sndarray[i+window_start])
        # print vals

        pitchMarksInd = numPitchMarks
        largest = heapq.nlargest(pitchMarksInd, vals)
        print largest
        if len(largest) < pitchMarksInd:
            pitchMarksInd = len(largest)

        while pitchMarksInd > 1 :
            indVals = numpy.where(vals == largest[pitchMarksInd-1])[0]
            spot_chunk = ind[indVals[0]]
            # print "spot_chunk " + str(spot_chunk)
            window_pitch_obj["pitch_marks"][cnt].append(window_start + spot_chunk)
            pitch_marks.append(window_start + spot_chunk)
            pitchMarksInd = pitchMarksInd - 1
        print "pitch_marks_in_window " + str(window_pitch_obj["pitch_marks"][cnt])
        shift = numpy.int(factor*periodToSamps)
        window_start = tm + shift
        window_end = tm + 2 * periodToSamps - shift
        # print "window_start and window end 0 "  + str(window_start) + "-" + str(window_end)
        if window_start >= chunk_end:
            window_start = chunk_end
        if window_end >= chunk_end:
            window_end = chunk_end
        if len(sndarray[window_start:window_end]) > 0:
            tm = numpy.argmax(sndarray[window_start:window_end])
        tm = tm + window_start
        cnt = cnt + 1
        # print "window_start2 "  + str(window_start)
        # print "window_end2 "  + str(window_end)
        # print "chunk_start2 " + str(chunk_start)
        # print "chunk_end2 " + str(chunk_end)

    tm = originalTM
    window_start = tm - shift

    # print "window_start0 "  + str(window_start)

    if window_start <= chunk_start:
        window_start = chunk_start
    window_end = tm + shift
    while tm >= chunk_start+120:
        shift = numpy.int(factor*periodToSamps)
        window_end = tm - shift
        window_start = tm - 2 * periodToSamps + shift
        print "window_start and window end 2  "  + str(window_start) + "-" + str(window_end) + "tm " + str(tm)
        if window_start <= chunk_start:
            window_start = chunk_start
        # print "window_start3 "  + str(window_start)
        # print "window_end3 "  + str(window_end)
        if window_end <= window_start:
            break
        # if window_start <= 30 + chunk_start:
        #     break
        # print "window_start and window end 2  "  + str(window_start) + "-" + str(window_end)
        window_pitch_obj["windows"].append([window_start,window_end])
        tm = numpy.argmax(sndarray[window_start:window_end])
        tm = tm + window_start
        # if tm <= chunk_start + 50:
        #     break
        pitch_marks.append(tm)
        window_pitch_obj["pitch_marks"].append([tm])
        # print "window_end1 "  + str(window_end)
        ind = get_zero_derivative_indices(sndarray[window_start:window_end+1])
        # print ind
        vals = []
        #
        for i in ind:
            vals.append(sndarray[i+window_start])
        # print vals

        pitchMarksInd = numPitchMarks
        largest = heapq.nlargest(pitchMarksInd, vals)
        if len(largest) < pitchMarksInd:
            pitchMarksInd = len(largest)

        # print res
        # print " pitchMarksInd " + str(pitchMarksInd)
        while pitchMarksInd > 1:
            indVals = numpy.where(vals == largest[pitchMarksInd-1])[0]
            spot_chunk = ind[indVals[0]]
            # print "appended " + str(window_start + spot_chunk)
            window_pitch_obj["pitch_marks"][cnt].append(window_start + spot_chunk)
            pitch_marks.append(window_start + spot_chunk)
            pitchMarksInd = pitchMarksInd - 1
        print "pitch_marks_in_window " + str(window_pitch_obj["pitch_marks"][cnt])
        cnt = cnt + 1
    print "window_pitch_obj " + str(window_pitch_obj)
    print "pitch_marks " + str(pitch_marks)
    return pitch_marks,window_pitch_obj

def get_pitch_marks_unvoiced_chunk():
    return

def get_pitch_marks_region(sndarray, region, chunk_size, type):
    pitch_marks = []
    len = region[1] - region[0] + 1
    steps = int(numpy.ceil(float(len)/float(chunk_size)))
    numPitchMarksPerChunk = 3
    freq_chunk_windows_pitch_marks_obj = {"freq_chunks":[],"windows":[],"pitch_marks":[]}
    f0 = voi.get_freq_array(sndarray,44100,chunk_size)
    print "number of freq chunks " + str(steps)
    print "freq is " + str(f0)
    for i in range(0,steps):
        chunk_start = i * chunk_size + region[0]
        if chunk_start >= region[1]:
            chunk_start = region[1]
        chunk_end = chunk_start + chunk_size - 1
        if chunk_end >= region[1]:
            chunk_end = region[1]
        if chunk_start == chunk_end:
            break
        spot = chunk_start/chunk_size
        # print " len of snd chunk " + str(numpy.size(numpy.asarray(sndarray[chunk_start:chunk_end+1]).tolist()))
        # print " snd chunk start end" + str(sndarray[chunk_start:chunk_end+1])
        print " chunk_start " + str(chunk_start)
        print " chunk_end " + str(chunk_end)
        # f0 = voi.get_freq_array(numpy.asarray(sndarray[chunk_start:chunk_end+1]).tolist(),44100,chunk_size)

        freq = f0[spot]
        # freq = get_freq_chunk(sndarray,chunk_start,chunk_size)

        print "freq chunk number " + str(i) + " range is " + str([chunk_start,chunk_end])
        if type == "voiced":
            freq_chunk_windows_pitch_marks_obj["freq_chunks"].append([chunk_start,chunk_end])
            pitch_marks_chunk,window_pitch_obj = get_pitch_marks_freq_chunk(sndarray,freq,chunk_start, chunk_end, numPitchMarksPerChunk)
            freq_chunk_windows_pitch_marks_obj["windows"].append(window_pitch_obj["windows"])
            freq_chunk_windows_pitch_marks_obj["pitch_marks"].append(window_pitch_obj["pitch_marks"])
        else:
            pitch_marks_chunk = get_pitch_marks_unvoiced_chunk(sndarray,freq,chunk_start, chunk_size, numPitchMarksPerChunk)
        for i in pitch_marks_chunk:
            pitch_marks.append(i)
    return pitch_marks,freq_chunk_windows_pitch_marks_obj

def get_pitch_marks_regions(sndarray,regions, chunk_size, type):
    pitch_marks = []
    cnt = 0
    print "Number of Regions " + str(len(regions))
    print "regions " + str(regions)
    voiced_region_freq_chunk_windows_pitch_marks_obj = {"voiced_region":[],"freq_chunks":[],"windows":[],"pitch_marks":[]}

    for i in regions:
        print "Region range" + str(i)
        voiced_region_freq_chunk_windows_pitch_marks_obj["voiced_region"].append(i)

        pitch_marks_region,freq_chunk_windows_pitch_marks_obj = get_pitch_marks_region(sndarray, i, chunk_size, type)

        voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"].append(freq_chunk_windows_pitch_marks_obj["freq_chunks"])
        voiced_region_freq_chunk_windows_pitch_marks_obj["windows"].append(freq_chunk_windows_pitch_marks_obj["windows"])
        voiced_region_freq_chunk_windows_pitch_marks_obj["pitch_marks"].append(freq_chunk_windows_pitch_marks_obj["pitch_marks"])
        for j in pitch_marks_region:
            pitch_marks.append(j)
        cnt = cnt + 1
    return pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj

if __name__ == "__main__":
    filename= "C:/Users/rediet/Documents/Vocie-samples/eric.wav"
    filenameTxt = "C:/Users/rediet/Documents/Vocie-samples/kendraVU500Pitch_marks.txt"
    filename500= "C:/Users/rediet/Documents/Vocie-samples/kendra500.wav"
    filenameFM = "C:/Users/rediet/Documents/Vocie-samples/kendraFM.wav"
    filenameVoiced = "C:/Users/rediet/Documents/Vocie-samples/kendraVoiced.wav"
    filenameVibrato = "C:/Users/rediet/Documents/Vocie-samples/kendraVibrato.wav"

    fs, x = wavfile.read(filename)
    y = numpy.arange(0,len(x),1)
    x = voi.get_one_channel_array(x)
    chunk_size = 1024

    # voi.plot(y,x,len(x),"signal amplitude")

    vSig = voi.get_signal_voiced_unvoiced_starting_info(x,44100,chunk_size)
    lengthVoiced = voi.get_signal_voiced_length_info(x,vSig)
    voiced_regions = voi.get_voiced_region_chunks(vSig,lengthVoiced)


    pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj = get_pitch_marks_regions(x,voiced_regions,chunk_size, "voiced")
    print voiced_region_freq_chunk_windows_pitch_marks_obj["pitch_marks"]
    print voiced_region_freq_chunk_windows_pitch_marks_obj["voiced_region"]
    print pitch_marks


    numpy.savetxt(filenameTxt,pitch_marks)

    pitch_marks_y = []
    for i in pitch_marks:
        pitch_marks_y.append(x[i])

    start = 230000
    end = 241000
    diff = end - start
    pitch_marks_new = []
    for j in pitch_marks:
        pitch_marks_new.append(j-start)

    import matplotlib.pyplot as plt
    plt.plot(pitch_marks_new,pitch_marks_y,'o',markersize=10,color='red', label=" pitch markers")
    plt.plot(y[0:diff],x[start:end],'-',color='black')
    plt.xlim(0, len(x[0:diff]))
    plt.legend()
    plt.show()



