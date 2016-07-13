
import numpy
import td_psola as tp
import voiced_unvoiced as voi
import pitch_mark_first_step as pmfs
import pitch_mark_second_stage as pmss
from scipy.io import wavfile
from pyo import *


def voiced_region_to_freq_chunks(region_info,chunk_size):
    len = region_info[1] - region_info[0] + 1
    steps = int(numpy.ceil(len/chunk_size))
    freq_chunks = []
    for i in range(0,steps):
        chunk_start = i * chunk_size + region_info[0]
        if chunk_start >= region_info[1]:
            chunk_start = region_info[1]
        chunk_end = chunk_start + chunk_size - 1
        if chunk_end >= region_info[1]:
            chunk_end = region_info[1]
        freq_chunks.append([chunk_start,chunk_end])

def inflection_two(sndarray,voiced_regions_info, inflection_duration, fs,chunk_size):
    new_sndarray = []

    duration_to_samps = inflection_duration * fs
    no_of_freq_chunks = int(numpy.floor(duration_to_samps/chunk_size))
    # pa_list_devices()
    print pa_count_devices()


    print "duration_to_samps " + str(duration_to_samps)
    print "no_of_freq_chunks " + str(no_of_freq_chunks)
    for i in sndarray:
        new_sndarray.append(i)


    # durations = []
    win_sizes = []
    vr_ends = []
    for i in range(0,len(voiced_regions_info)):
        len_vr = voiced_regions_info[i][1] - voiced_regions_info[i][0] + 1
        vr_start = 0
        vr_end = (no_of_freq_chunks * chunk_size) - 1
        if vr_end >= len_vr:
            vr_end = len_vr - 1
        # duration = float(vr_end-vr_start+1)/float(fs)
        voiced_regions = []
        voiced_regions.append([vr_start,vr_end])
        snd_arr = new_sndarray[voiced_regions_info[i][0]:vr_end+ voiced_regions_info[i][0] + 1]
        winsize = 1024
        if vr_end - vr_start + 1 < 1024:
             winsize = vr_end - vr_start
        win_sizes.append(winsize)
        vr_ends.append(vr_end)
 
        print voiced_regions_info[i]
        print voiced_regions
        pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj = pmfs.get_pitch_marks_regions(snd_arr,voiced_regions,chunk_size, "voiced")
         # print voiced_region_freq_chunk_windows_pitch_marks_obj
        best_voiced_region_freq_chunk_windows_pitch_marks_obj = pmss.optimal_accumulated_log_probability(snd_arr,voiced_region_freq_chunk_windows_pitch_marks_obj)
        best_pitch_marks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"]
        freq_chunks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"]
        new_snd = tp.freq_shift_using_td_psola(snd_arr,chunk_size,1.15,best_pitch_marks_info , freq_chunks_info)
         # print "new_snd " + str(new_snd)

        start = voiced_regions_info[i][0]
        for i in range(0,len(new_snd)):
            new_sndarray[i+start] = new_snd[i]
    return new_sndarray

def inflection(sndarray,voiced_regions_info, inflection_duration, fs,chunk_size):
    new_sndarray = []

    duration_to_samps = inflection_duration * fs
    no_of_freq_chunks = int(numpy.floor(duration_to_samps/chunk_size))
    # pa_list_devices()
    print pa_count_devices()


    print "duration_to_samps " + str(duration_to_samps)
    print "no_of_freq_chunks " + str(no_of_freq_chunks)
    for i in sndarray:
        new_sndarray.append(i)

    # durations = []
    win_sizes = []
    vr_ends = []
    for i in range(0,len(voiced_regions_info)):
        len_vr = voiced_regions_info[i][1] - voiced_regions_info[i][0] + 1
        vr_start = 0
        vr_end = (no_of_freq_chunks * chunk_size) - 1
        if vr_end >= len_vr:
            vr_end = len_vr - 1
        # duration = float(vr_end-vr_start+1)/float(fs)
        voiced_regions = []
        voiced_regions.append([vr_start,vr_end])

        snd_arr = new_sndarray[voiced_regions_info[i][0]:vr_end+ voiced_regions_info[i][0] + 1]
        winsize = 1024
        if vr_end - vr_start + 1 < 1024:
            winsize = vr_end - vr_start
        win_sizes.append(winsize)
        vr_ends.append(vr_end)

        new_snd = []
        for n in range(0,no_of_freq_chunks):
            fr_start = n * chunk_size + voiced_regions_info [i] [0]
            fr_end = (n*chunk_size) + chunk_size + voiced_regions_info [i][0]
            if fr_start >= voiced_regions_info[i][1]:
                break
            if fr_end >= voiced_regions_info[i][1]:
                fr_end = voiced_regions_info [i][1]
            snd_arr_inflect = new_sndarray[fr_start:fr_end]
            b = 1.1
            a = float(1- b)/float(no_of_freq_chunks-3)
            if n > 2:
                pitch_shift = (float(a) *float(n)) + float(b)
            else:
                pitch_shift = 0.85
            voiced_regions = []
            voiced_regions.append([0,chunk_size-1])

            pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj = pmfs.get_pitch_marks_regions(snd_arr_inflect,voiced_regions,chunk_size, "voiced")

            best_voiced_region_freq_chunk_windows_pitch_marks_obj = pmss.optimal_accumulated_log_probability(snd_arr_inflect,voiced_region_freq_chunk_windows_pitch_marks_obj)
            best_pitch_marks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"]
            freq_chunks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"]
            best_pitch_marks = []
            for best_pitch_marks_region in best_pitch_marks_info:
                for best_pitch_marks_freq_chunk in best_pitch_marks_region:
                    for j in best_pitch_marks_freq_chunk:
                        best_pitch_marks.append(j)
            print "freq chunk number " + str(n)
            print "best_pitch_marks "  + str(best_pitch_marks)
            print voiced_region_freq_chunk_windows_pitch_marks_obj
            new_snd_tmp = tp.freq_shift_using_td_psola_helper_new_two(snd_arr_inflect,best_pitch_marks,pitch_shift)
            new_snd = numpy.concatenate([new_snd,new_snd_tmp])
        new_sndTwo= []
        for k in new_snd:
            new_sndTwo.append(numpy.int16(k))

        print "i " + str(i) + " " + str(voiced_regions_info) + " " + str(len(voiced_regions_info))
        print "len new sndtwo " + str(len(new_sndTwo)/chunk_size)
        start = voiced_regions_info[i][0]
        print "m plus start " + str(len(new_sndTwo)+start)
        for m in range(0,len(new_sndTwo)):
            new_sndarray[m+start] = new_sndTwo[m]
    return new_sndarray

if __name__ == "__main__":
    filename= "C:/Users/rediet/Documents/Vocie-samples/salli.wav"
    filenameInflection = "C:/Users/rediet/Documents/Vocie-samples/emmaInflection.wav"
    filenameInflectionAverage = "C:/Users/rediet/Documents/Vocie-samples/kendraInflectionAverage.wav"
    filenameInflectionHappy = "C:/Users/rediet/Documents/Vocie-samples/kendraInflectionHappy.wav"


    fs, x = wavfile.read(filename)
    y = numpy.arange(0,len(x),1)
    x = voi.get_one_channel_array(x)
    chunk_size = 1024

    # voi.plot(y,x,len(x),"signal amplitude")

    vSig = voi.get_signal_voiced_unvoiced_starting_info(x,fs,chunk_size)
    lengthVoiced = voi.get_signal_voiced_length_info(x,vSig)
    # lengthUnvoiced = voi.get_signal_unvoiced_length_info(x,vSig)

    voiced_regions = voi.get_voiced_region_chunks(vSig,lengthVoiced)

    new_snd = inflection_two(x,voiced_regions,0.5,fs,chunk_size)
    new_snd_new = []
    cnt = 0
    for i in new_snd:
        # print str(cnt) + " " + str(len(new_snd))
        new_snd_new.append(numpy.int16(i))
        if cnt == len(new_snd)-1:
            break
        cnt = cnt + 1
    wavfile.write(filenameInflection,fs,numpy.array(new_snd_new))





