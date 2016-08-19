
import numpy
import td_psola as tp
import voiced_unvoiced as voi
import pitch_mark_first_step as pmfs
import pitch_mark_second_stage as pmss
from scipy.io import wavfile
from pyo import *
import td_psola as psola
import freq_shift_array as fsa

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

#inflects the pitch with a constant pitch shift - implementation works but cann't be used for happy inflection
def inflection_two(sndarray,voiced_regions_info, inflection_duration, fs,chunk_size):
    new_sndarray = []

    duration_to_samps = inflection_duration * fs
    no_of_freq_chunks = int(numpy.floor(duration_to_samps/chunk_size))

    # print "duration_to_samps " + str(duration_to_samps)
    # print "no_of_freq_chunks " + str(no_of_freq_chunks)
    for i in sndarray:
        new_sndarray.append(i)

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

        f0 = voi.get_freq_array(snd_arr,fs,chunk_size)
        pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj = pmfs.get_pitch_marks_regions(snd_arr,f0,voiced_regions,chunk_size, "voiced")
        best_voiced_region_freq_chunk_windows_pitch_marks_obj = pmss.optimal_accumulated_log_probability(snd_arr,voiced_region_freq_chunk_windows_pitch_marks_obj)

        best_pitch_marks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"]
        freq_chunks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"]

        # new_snd = tp.freq_shift_using_td_psola(snd_arr,chunk_size,1.15,best_pitch_marks_info , freq_chunks_info)
        best_pitch_marks = []
        for best_pitch_marks_region in best_pitch_marks_info:
            for best_pitch_marks_freq_chunk in best_pitch_marks_region:
                for j in best_pitch_marks_freq_chunk:
                    best_pitch_marks.append(j)

        # new_snd = tp.freq_shift_using_td_psola_helper_new_two(snd_arr,best_pitch_marks,1.12)
        freq_shift_arr = psola.create_constant_freq_shift_array(freq_chunks_info,1.12)
        new_snd = psola.freq_shift_using_td_psola_newest(snd_arr,voiced_regions,best_pitch_marks_info,freq_shift_arr)

        start = voiced_regions_info[i][0]
        for i in range(0,len(new_snd)):
            new_sndarray[i+start] = new_snd[i]
    return new_sndarray

#inflects the pitch according to DAVID - does not use an inflection array
def inflection_happy(sndarray,voiced_regions_info, inflection_duration, fs,chunk_size):
    new_sndarray = []

    duration_to_samps = float(inflection_duration) * float(fs)
    no_of_freq_chunks = int(numpy.floor(duration_to_samps/chunk_size))

    # print "duration_to_samps " + str(duration_to_samps)
    # print "no_of_freq_chunks " + str(no_of_freq_chunks)
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

        # snd_arr = new_sndarray[voiced_regions_info[i][0]:vr_end+ voiced_regions_info[i][0] + 1]
        winsize = 1024
        if vr_end - vr_start + 1 < 1024:
            winsize = vr_end - vr_start
        win_sizes.append(winsize)
        vr_ends.append(vr_end)

        no_l_h_l = int((3.0/20.0) * no_of_freq_chunks)
        no_h = no_of_freq_chunks - (2 * no_l_h_l)
        total_chunks = no_h + (2*no_l_h_l)
        f_start = voiced_regions_info [i] [0]
        f_end = f_start + (total_chunks*chunk_size)
        if f_end > voiced_regions_info[i][1]:
            f_end = voiced_regions_info[i][1]
        snd_arr_f = new_sndarray[f_start:f_end]
        f = voi.get_freq_array(snd_arr_f,fs,chunk_size)
        # print f
        new_snd = []
        b = 0.9
        a = float(1.1- b)/float(no_l_h_l-1)
        for n in range(0,no_l_h_l):
            fr_start = n * chunk_size + voiced_regions_info [i] [0]
            fr_end = fr_start + chunk_size
            if fr_start >= voiced_regions_info[i][1]:
                break
            if fr_end >= voiced_regions_info[i][1]:
                fr_end = voiced_regions_info [i][1]
            snd_arr_inflect = new_sndarray[fr_start:fr_end]
            pitch_shift = (float(a)*float(n)) + float(b)

            voiced_regions = []
            voiced_regions.append([0,chunk_size-1])
            f0= [f[n]]
            # print "f0 " + str(f0)
            pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj = pmfs.get_pitch_marks_regions(snd_arr_inflect,f0,voiced_regions,chunk_size, "voiced")
            best_voiced_region_freq_chunk_windows_pitch_marks_obj = pmss.optimal_accumulated_log_probability(snd_arr_inflect,voiced_region_freq_chunk_windows_pitch_marks_obj)
            best_pitch_marks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"]
            freq_chunks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"]

            best_pitch_marks = []
            for best_pitch_marks_region in best_pitch_marks_info:
                for best_pitch_marks_freq_chunk in best_pitch_marks_region:
                    for j in best_pitch_marks_freq_chunk:
                        best_pitch_marks.append(j)

            # new_snd_tmp = tp.freq_shift_using_td_psola_helper_new_two(snd_arr_inflect,best_pitch_marks,pitch_shift)
            freq_shift_arr = fsa.create_constant_freq_shift_array(freq_chunks_info,pitch_shift)
            new_snd_tmp = psola.freq_shift_using_td_psola_newest(snd_arr_inflect,voiced_regions,best_pitch_marks_info,freq_shift_arr)

            new_snd = numpy.concatenate([new_snd,new_snd_tmp])

        fr_start = no_l_h_l * chunk_size + voiced_regions_info [i] [0]
        fr_end = fr_start + (chunk_size*no_h)
        if fr_end >= voiced_regions_info[i][1]:
            fr_end = voiced_regions_info [i][1]
        if fr_start < voiced_regions_info[i][1]:
            snd_arr_inflect = new_sndarray[fr_start:fr_end]
            pitch_shift = 1.12
            voiced_regions = []
            voiced_regions.append([0,fr_end-fr_start-1])
            # print len(snd_arr_inflect)
            f0 = f[no_l_h_l:no_l_h_l+no_h]
            pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj = pmfs.get_pitch_marks_regions(snd_arr_inflect,f0,voiced_regions,chunk_size, "voiced")
            best_voiced_region_freq_chunk_windows_pitch_marks_obj = pmss.optimal_accumulated_log_probability(snd_arr_inflect,voiced_region_freq_chunk_windows_pitch_marks_obj)
            best_pitch_marks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"]
            freq_chunks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"]

            best_pitch_marks = []
            for best_pitch_marks_region in best_pitch_marks_info:
                for best_pitch_marks_freq_chunk in best_pitch_marks_region:
                    for j in best_pitch_marks_freq_chunk:
                        best_pitch_marks.append(j)

            # new_snd_tmp = tp.freq_shift_using_td_psola_helper_new_two(snd_arr_inflect,best_pitch_marks,pitch_shift)
            freq_shift_arr = fsa.create_constant_freq_shift_array(freq_chunks_info,pitch_shift)
            new_snd_tmp = psola.freq_shift_using_td_psola_newest(snd_arr_inflect,voiced_regions,best_pitch_marks_info,freq_shift_arr)
            new_snd = numpy.concatenate([new_snd,new_snd_tmp])

        if fr_end < voiced_regions_info [i][1]:
            b = 1.12
            a = float(1- b)/float(no_l_h_l-1)
            for n in range(0,no_l_h_l):
                fr_start = fr_end
                fr_end = fr_start + chunk_size
                if fr_start >= voiced_regions_info[i][1]:
                    break
                if fr_end >= voiced_regions_info[i][1]:
                    fr_end = voiced_regions_info [i][1]
                snd_arr_inflect = new_sndarray[fr_start:fr_end]
                pitch_shift = (float(a)*float(n)) + float(b)

                voiced_regions = []
                voiced_regions.append([0,chunk_size-1])
                spot = no_l_h_l + no_h + n
                f0 = [f[spot]]
                pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj = pmfs.get_pitch_marks_regions(snd_arr_inflect,f0,voiced_regions,chunk_size, "voiced")
                best_voiced_region_freq_chunk_windows_pitch_marks_obj = pmss.optimal_accumulated_log_probability(snd_arr_inflect,voiced_region_freq_chunk_windows_pitch_marks_obj)
                best_pitch_marks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"]
                freq_chunks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"]

                best_pitch_marks = []
                for best_pitch_marks_region in best_pitch_marks_info:
                    for best_pitch_marks_freq_chunk in best_pitch_marks_region:
                        for j in best_pitch_marks_freq_chunk:
                            best_pitch_marks.append(j)

                # new_snd_tmp = tp.freq_shift_using_td_psola_helper_new_two(snd_arr_inflect,best_pitch_marks,pitch_shift)
                freq_shift_arr = fsa.create_constant_freq_shift_array(freq_chunks_info,pitch_shift)
                new_snd_tmp = psola.freq_shift_using_td_psola_newest(snd_arr_inflect,voiced_regions,best_pitch_marks_info,freq_shift_arr)
                new_snd = numpy.concatenate([new_snd,new_snd_tmp])

        new_sndTwo= []
        for k in new_snd:
            new_sndTwo.append(numpy.int16(k))

        # print "i " + str(i) + " " + str(voiced_regions_info) + " " + str(len(voiced_regions_info))
        # print "len new sndtwo " + str(len(new_sndTwo)/chunk_size)
        start = voiced_regions_info[i][0]
        # print "m plus start " + str(len(new_sndTwo)+start)
        for m in range(0,len(new_sndTwo)):
            new_sndarray[m+start] = new_sndTwo[m]
    return new_sndarray

#inflects the pitch according to DAVID- uses an inflection array to do so - calls pmfs and pmss in a for loop-calls get_freq_array in a for loop
def inflection_happy_two(sndarray,voiced_regions_info,inflection_duration, fs,chunk_size):
    new_sndarray = []

    duration_to_samps = float(inflection_duration) * float(fs)
    no_of_freq_chunks = int(numpy.floor(duration_to_samps/chunk_size))

    # print "duration_to_samps " + str(duration_to_samps)
    # print "no_of_freq_chunks " + str(no_of_freq_chunks)
    for i in sndarray:
        new_sndarray.append(i)
    sTimeAll = time.time()
    time_taken = 0
    for i in range(0,len(voiced_regions_info)):
        len_vr = voiced_regions_info[i][1] - voiced_regions_info[i][0] + 1
        vr_start = 0
        vr_end = (no_of_freq_chunks * chunk_size) - 1
        if vr_end >= len_vr:
            vr_end = len_vr - 1
        # duration = float(vr_end-vr_start+1)/float(fs)
        voiced_regions = []
        voiced_regions.append([vr_start,vr_end])
        sTime = time.time()
        snd_arr = new_sndarray[voiced_regions_info[i][0]:vr_end+ voiced_regions_info[i][0] + 1]

        f0 = voi.get_freq_array(snd_arr,fs,chunk_size)
        # f_start = voiced_regions_info[i][0]/chunk_size
        # f_end = (vr_end + voiced_regions_info[i][0])/chunk_size
        # f_arr = f[f_start:f_end+1]
        # print "f0 " + str(f0)
        # print "f_arr!! " + str(f_arr)

        pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj = pmfs.get_pitch_marks_regions(snd_arr,f0,voiced_regions,chunk_size, "voiced")
        best_voiced_region_freq_chunk_windows_pitch_marks_obj = pmss.optimal_accumulated_log_probability(snd_arr,voiced_region_freq_chunk_windows_pitch_marks_obj)
        # eTime = time.time()
        # print "time taken by pmfs and pmss is " + str(eTime - sTime)

        best_pitch_marks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"]
        freq_chunks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"]

        # sTime = time.time()
        freq_shift_arr = fsa.create_happiness_inflection_array(freq_chunks_info)
        # print freq_shift_arr
        # print len(freq_shift_arr[0])
        # print len(snd_arr)
        # print voiced_regions
        # print best_pitch_marks_info
        # print len(best_pitch_marks_info[0])
        new_snd = psola.freq_shift_using_td_psola_newest(snd_arr,voiced_regions,best_pitch_marks_info,freq_shift_arr)


        new_sndTwo= []
        for k in new_snd:
            new_sndTwo.append(numpy.int16(k))


        # print "i " + str(i) + " " + str(voiced_regions_info) + " " + str(len(voiced_regions_info))
        # print "len new sndtwo " + str(len(new_sndTwo)/chunk_size)
        start = voiced_regions_info[i][0]
        # print "m plus start " + str(len(new_sndTwo)+start)
        for m in range(0,len(new_sndTwo)):
            new_sndarray[m+start] = new_sndTwo[m]
        eTime = time.time()
        time_taken = time_taken + (eTime - sTime)
        print "time taken by psola in inflection happy two is " + str(eTime - sTime)
    eTime = time.time()
    print "time taken by inflection happy two for loop is " + str(eTime-sTimeAll)
    print "time taken by inflection happy two for loop is " + str(time_taken)
    return new_sndarray

#inflects the pitch according to DAVID- uses an inflection array to do so-calls pmfs and pmss once
def inflection_happy_newest(sndarray,voiced_regions_info, f0, inflection_duration, fs,chunk_size):
    new_sndarray = []

    duration_to_samps = float(inflection_duration) * float(fs)
    no_of_freq_chunks = int(numpy.floor(duration_to_samps/chunk_size))

    # print "duration_to_samps " + str(duration_to_samps)
    # print "no_of_freq_chunks " + str(no_of_freq_chunks)
    for i in sndarray:
        new_sndarray.append(i)
    sTime = time.time()
    pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj = pmfs.get_pitch_marks_regions(sndarray,f0,voiced_regions_info,chunk_size, "voiced")
    eTime = time.time()
    print "time taken by pmfs  is " + str(eTime - sTime)
    sTime = time.time()
    best_voiced_region_freq_chunk_windows_pitch_marks_obj = pmss.optimal_accumulated_log_probability(sndarray,voiced_region_freq_chunk_windows_pitch_marks_obj)
    eTime = time.time()
    print "time taken by pmss is " + str(eTime - sTime)
    sTimeAll = time.time()
    for i in range(0,len(voiced_regions_info)):
        len_vr = voiced_regions_info[i][1] - voiced_regions_info[i][0] + 1
        vr_start = 0
        vr_end = (no_of_freq_chunks * chunk_size) - 1
        if vr_end >= len_vr:
            vr_end = len_vr - 1
            no_of_freq_chunks =  int(numpy.floor(vr_end-vr_start/chunk_size))
        # duration = float(vr_end-vr_start+1)/float(fs)
        voiced_regions = []
        voiced_regions.append([vr_start,vr_end])

        snd_arr = new_sndarray[voiced_regions_info[i][0]:vr_end+ voiced_regions_info[i][0] + 1]
        # print best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"][i][0:no_of_freq_chunks]

        sTime = time.time()
        best_pitch_marks_info = []
        cnt = 0
        for p_mark_freq_chunk in best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"][i][0:no_of_freq_chunks]:
            best_pitch_marks_info.append([])
            for p_mark in p_mark_freq_chunk:
                best_pitch_marks_info[cnt].append(p_mark - voiced_regions_info[i][0])
            cnt = cnt + 1
        freq_chunks_info = []
        cnt = 0
        for freq_chunk in best_voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"][i][0:no_of_freq_chunks]:
            freq_chunks_info.append([])
            for freq_chunk_s_e in freq_chunk:
                freq_chunks_info[cnt].append(freq_chunk_s_e - voiced_regions_info[i][0])
            cnt = cnt + 1

        # eTime = time.time()
        # print "time taken by the two for loops is " + str(eTime-sTime)


        # best_pitch_marks_sub = numpy.abs(numpy.asarray(best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"][i][0:no_of_freq_chunks])-voiced_regions_info[i][0])
        best_pitch_marks_info = [best_pitch_marks_info]
        freq_chunks_info = [freq_chunks_info]

        # print "best_pitch_marks_info " + str(best_pitch_marks_info)
        # print "freq_chunks_info" + str(freq_chunks_info)
        # print voiced_regions_info
        # if i < 12:
        #    continue
        # else:
        #     break
        # sTime = time.time()
        freq_shift_arr = fsa.create_happiness_inflection_array(freq_chunks_info)
        # print freq_shift_arr
        # print len(freq_shift_arr[0])
        # print len(snd_arr)
        # print voiced_regions
        # print best_pitch_marks_info
        # print len(best_pitch_marks_info[0])
        new_snd = psola.freq_shift_using_td_psola_newest(snd_arr,voiced_regions,best_pitch_marks_info,freq_shift_arr)
        eTime = time.time()
        print "time taken by td psola and two for loops is " + str(eTime-sTime)

        new_sndTwo= []
        for k in new_snd:
            new_sndTwo.append(numpy.int16(k))

        # print "i " + str(i) + " " + str(voiced_regions_info) + " " + str(len(voiced_regions_info))
        # print "len new sndtwo " + str(len(new_sndTwo)/chunk_size)
        start = voiced_regions_info[i][0]
        # print "m plus start " + str(len(new_sndTwo)+start)
        for m in range(0,len(new_sndTwo)):
            new_sndarray[m+start] = new_sndTwo[m]
    eTime = time.time()
    print "time taken by for loop is " + str(eTime - sTimeAll)
    return new_sndarray

#inflects the pitch according to DAVID- uses an inflection array to do so - calls pmfs and pmss in a for loop-doesn't call get_freq_array in a for loop - fastest
def inflection_happy_newest_two(sndarray,voiced_regions_info,f0, inflection_duration, fs,chunk_size):
    new_sndarray = []

    duration_to_samps = float(inflection_duration) * float(fs)
    no_of_freq_chunks = int(numpy.floor(duration_to_samps/chunk_size))

    # print "duration_to_samps " + str(duration_to_samps)
    # print "no_of_freq_chunks " + str(no_of_freq_chunks)
    for i in sndarray:
        new_sndarray.append(i)
    sTimeAll = time.time()
    time_taken = 0
    for i in range(0,len(voiced_regions_info)):
        len_vr = voiced_regions_info[i][1] - voiced_regions_info[i][0] + 1
        vr_start = 0
        vr_end = (no_of_freq_chunks * chunk_size) - 1
        if vr_end >= len_vr:
            vr_end = len_vr - 1
        # duration = float(vr_end-vr_start+1)/float(fs)
        voiced_regions = []
        voiced_regions.append([vr_start,vr_end])
        sTime = time.time()
        snd_arr = new_sndarray[voiced_regions_info[i][0]:vr_end+ voiced_regions_info[i][0] + 1]
        # f0 = voi.get_freq_array(snd_arr,fs,chunk_size)
        f_start = voiced_regions_info[i][0]/chunk_size
        f_end = (vr_end + voiced_regions_info[i][0])/chunk_size
        f_arr = f0[f_start:f_end+1]


        pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj = pmfs.get_pitch_marks_regions(snd_arr,f_arr,voiced_regions,chunk_size, "voiced")
        best_voiced_region_freq_chunk_windows_pitch_marks_obj = pmss.optimal_accumulated_log_probability(snd_arr,voiced_region_freq_chunk_windows_pitch_marks_obj)
        # eTime = time.time()
        # print "time taken by pmfs and pmss is " + str(eTime - sTime)

        best_pitch_marks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"]
        freq_chunks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"]

        # sTime = time.time()
        freq_shift_arr = fsa.create_happiness_inflection_array(freq_chunks_info)
        # print freq_shift_arr
        # print len(freq_shift_arr[0])
        # print len(snd_arr)
        # print voiced_regions
        # print best_pitch_marks_info
        # print len(best_pitch_marks_info[0])
        new_snd = psola.freq_shift_using_td_psola_newest(snd_arr,voiced_regions,best_pitch_marks_info,freq_shift_arr)


        new_sndTwo= []
        for k in new_snd:
            new_sndTwo.append(numpy.int16(k))


        # print "i " + str(i) + " " + str(voiced_regions_info) + " " + str(len(voiced_regions_info))
        # print "len new sndtwo " + str(len(new_sndTwo)/chunk_size)
        start = voiced_regions_info[i][0]
        # print "m plus start " + str(len(new_sndTwo)+start)
        for m in range(0,len(new_sndTwo)):
            new_sndarray[m+start] = new_sndTwo[m]
        eTime = time.time()
        time_taken = time_taken + (eTime - sTime)
        # print "time taken by psola in inflection happy two is " + str(eTime - sTime)
    eTime = time.time()
    # print "time taken by inflection happy two for loop is " + str(eTime-sTimeAll)
    return new_sndarray

#inflects the pitch according to DAVID- uses an inflection array to do so
def inflection_fear(sndarray,voiced_regions_info, inflection_duration, fs,chunk_size):
    new_sndarray = []

    duration_to_samps = float(inflection_duration) * float(fs)
    no_of_freq_chunks = int(numpy.floor(duration_to_samps/chunk_size))

    # print "duration_to_samps " + str(duration_to_samps)
    # print "no_of_freq_chunks " + str(no_of_freq_chunks)
    for i in sndarray:
        new_sndarray.append(i)

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

        f0 = voi.get_freq_array(snd_arr,fs,chunk_size)

        pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj = pmfs.get_pitch_marks_regions(snd_arr,f0,voiced_regions,chunk_size, "voiced")
        best_voiced_region_freq_chunk_windows_pitch_marks_obj = pmss.optimal_accumulated_log_probability(snd_arr,voiced_region_freq_chunk_windows_pitch_marks_obj)
        best_pitch_marks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"]
        freq_chunks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"]

        freq_shift_arr = fsa.create_fear_inflection_array(freq_chunks_info, chunk_size)
        new_snd = psola.freq_shift_using_td_psola_newest(snd_arr,voiced_regions,best_pitch_marks_info,freq_shift_arr)

        new_sndTwo= []
        for k in new_snd:
            new_sndTwo.append(numpy.int16(k))

        # print "i " + str(i) + " " + str(voiced_regions_info) + " " + str(len(voiced_regions_info))
        # print "len new sndtwo " + str(len(new_sndTwo)/chunk_size)
        start = voiced_regions_info[i][0]
        # print "m plus start " + str(len(new_sndTwo)+start)
        for m in range(0,len(new_sndTwo)):
            new_sndarray[m+start] = new_sndTwo[m]
    return new_sndarray

if __name__ == "__main__":
    filename= "Voice-samples/salli.wav"
    filenameInflection = "Voice-samples/emmaInflection.wav"
    filenameInflectionAverage = "Voice-samples/kendraInflectionAverage.wav"
    filenameInflectionHappy = "Voice-samples/kendraInflectionHappy.wav"

    sTime = time.time()
    fs, x = wavfile.read(filename)
    x = voi.get_one_channel_array(x)
    chunk_size = 1024

    f0 = voi.get_freq_array(x,fs,chunk_size)
    vSig = voi.get_signal_voiced_unvoiced_starting_info(x,f0, fs,chunk_size)
    lengthVoiced = voi.get_signal_voiced_length_info(x,vSig)
    voiced_regions = voi.get_voiced_region_chunks(vSig,lengthVoiced)
    eTime = time.time()
    print "time taken by get freq array for inflection " + str(eTime-sTime)

    sTime = time.time()
    new_snd = inflection_happy_newest_two(x,voiced_regions,f0,0.53,fs,chunk_size)
    # new_snd_new = []
    # cnt = 0
    # for i in new_snd:
    #     new_snd_new.append(numpy.int16(i))
    #     if cnt == len(new_snd)-1:
    #         break
    #     cnt = cnt + 1
    # eTime = time.time()
    print "time taken by inflection happy two " + str(eTime-sTime)
    voi.write_to_new_file(filenameInflection,numpy.asarray(new_snd))






