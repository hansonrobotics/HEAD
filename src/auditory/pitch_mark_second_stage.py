from scipy.io import wavfile
import numpy
import voiced_unvoiced as voi
import pitch_mark_first_step as pmfs
import math
from pyo import *

def calculate_state_probability(hcanditate, hmax, hmin):
    num = hcanditate - hmin
    if hmax == hmin:
        return hcanditate
    den = hmax - hmin
    quo = float(num)/float(den)
    return quo

def get_state_probabilities(sndarray,pitch_marks_freq_array,wind, cand,hmax, hmin):
    state_prob = []
    state_prob_wind_cand = calculate_state_probability(sndarray[pitch_marks_freq_array[wind][cand]],hmax,hmin)
    for i in pitch_marks_freq_array[wind]:
        state_prob.append(calculate_state_probability(sndarray[i],hmax,hmin))
    summ = numpy.sum(state_prob)
    if summ == 0:
        return 0
    norm = float(state_prob_wind_cand)/float(summ)
    return norm

def calculate_transition_probability(beta,fs,dist):
    fs = float(fs)
    dist = float(dist)
    freq = dist/fs
    if dist == 0:
        return 0
    den = 1 + (beta * numpy.abs(freq-fs/dist))
    num = 1
    quo = num/den
    return quo

def get_transition_probabilities(sndarray,pitch_marks_freq_array,wind,fs, candOne,candTwo, beta):
    trans_prob = []
    dist = numpy.abs(pitch_marks_freq_array[wind+1][candOne] - pitch_marks_freq_array[wind][candTwo])
    trans_prob_wind_cand = calculate_transition_probability(beta,fs,dist)
    for i in pitch_marks_freq_array[wind+1]:
        dist = i - pitch_marks_freq_array[wind][candTwo]
        trans_prob.append(calculate_transition_probability(beta,fs,dist))
    summ = numpy.sum(trans_prob)
    if summ == 0:
        return float(trans_prob_wind_cand)
    norm = float(trans_prob_wind_cand)/float(summ)
    return norm

def optimal_accumulated_log_probability_recur_helper(sndarray,pitch_marks_freq_array,identifier, helper_array, helper_array_two,hmax, hmin,fs):
    retVal = []
    opt = []
    cnt = 0
    pitchMarksCnt = len(pitch_marks_freq_array[0])

    two_len = len(helper_array_two)
    # print "identifier " + str(identifier)
    helper_array_three = []
    for i in pitch_marks_freq_array:
        helper_array_three.append(len(i))
    three_len = len(helper_array_three)
    for i in pitch_marks_freq_array[identifier]:
        candState = get_state_probabilities(sndarray,pitch_marks_freq_array,identifier,cnt,hmax,hmin)
        if identifier == 0:
            # print " in 0 candState is " + str(candState)
            opt.append(candState)
            helper_array[cnt] = candState
            helper_array_two[cnt] = int(0)
        else:
            cntTwo = 0
            beta = 1
            optTwo = []
            for i in pitch_marks_freq_array[identifier-1]:
                cntT2 = numpy.sum(helper_array_three[0:identifier-1])
                # print "helper_array " + str(helper_array)
                # print "helper_array_two " + str(helper_array_two)
                # print "helper_array_identifier " + str(cntTwo + cntT2)
                if numpy.isinf(helper_array[cntTwo + cntT2]) == False:
                    # print "I am going to call a function 0"
                    prev_opt = helper_array[cntTwo + cntT2]
                else:
                    # print "I am going to call a function 1"
                    optimal_accumulated_log_probability_recur_helper(sndarray,pitch_marks_freq_array,identifier-1,helper_array,helper_array_two,hmax,hmin,fs)
                    prev_opt = helper_array[cntTwo + cntT2]
                trans_prob = get_transition_probabilities(sndarray,pitch_marks_freq_array,identifier-1,fs,cnt,cntTwo,beta)
                optTwo.append( prev_opt + trans_prob )
                cntTwo = cntTwo + 1
            maxK = numpy.argmax(optTwo)
            # retVal.append(maxK)
            summ = optTwo[maxK] + candState
            # print "optTwo " + str(optTwo)
            # print "optTwo[maxK] " + str(optTwo[maxK])
            # print "summ " + str(summ)
            cntT = numpy.sum(helper_array_three[0:identifier])
            helper_array[cnt + cntT] = summ
            helper_array_two[cnt + cntT] = int(maxK)
            opt.append(summ)
        cnt = cnt + 1
    maxOpt = numpy.argmax(opt)

    if identifier == len(pitch_marks_freq_array) - 1:
        retVal.append(maxOpt)
        # retVal.append(maxKopt[maxOpt])

        cnt = len(pitch_marks_freq_array)
        cntT = 0
        # print "last opt " + str(opt)
        # print "maxOpt " + str(maxOpt)
        # print "helper arrayTwo " + str(helper_array_two)
        # print "helper arrayThree " + str(helper_array_three)
        for i in range(0,cnt-1):
            cntT = numpy.sum(helper_array_three[three_len-1-i:three_len])
            # print "cntT " + str(cntT)
            # print "two_lenCntT " + str(two_len-cntT + maxOpt)
            maxOpt = int(helper_array_two[two_len-cntT + maxOpt])
            retVal.append(maxOpt)
            # print "maxOptTwo " + str(maxOpt)
        # print "retValOne " + str(retVal)
        # retVal.append(maxOpt)
    x = list(reversed(retVal))
    return x

def optimal_accumulated_log_probability_recur(sndarray,pitch_marks_freq_chunk, hmax, hmin,fs):
    pitchMarksCnt = len(pitch_marks_freq_chunk[0])
    helper_array_len = 0
    for i in range(0,len(pitch_marks_freq_chunk)):
        helper_array_len = helper_array_len + len(pitch_marks_freq_chunk[i])
    helper_array = numpy.empty(helper_array_len,float)
    helper_array_two = numpy.empty(helper_array_len,float)
    helper_array[:] = numpy.inf
    helper_array_two[:] = numpy.inf
    identifier = len(pitch_marks_freq_chunk)-1
    return optimal_accumulated_log_probability_recur_helper(sndarray,pitch_marks_freq_chunk,identifier,helper_array,helper_array_two,hmax,hmin,fs)


def optimal_accumulated_log_probability(sndarray, all_snd_info):
    best_voiced_region_freq_chunk_windows_pitch_marks_obj = {"voiced_region":[],"freq_chunks":[],"windows":[],"pitch_marks":[],"best_pitch_marks":[]}
    cnt = 0
    new_snd_info = all_snd_info
    best_voiced_region_freq_chunk_windows_pitch_marks_obj["voiced_region"] = new_snd_info["voiced_region"]
    best_voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"] = new_snd_info["freq_chunks"]
    best_voiced_region_freq_chunk_windows_pitch_marks_obj["windows"] = new_snd_info["windows"]
    best_voiced_region_freq_chunk_windows_pitch_marks_obj["pitch_marks"] = new_snd_info["pitch_marks"]

    cntVoicedRegions = 0
    tot_time_two = 0.0
    for pitch_marks_voiced_region in all_snd_info["pitch_marks"]:
        best_pitch_marks_freq_chunks = []
        voiced_region_start = best_voiced_region_freq_chunk_windows_pitch_marks_obj["voiced_region"][cntVoicedRegions][0]
        voiced_region_end = best_voiced_region_freq_chunk_windows_pitch_marks_obj["voiced_region"][cntVoicedRegions][1]
        # print voiced_region_start
        # print voiced_region_end
        # print len(sndarray)
        hmaxIndex = numpy.argmax(sndarray[voiced_region_start:voiced_region_end])
        hminIndex = numpy.argmin(sndarray[voiced_region_start:voiced_region_end])
        # print hmaxIndex
        # print hminIndex
        hmax = sndarray[voiced_region_start + hmaxIndex]
        hmin = sndarray[voiced_region_start + hminIndex]
        # print "hmax " + str(hmax)
        # print "hmin " + str(hmin)
        tot_time = 0.0
        for pitch_marks_freq_chunk in pitch_marks_voiced_region:
            # print "pitch_marks_freq_chunk " + str(pitch_marks_freq_chunk)
            best_pitch_marks = []
            if len(pitch_marks_freq_chunk) == 0:
                best_pitch_marks_freq_chunks.append(best_pitch_marks)
            else:
                sTime = time.time()
                x = optimal_accumulated_log_probability_recur(sndarray,pitch_marks_freq_chunk,hmax,hmin,44100)
                eTime = time.time()
                tot_time = tot_time + (eTime-sTime)
                # print "time taken by log prob recur is " + str(eTime-sTime)
                for j in range(0,len(pitch_marks_freq_chunk)):
                    # print " j !!! " + str(j)
                    # print "pitch_marks!!!!"  + str(sndarray[pitch_marks_freq_chunk[j][x[j]]])
                    best_pitch_marks.append(pitch_marks_freq_chunk[j][x[j]])
                # print "optimal pitch marks " + str(best_pitch_marks) + "best pitch marks " + str(pitch_marks_freq_chunk)
                best_pitch_marks_freq_chunks.append(best_pitch_marks)
        # print "time taken by log prob recur all is " + str(tot_time)
        tot_time_two = tot_time_two + tot_time
        # print "length of freq chunks " + str(len(pitch_marks_voiced_region))
        # print best_pitch_marks_freq_chunks
        best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"].append(best_pitch_marks_freq_chunks)
        cntVoicedRegions = cntVoicedRegions + 1
    print "time taken by log prob recur two all is " + str(tot_time_two)
    return best_voiced_region_freq_chunk_windows_pitch_marks_obj

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
    f0 = voi.get_freq_array(x,fs,chunk_size)
    vSig = voi.get_signal_voiced_unvoiced_starting_info(x,f0,44100,chunk_size,)
    lengthVoiced = voi.get_signal_voiced_length_info(x,vSig)
    voiced_regions = voi.get_voiced_region_chunks(vSig,lengthVoiced)
    pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj = pmfs.get_pitch_marks_regions(x, f0, voiced_regions,chunk_size, "voiced")
    import time
    sTime = time.time()
    best_voiced_region_freq_chunk_windows_pitch_marks_obj = optimal_accumulated_log_probability(x,voiced_region_freq_chunk_windows_pitch_marks_obj)
    eTime = time.time()
    print "time taken by pmss is " + str(eTime - sTime)



    # print best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"]
    # print best_voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"]

    # best_pitch_marks = []
    # for best_pitch_marks_voiced_region in best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"]:
    #     for best_pitch_marks_chunk in best_pitch_marks_voiced_region:
    #         for i in best_pitch_marks_chunk:
    #             best_pitch_marks.append(i)
    #
    # best_pitch_marks_y = []
    # for i in best_pitch_marks:
    #     best_pitch_marks_y.append(x[i])
    #
    # start = 0
    # end = 24100
    # diff = end - start
    # best_pitch_marks_new = []
    # for j in best_pitch_marks:
    #     best_pitch_marks_new.append(j-start)
    #
    # import matplotlib.pyplot as plt
    # plt.plot(best_pitch_marks_new,best_pitch_marks_y,'o',markersize=10,color='red', label=" best pitch markers")
    # plt.plot(y[0:diff],x[start:end],'-',color='black')
    # plt.xlim(0, len(x[0:diff]))
    # plt.legend()
    # plt.show()