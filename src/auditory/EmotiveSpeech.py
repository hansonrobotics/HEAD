from pyo import *
import numpy
import td_psola as tp
import voiced_unvoiced as voi
import pitch_mark_first_step as pmfs
import pitch_mark_second_stage as pmss
from scipy.io import wavfile
import Inflection as inflect
import td_psola as psola
from scipy.signal import *
import freq_shift_array as fsa

def emotiveSpeech(filename,typeOfEmotion):
    if typeOfEmotion == "Happy":
        sTimeAll = time.time()
        sep = filename.split("/")
        print sep
        rootName = ""
        for i in range(1,len(sep)):
            rootName = str(rootName) + "/" + str(sep[i])
        print rootName
        name = sep[len(sep)-1].split(".")[0]
        filenameFreqShift = str(rootName) + "/" + str(name) + "FreqShiftHappy.wav"
        filenameInflection = str(rootName) + "/" + str(name) + "InflectionHappy.wav"
        filenameAverage = str(rootName) + "/" + str(name) + "AverageHappy.wav"
        filenameHappy = str(rootName) + "/" + str(name) + "Happy.wav"

        sTime = time.time()
        fs, x = wavfile.read(filename)
        # x = voi.get_one_channel_array(x)
        chunk_size = 1024

        f0 = voi.get_freq_array(x,fs,chunk_size)
        vSig = voi.get_signal_voiced_unvoiced_starting_info(x,f0, fs,chunk_size)
        lengthVoiced = voi.get_signal_voiced_length_info(x,vSig)
        voiced_regions = voi.get_voiced_region_chunks(vSig,lengthVoiced)
        eTime = time.time()
        print "time taken by get freq array for inflection " + str(eTime-sTime)

        sTime = time.time()
        new_snd = inflect.inflection_happy_newest_two(x,voiced_regions,f0,0.53,fs,chunk_size)
        # new_snd_new = []
        # cnt = 0
        # for i in new_snd:
        #     new_snd_new.append(numpy.int16(i))
        #     if cnt == len(new_snd)-1:
        #         break
        #     cnt = cnt + 1
        eTime = time.time()
        print "time taken by inflection happy two " + str(eTime-sTime)

        sTime = time.time()
        wavfile.write(filenameInflection,fs,numpy.array(new_snd))


        fs, xf = wavfile.read(filenameInflection)
        eTime = time.time()
        print "time taken write and read inflection file " + str(eTime-sTime)
        # xf = voi.get_one_channel_array(x)
        chunk_size = 1024


        sTime = time.time()
        f0 = voi.get_freq_array(xf,fs,chunk_size)
        vSig = voi.get_signal_voiced_unvoiced_starting_info(xf,f0, fs,chunk_size)
        lengthVoiced = voi.get_signal_voiced_length_info(xf,vSig)
        voiced_regions = voi.get_voiced_region_chunks(vSig,lengthVoiced)

        pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj = pmfs.get_pitch_marks_regions(xf, f0, voiced_regions,chunk_size, "voiced")
        best_voiced_region_freq_chunk_windows_pitch_marks_obj = pmss.optimal_accumulated_log_probability(xf,voiced_region_freq_chunk_windows_pitch_marks_obj)
        eTime = time.time()

        print "time taken by get freq array, pmfs and pmss " + str(eTime-sTime)
        best_pitch_marks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"]
        freq_chunks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"]

        sTime = time.time()
        # best_pitch_marks = []
        # for best_pitch_marks_region in best_pitch_marks_info:
        #     for best_pitch_marks_freq_chunk in best_pitch_marks_region:
        #         for i in best_pitch_marks_freq_chunk:
        #             best_pitch_marks.append(i)
        # new_x = psola.freq_shift_using_td_psola_helper_new_two(x,best_pitch_marks,1.5)

        freq_shift_arr = fsa.create_constant_freq_shift_array(freq_chunks_info,1.03)
        new_xf = psola.freq_shift_using_td_psola_newest(xf,voiced_regions,best_pitch_marks_info,freq_shift_arr)
        eTime = time.time()
        print "time taken by td-psola newest " + str(eTime-sTime)

        sTime = time.time()
        # new_sndarray = []
        # for i in range(0,len(new_xf)):
        #     new_sndarray.append(numpy.int16(new_xf[i]))
        # #
        # new_sndarray = voi.make_two_channels(new_sndarray)
        # new_xf = voi.make_two_channels(new_xf)
        # voi.write_to_new_file(filenameFreqShift,numpy.asarray(new_xf))
        eTime = time.time()
        print "time taken write freq shifted file " + str(eTime-sTime)

        sTime = time.time()
        nyq = 0.5 * fs
        cutFreq = 1000/nyq
        order = 5
        b, a = butter(order,cutFreq , btype='highpass')
        snd_high_shelf = lfilter(b, a, new_xf)
        eTime = time.time()
        print "time taken by butterworth filter " + str(eTime-sTime)

        sTime = time.time()
        new_sndarray_high_shelf = []
        for i in range(0,len(snd_high_shelf)):
            new_sndarray_high_shelf.append(numpy.int16(snd_high_shelf[i]))

        # new_sndarray_high_shelf = voi.make_two_channels(new_sndarray_high_shelf)
        voi.write_to_new_file(filenameHappy,numpy.asarray(new_sndarray_high_shelf))
        eTime = time.time()
        print "time taken write and read happy file " + str(eTime-sTime)
        eTimeAll = time.time()
        print "time taken by all " + str(eTimeAll-sTimeAll)

    elif typeOfEmotion == "Sad":
        sep = filename.split("/")
        name = sep[len(sep)-1].split(".")[0]
        rootName = ""
        for i in range(1, len(sep)):
            rootName = str(rootName) + "/" + str(sep[i])
        print rootName
        filenameFreqShift = str(rootName) + "/" + str(name) + "FreqShiftSad.wav"
        filenameSad = str(rootName) + "/" + str(name) + "Sad.wav"

        fs, x = wavfile.read(filename)
        x = voi.get_one_channel_array(x)
        chunk_size = 1024

        f0 = voi.get_freq_array(x,fs,chunk_size)
        vSig = voi.get_signal_voiced_unvoiced_starting_info(x,f0, fs,chunk_size)
        lengthVoiced = voi.get_signal_voiced_length_info(x,vSig)

        voiced_regions = voi.get_voiced_region_chunks(vSig,lengthVoiced)

        pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj = pmfs.get_pitch_marks_regions(x, f0, voiced_regions,chunk_size, "voiced")
        best_voiced_region_freq_chunk_windows_pitch_marks_obj = pmss.optimal_accumulated_log_probability(x,voiced_region_freq_chunk_windows_pitch_marks_obj)

        best_pitch_marks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"]
        freq_chunks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"]


        # best_pitch_marks = []
        # for best_pitch_marks_region in best_pitch_marks_info:
        #     for best_pitch_marks_freq_chunk in best_pitch_marks_region:
        #         for i in best_pitch_marks_freq_chunk:
        #             best_pitch_marks.append(i)

        # new_x = psola.freq_shift_using_td_psola_helper_new_two(x,best_pitch_marks,0.95)
        freq_shift_arr = fsa.create_constant_freq_shift_array(freq_chunks_info,0.9)
        new_x = psola.freq_shift_using_td_psola_newest(x,voiced_regions,best_pitch_marks_info,freq_shift_arr)

        # new_sndarray = []
        # for i in range(0,len(new_x)):
        #     new_sndarray.append(numpy.int16(new_x[i]))
        #
        # new_sndarray = voi.make_two_channels(new_sndarray)
        # voi.write_to_new_file(filenameFreqShift,numpy.asarray(new_sndarray))

        nyq = 0.5 * fs
        cutFreq = 8000/nyq
        order = 5
        b, a = butter(order,cutFreq , btype='lowpass')
        snd_low_shelf = lfilter(b, a, new_x)

        new_sndarray_low_shelf = []
        for i in range(0,len(snd_low_shelf)):
            new_sndarray_low_shelf.append(numpy.int16(snd_low_shelf[i]))
        voi.write_to_new_file(filenameSad,numpy.asarray(new_sndarray_low_shelf))
    elif typeOfEmotion == "Afraid":
        sep = filename.split("/")
        name = sep[len(sep)-1].split(".")[0]
        rootName = ""
        for i in range(1, len(sep)):
            rootName = str(rootName) + "/" + str(sep[i])
        filenameInflection = str(rootName) + "/" + str(name) + "InflectionAfraid.wav"
        filenameAfraid = str(rootName) + "/" + str(name) + "Afraid.wav"

        fs, x = wavfile.read(filename)
        x = voi.get_one_channel_array(x)
        chunk_size = 1024

        f0 = voi.get_freq_array(x,fs,chunk_size)
        vSig = voi.get_signal_voiced_unvoiced_starting_info(x,f0, fs,chunk_size)
        lengthVoiced = voi.get_signal_voiced_length_info(x,vSig)

        voiced_regions = voi.get_voiced_region_chunks(vSig,lengthVoiced)

        new_snd = inflect.inflection_fear(x,voiced_regions,0.53,fs,chunk_size)
        new_snd_new = []
        cnt = 0
        for i in new_snd:
            new_snd_new.append(numpy.int16(i))
            if cnt == len(new_snd)-1:
                break
            cnt = cnt + 1
        wavfile.write(filenameInflection,fs,numpy.array(new_snd_new))


        fs, xf = wavfile.read(filenameInflection)
        # xf = voi.get_one_channel_array(x)
        chunk_size = 1024

        f0 = voi.get_freq_array(xf,fs,chunk_size)
        vSig = voi.get_signal_voiced_unvoiced_starting_info(xf,f0, fs,chunk_size)
        lengthVoiced = voi.get_signal_voiced_length_info(xf,vSig)
        voiced_regions = voi.get_voiced_region_chunks(vSig,lengthVoiced)

        pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj = pmfs.get_pitch_marks_regions(xf, f0, voiced_regions,chunk_size, "voiced")
        best_voiced_region_freq_chunk_windows_pitch_marks_obj = pmss.optimal_accumulated_log_probability(xf,voiced_region_freq_chunk_windows_pitch_marks_obj)

        best_pitch_marks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"]
        freq_chunks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"]


        # best_pitch_marks = []
        # for best_pitch_marks_region in best_pitch_marks_info:
        #     for best_pitch_marks_freq_chunk in best_pitch_marks_region:
        #         for i in best_pitch_marks_freq_chunk:
        #             best_pitch_marks.append(i)

        # new_x = psola.freq_shift_using_td_psola_helper_new_two(x,best_pitch_marks,1.03)
        freq_shift_arr = fsa.create_vibrato_freq_shift_array(freq_chunks_info,chunk_size,shift_amt=0.07)
        new_xf = psola.freq_shift_using_td_psola_newest(xf,voiced_regions,best_pitch_marks_info,freq_shift_arr)

        new_sndarray = []
        for i in range(0,len(new_xf)):
            new_sndarray.append(numpy.int16(new_xf[i]))

        new_sndarray = voi.make_two_channels(new_sndarray)
        voi.write_to_new_file(filenameAfraid,numpy.asarray(new_sndarray))
    else:
        fs, x = wavfile.read(filename)
        x = voi.get_one_channel_array(x)
        voi.write_to_new_file(filename,numpy.asarray(x))

if __name__ == "__main__":
    filename= "/home/naty/Desktop/Vocie-samples/jennifer.wav"
    sTime = time.time()
    emotiveSpeech(filename,"Happy")
    eTime = time.time()
    print "time taken by emotiveSpeech " + str(eTime-sTime)







