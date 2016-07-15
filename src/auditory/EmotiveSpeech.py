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
        sep = filename.split("/")
        name = sep[len(sep)-1].split(".")[0]
        filenameFreqShift = "C:/Users/rediet/Documents/Vocie-samples/" + str(name) + "FreqShiftHappy.wav"
        filenameInflection = "C:/Users/rediet/Documents/Vocie-samples/" + str(name) + "InflectionHappy.wav"
        filenameAverage = "C:/Users/rediet/Documents/Vocie-samples/" + str(name) + "AverageHappy.wav"
        filenameHappy = "C:/Users/rediet/Documents/Vocie-samples/" + str(name) + "Happy.wav"

        fs, x = wavfile.read(filename)
        x = voi.get_one_channel_array(x)
        chunk_size = 1024

        f0 = voi.get_freq_array(x,fs,chunk_size)
        vSig = voi.get_signal_voiced_unvoiced_starting_info(x,f0, fs,chunk_size)
        lengthVoiced = voi.get_signal_voiced_length_info(x,vSig)

        voiced_regions = voi.get_voiced_region_chunks(vSig,lengthVoiced)

        new_snd = inflect.inflection_happy(x,voiced_regions,0.53,fs,chunk_size)
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


        best_pitch_marks = []
        for best_pitch_marks_region in best_pitch_marks_info:
            for best_pitch_marks_freq_chunk in best_pitch_marks_region:
                for i in best_pitch_marks_freq_chunk:
                    best_pitch_marks.append(i)
        # new_x = psola.freq_shift_using_td_psola_helper_new_two(x,best_pitch_marks,1.03)
        freq_shift_arr = fsa.create_constant_freq_shift_array(freq_chunks_info,1.03)
        new_xf = psola.freq_shift_using_td_psola_newest(xf,voiced_regions,best_pitch_marks_info,freq_shift_arr)

        new_sndarray = []
        for i in range(0,len(new_xf)):
            new_sndarray.append(numpy.int16(new_xf[i]))

        new_sndarray = voi.make_two_channels(new_sndarray)
        voi.write_to_new_file(filenameFreqShift,numpy.asarray(new_sndarray))

        nyq = 0.5 * fs
        cutFreq = 8000/nyq
        order = 5
        b, a = butter(order,cutFreq , btype='highpass')
        snd_high_shelf = lfilter(b, a, new_sndarray)
        new_sndarray_high_shelf = []
        for i in range(0,len(snd_high_shelf)):
            new_sndarray_high_shelf.append(numpy.int16(snd_high_shelf[i]))
        voi.write_to_new_file(filenameHappy,numpy.asarray(new_sndarray_high_shelf))

    if typeOfEmotion == "Sad":
        sep = filename.split("/")
        name = sep[len(sep)-1].split(".")[0]
        filenameFreqShift = "C:/Users/rediet/Documents/Vocie-samples/" + str(name) + "FreqShiftSad.wav"
        filenameSad = "C:/Users/rediet/Documents/Vocie-samples/" + str(name) + "Sad.wav"

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


        best_pitch_marks = []
        for best_pitch_marks_region in best_pitch_marks_info:
            for best_pitch_marks_freq_chunk in best_pitch_marks_region:
                for i in best_pitch_marks_freq_chunk:
                    best_pitch_marks.append(i)

        # new_x = psola.freq_shift_using_td_psola_helper_new_two(x,best_pitch_marks,0.95)
        freq_shift_arr = fsa.create_constant_freq_shift_array(freq_chunks_info,0.95)
        new_x = psola.freq_shift_using_td_psola_newest(x,voiced_regions,best_pitch_marks_info,freq_shift_arr)

        new_sndarray = []
        for i in range(0,len(new_x)):
            new_sndarray.append(numpy.int16(new_x[i]))

        new_sndarray = voi.make_two_channels(new_sndarray)
        voi.write_to_new_file(filenameFreqShift,numpy.asarray(new_sndarray))

        nyq = 0.5 * fs
        cutFreq = 8000/nyq
        order = 5
        b, a = butter(order,cutFreq , btype='lowpass')
        snd_low_shelf = lfilter(b, a, new_sndarray)

        new_sndarray_low_shelf = []
        for i in range(0,len(snd_low_shelf)):
            new_sndarray_low_shelf.append(numpy.int16(snd_low_shelf[i]))
        voi.write_to_new_file(filenameSad,numpy.asarray(new_sndarray_low_shelf))


if __name__ == "__main__":
    filename= "C:/Users/rediet/Documents/Vocie-samples/eric.wav"
    sTime = time.time()
    emotiveSpeech(filename,"Happy")
    eTime = time.time()
    print "time taken by emotiveSpeech " + str(eTime-sTime)







