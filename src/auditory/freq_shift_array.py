import numpy

def create_vibrato_freq_shift_array(freq_chunks,chunk_size):
    freq_shift_arr = []
    cnt = 0
    frequency = 10.0/(44100.0/float(chunk_size))
    fctr = frequency * numpy.pi * 2
    for freq_region in freq_chunks:
        freq_shift_arr.append([])
        cntTwo = 0
        for freq_chunk in freq_region:
            freqShiftFactor = numpy.sin(fctr*cntTwo) * 0.15
            freq_shift_arr[cnt].append(1+freqShiftFactor)
            cntTwo = cntTwo + 1
        cnt = cnt + 1

    return freq_shift_arr

def create_constant_freq_shift_array(freq_chunks,freqShiftFactor):
    freq_shift_arr = []
    cnt = 0
    for freq_region in freq_chunks:
        freq_shift_arr.append([])
        for freq_chunk in freq_region:
            freq_shift_arr[cnt].append(freqShiftFactor)
        cnt = cnt + 1

    return freq_shift_arr