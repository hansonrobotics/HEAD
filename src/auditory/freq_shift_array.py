import numpy

def create_vibrato_freq_shift_array(freq_chunks,chunk_size,shift_amt):
    freq_shift_arr = []
    cnt = 0
    frequency = 10.0/(44100.0/float(chunk_size))
    fctr = frequency * numpy.pi * 2
    for freq_region in freq_chunks:
        freq_shift_arr.append([])
        cntTwo = 0
        for freq_chunk in freq_region:
            freqShiftFactor = numpy.sin(fctr*cntTwo) * shift_amt
            freq_shift_arr[cnt].append(1+freqShiftFactor)
            cntTwo = cntTwo + 1
        cnt = cnt + 1

    return freq_shift_arr

def create_fear_inflection_array(freq_chunks,chunk_size):
    no_of_freq_chunks = len(freq_chunks[0])
    freq_shift_arr = []
    cnt = 0
    frequency = 10.0/(44100.0/float(chunk_size))
    fctr = frequency * numpy.pi * 2
    for freq_region in freq_chunks:
        freq_shift_arr.append([])
        cntTwo = 0
        for freq_chunk in freq_region:
            b = 0.2
            a = float(-b)/float(no_of_freq_chunks-1)
            shift_amt = (float(a)*float(cntTwo)) + float(b)
            # print shift_amt
            freqShiftFactor = numpy.sin(fctr*cntTwo) * shift_amt
            freq_shift_arr[cnt].append(1+freqShiftFactor)
            cntTwo = cntTwo + 1
        cnt = cnt + 1

    return freq_shift_arr

def create_happiness_inflection_array(freq_chunks):
    no_of_freq_chunks = len(freq_chunks[0])
    no_l_h_l = int((3.0/20.0) * no_of_freq_chunks)
    no_h = no_of_freq_chunks - (2 * no_l_h_l)

    # total_chunks = no_h + (2*no_l_h_l)
    # print "nolhl " + str(no_l_h_l)
    # print "noh " + str(no_h)
    # print "no freq chunks " + str(no_of_freq_chunks)

    freq_shift_arr = []
    cnt = 0
    for freq_region in freq_chunks:
        freq_shift_arr.append([])
        cntTwo = 0
        for freq_chunk in freq_region:
            if cntTwo < no_l_h_l:
                b = 0.9
                if no_l_h_l != 1:
                    a = float(1.1- b)/float(no_l_h_l-1)
                else:
                    a = 0
                freqShiftFactor = (float(a)*float(cntTwo)) + float(b)
            elif cntTwo > no_l_h_l + no_h:
                b = 1.12
                if no_l_h_l != 1:
                    a = float(1- b)/float(no_l_h_l-1)
                else:
                    a = 0
                freqShiftFactor = (float(a)*float(cntTwo)) + float(b)
            else:
                freqShiftFactor = 1.12
            freq_shift_arr[cnt].append(freqShiftFactor)
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