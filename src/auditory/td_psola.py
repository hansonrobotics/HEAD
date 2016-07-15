import voiced_unvoiced as voi
import numpy
from  scipy.io import wavfile
import pitch_mark_first_step as pmfs
import pitch_mark_second_stage as pmss
from pyo import *
import pysptk

def freq_shift_using_td_psola_helper_old(sndarray, freq_chunk, best_pitch_marks_freq_chunk,freqShiftFactor):
    best_pitch_marks_freq_chunk_sorted=  numpy.sort(best_pitch_marks_freq_chunk)
    # print "freq chunk " + str(freq_chunk)
    # print sndarray[freq_chunk[0]:freq_chunk[1]+1]
    maxSndArray = numpy.max(sndarray[freq_chunk[0]:freq_chunk[1]+1])
    minSndArray = numpy.min(sndarray[freq_chunk[0]:freq_chunk[1]+1])
    # print "maxSndArray " + str(maxSndArray)
    # print "minSndArray " + str(minSndArray)
    # or (freq_chunk[0] < 15360 or freq_chunk[0] >= 16383)
    if maxSndArray <= 1000 and maxSndArray >= 0 or minSndArray < 0 and minSndArray >= -1000 :
        return sndarray[freq_chunk[0]:freq_chunk[1]+1]
        # return numpy.zeros(len(sndarray[freq_chunk[0]:freq_chunk[1]+1]))
    # for i in range(freq_chunk[0],freq_chunk[1]+1):
    #     if sndarray[i] <= 500 and sndarray[i]>= 0 or sndarray[i] < 0 and sndarray[i] >= -500:
    #         sndarray[i] = 0.0

    diff_pitch_marks_freq_chunk = numpy.diff(best_pitch_marks_freq_chunk_sorted)
    if len(diff_pitch_marks_freq_chunk) == 0:
        # period = float(1)/float(freq)
        # periodToSamps = period * 44100
        average_period = numpy.int(freq_chunk[1]-freq_chunk[0]+1)
    else:
        average_period = numpy.average(diff_pitch_marks_freq_chunk)
    synthesis_pitch_marks = []


    prepared_segments=[]
    segments_range = []
    cnt = 0
    for i in best_pitch_marks_freq_chunk_sorted:
        if cnt != 0:
            scopeLeft = diff_pitch_marks_freq_chunk[cnt-1]
        if cnt != len(diff_pitch_marks_freq_chunk):
            scopeRight = diff_pitch_marks_freq_chunk[cnt]
        if cnt == 0:
            scopeLeft = i-freq_chunk[0]
        if cnt == len(diff_pitch_marks_freq_chunk):
            scopeRight = freq_chunk[1]-i
        leftSeg = i - scopeLeft
        rightSeg = i + scopeRight
        additional_len = numpy.abs(scopeLeft-scopeRight)

        # print best_pitch_marks_freq_chunk_sorted
        # print diff_pitch_marks_freq_chunk
        # print "cnt " + str(cnt)
        # print "best pitch mark " + str(i)
        # print "scope right " + str(scopeRight)
        # print "scope left " + str(scopeLeft)
        # print "leftSeg " + str(leftSeg)
        # print "rightSeg " + str(rightSeg)
        # print "additional len " + str(additional_len)
        hanning = numpy.asarray(numpy.hanning(scopeLeft +  scopeRight + additional_len))
        if scopeRight >= scopeLeft:
            # print "I ma here 1"
            new_hanning = hanning[additional_len:]
        else:
            # print "I am here 2"
            new_hanning = hanning[0:scopeLeft+scopeRight]
        segment = numpy.asarray(sndarray[leftSeg:rightSeg])
        segmentHanning = segment * new_hanning
        x = numpy.arange(0,freqShiftFactor,len(segment))
        xp = numpy.arange(0,len(segmentHanning))
        newSegmentHanning = numpy.interp(x,xp,segmentHanning)
        # start = leftSeg
        # end = rightSeg
        # diff = end - start
        # print len(new_hanning)
        # print new_hanning
        # print diff
        import matplotlib.pyplot as plt
        # xplot = []
        # for j in range(leftSeg,rightSeg):
        #     xplot.append(j)
        # print "i " + str(i)
        # plt.plot(i,sndarray[i],'o',markersize=10,color='red', label=" best pitch markers")
        # plt.plot(xplot,sndarray[start:end],'-o',color='blue',label="amplitude of sound")
        # plt.plot(xplot,numpy.asarray(new_hanning)*sndarray[i],'-',color='black', label='hanning')
        # plt.xlim(leftSeg,rightSeg)
        # # plt.legend()
        # plt.show()


        segments_range.append([scopeLeft,scopeRight])
        prepared_segments.append(newSegmentHanning)
        cnt = cnt + 1



    new_pitch_mark = best_pitch_marks_freq_chunk_sorted[0]
    new_pitch_marks =[]
    leftSegs = []
    rightSegs = []
    new_pitch = int(float(average_period)/float(freqShiftFactor))
    new_snd_array = numpy.zeros(freq_chunk[1]-freq_chunk[0]+1)

    while new_pitch_mark <= freq_chunk[1]:
        new_pitch_marks.append(new_pitch_mark)
        # minimum = numpy.min(numpy.abs(numpy.asarray(best_pitch_marks_freq_chunk)-new_pitch))
        diff_array = numpy.abs(numpy.asarray(best_pitch_marks_freq_chunk_sorted)-new_pitch_mark)
        minArg = numpy.argmin(diff_array)
        scopeLeft= segments_range[minArg][0]
        scopeRight = segments_range[minArg][1]
        leftSeg = new_pitch_mark - freq_chunk[0] - scopeLeft
        rightSeg = new_pitch_mark - freq_chunk[0] + scopeRight
        # print best_pitch_marks_freq_chunk_sorted
        # print segments_range
        # print diff_array
        # print minArg
        # print new_pitch_mark
        # print "freq_chunk_start " + str(freq_chunk[0])
        # print "freq_chunk_end " + str(freq_chunk[1])
        # print "scopeLeft " + str(scopeLeft)
        # print "scopeRight " + str(scopeRight)
        # print "LeftSeg " + str(leftSeg+ freq_chunk[0])
        # print "RightSeg " + str(rightSeg + freq_chunk[0])
        added_snd_array = prepared_segments[minArg]
        if(leftSeg) < 0:
            added_snd_array = prepared_segments[minArg][-leftSeg:]
            leftSeg = 0
        if(rightSeg) >= len(new_snd_array):
            len_seg = len(prepared_segments[minArg])
            len_snd = len(new_snd_array)
            added_snd_array = prepared_segments[minArg][0:(len_seg-(rightSeg-len_snd)-1)]
            rightSeg = len(new_snd_array)-1
        leftSegs.append(leftSeg+freq_chunk[0])
        rightSegs.append(rightSeg+freq_chunk[0])
        # print "min Arg array " + str(diff_array)
        # print "new_pitch_mark " + str(new_pitch_mark)
        # print "minArg " + str(minArg)
        # print "len new_snd_array_seg " + str(len(new_snd_array[leftSeg:rightSeg]))
        # print "len minArg sndarray " + str(len(added_snd_array))

        # break
        new_snd_array[leftSeg:rightSeg] = numpy.asarray(new_snd_array[leftSeg:rightSeg]) + numpy.asarray(added_snd_array)
        # synthesis_pitch_marks.append(new_pitch_mark)
        new_pitch_mark = new_pitch_mark + new_pitch
        if new_pitch_mark > freq_chunk[1] and rightSeg+freq_chunk[0] < freq_chunk[1]:
            scopeLeft= segments_range[len(segments_range)-1][0]
            scopeRight = segments_range[len(segments_range)-1][1]
            # print "rightSeg " + str(rightSeg + freq_chunk[0])

            # print new_pitch_mark
            # print scopeLeft
            # print scopeRight
            leftSeg = new_pitch_mark - freq_chunk[0] - scopeLeft
            # print leftSeg + freq_chunk[0]
            added_snd_array = prepared_segments[len(prepared_segments)-1][0:freq_chunk[1]-(leftSeg+freq_chunk[0])+1]
            # print "length " + str(freq_chunk[1]-(leftSeg+freq_chunk[0])+1)
            # print "len " +  str(len(prepared_segments[len(prepared_segments)-1]))
            if freq_chunk[1]-(leftSeg+freq_chunk[0])+1 > 0:
                new_snd_array[leftSeg:freq_chunk[1]+1] = numpy.asarray(new_snd_array[leftSeg:freq_chunk[1]+1]) + numpy.asarray(added_snd_array)


    best_pitch_marks_y = []
    for i in best_pitch_marks_freq_chunk_sorted:
        best_pitch_marks_y.append(sndarray[i])
    new_pitch_marks_y = []
    for i in new_pitch_marks:
        new_pitch_marks_y.append(0.0)
    leftSegs_y = []
    for i in leftSegs:
        leftSegs_y.append(sndarray[i])
    rightSegs_y = []
    for i in rightSegs:
        rightSegs_y.append(sndarray[i])

    start = freq_chunk[0]
    end = freq_chunk[1]+1
    diff = end - start
    best_pitch_marks_new = []
    for j in best_pitch_marks_freq_chunk_sorted:
        best_pitch_marks_new.append(j)
    new_pitch_marks_new = []
    for j in new_pitch_marks:
        new_pitch_marks_new.append(j)
    leftSegs_new = []
    for j in leftSegs:
        leftSegs_new.append(j)
    rightSegs_new = []
    for j in rightSegs:
        rightSegs_new.append(j)
    xplotTwo = []
    for i in range(start,end):
        xplotTwo.append(i)

    import matplotlib.pyplot as plt
    # print leftSegs
    # print best_pitch_marks_new
    # plt.plot(best_pitch_marks_new,best_pitch_marks_y,'x',markersize=60,color='red', label=" analysis pitch markers")
    # plt.plot(new_pitch_marks,new_pitch_marks_y,'o',markersize=10,color='black', label=" synthesis pitch markers")
    # # plt.plot(leftSegs,leftSegs_y,'o',markersize=20,color='red', label=" leftSegsStart")
    # # plt.plot(rightSegs,rightSegs_y,'o',markersize=20,color='black', label=" rightSegsStart")
    # plt.plot(xplotTwo,sndarray[start:end],'-o',color='blue',label="amplitude old sound")
    # plt.plot(xplotTwo,new_snd_array,'-',color='red',label="amplitude new sound")
    # plt.xlim(start, end)
    # plt.legend()
    # plt.show()
    return new_snd_array
    # print average_period
    # print new_pitch
    # print synthesis_pitch_marks

def freq_shift_using_td_psola_helper_new(sndarray, freq_chunk, best_pitch_marks_freq_chunk, freqShiftFactor):
    best_pitch_marks_freq_chunk =  numpy.sort(best_pitch_marks_freq_chunk)
    diff_pitch_marks_freq_chunk = numpy.diff(best_pitch_marks_freq_chunk)

    prepared_segments=[]
    segments_range = []
    cnt = 0
    for i in best_pitch_marks_freq_chunk:
        if cnt != 0:
            scopeLeft = diff_pitch_marks_freq_chunk[cnt-1]
        if cnt != len(diff_pitch_marks_freq_chunk):
            scopeRight = diff_pitch_marks_freq_chunk[cnt]
        if cnt == 0:
            scopeLeft = i-freq_chunk[0]
        if cnt == len(diff_pitch_marks_freq_chunk):
            scopeRight = freq_chunk[1]-i
        leftSeg = i - scopeLeft
        rightSeg = i + scopeRight
        additional_len = numpy.abs(scopeLeft-scopeRight)
        hanning = numpy.asarray(numpy.hanning(scopeLeft +  scopeRight + additional_len))
        if scopeRight >= scopeLeft:
            new_hanning = hanning[additional_len:]
        else:
            new_hanning = hanning[0:scopeLeft+scopeRight]
        segment = numpy.asarray(sndarray[leftSeg:rightSeg])
        segmentHanning = segment * new_hanning

        segments_range.append([scopeLeft,scopeRight])
        prepared_segments.append(segmentHanning)
        cnt = cnt + 1

    new_pitch_mark = best_pitch_marks_freq_chunk[0]
    new_snd_array = numpy.zeros(freq_chunk[1]-freq_chunk[0]+1)
    new_pitch_marks =[]
    # new_segments_range = []
    # new_prepared_segments = []
    cntTwo = 0
    # print "best_pitch_marks " + str(best_pitch_marks_freq_chunk)
    while new_pitch_mark <= freq_chunk[1]:
        new_pitch_marks.append(new_pitch_mark)
        diff_array = numpy.abs(numpy.asarray(best_pitch_marks_freq_chunk)-new_pitch_mark)
        minArg = numpy.argmin(diff_array)
        newScopeRight = int(float(segments_range[minArg][1])/float(freqShiftFactor))
        if minArg == len(best_pitch_marks_freq_chunk)-1:
            newScopeRight = int(float(segments_range[minArg-1][1])/float(freqShiftFactor))
        if cntTwo != 0:
            newScopeLeft = new_pitch_mark - new_pitch_marks[cntTwo-1]
        else:
            newScopeLeft = segments_range[0][0]

        newLeftSeg = new_pitch_mark - freq_chunk[0] - newScopeLeft
        newRightSeg = new_pitch_mark - freq_chunk[0] + newScopeRight

        if newRightSeg >= len(new_snd_array)-1:
            # print " IM HERE " + str(new_pitch_mark)
            newScopeRight = len(new_snd_array) - (new_pitch_mark - freq_chunk[0])
            newRightSeg = len(new_snd_array)-1

        # print "freq_chunk " + str(freq_chunk)
        # print "new_pitch_marks " + str(new_pitch_marks)
        # print "current syn pitch mark " + str(new_pitch_mark)
        # print "segments range " + str(segments_range)
        # print "minArg " + str(minArg)
        # print "newScopeRight " + str(newScopeRight)
        # print "newScopeLeft " + str(newScopeLeft)
        # print "newLeftSeg " + str(newLeftSeg)
        # print "newRightSeg " + str(newRightSeg)

        scopeLeft = segments_range[minArg][0]
        scopeRight = segments_range[minArg][1]
        if newScopeLeft != 0:
            newLeftFreqShiftFactor = float(scopeLeft)/float(newScopeLeft)
            if scopeLeft != 0:
                xLeft = numpy.arange(0,scopeLeft,newLeftFreqShiftFactor)
                xpLeft = numpy.arange(0,scopeLeft)
                newSegmentLeft = numpy.interp(xLeft,xpLeft,prepared_segments[minArg][0:scopeLeft])
                # print "lenxLeft "  + str(len(xLeft))
                # print "lenxpLeft " + str(len(xpLeft))
            else:
                newSegmentLeft = []
                newLeftSeg = new_pitch_mark - freq_chunk[0]
        else:
            newSegmentLeft = []

        if newScopeRight != 0:
            newRightFreqShiftFactor = float(scopeRight)/float(newScopeRight)
            if scopeRight != 0:
                xRight = numpy.arange(0,scopeRight,newRightFreqShiftFactor)
                xpRight = numpy.arange(0,scopeRight)
                newSegmentRight = numpy.interp(xRight,xpRight,prepared_segments[minArg][scopeLeft:])
                # print "lenxRight "  + str(len(xRight))
                # print "lenxpRight " + str(len(xpRight))
            else:
                newSegmentRight = []
                newRightSeg = new_pitch_mark - freq_chunk[0]
        else:
            newSegmentRight = []

        newSegmentHanning = numpy.concatenate([newSegmentLeft,newSegmentRight])
        newSegmentHanning = newSegmentHanning[0:newRightSeg-newLeftSeg]
        added_snd_array = newSegmentHanning
        # print "lenHanning " + str(len(newSegmentHanning))

        # new_segments_range.append([newScopeLeft,newScopeRight])
        # new_prepared_segments.append(newSegmentHanning)
        new_snd_array[newLeftSeg:newRightSeg] = numpy.asarray(new_snd_array[newLeftSeg:newRightSeg]) + numpy.asarray(added_snd_array)
        new_pitch_mark = new_pitch_mark + newScopeRight
        cntTwo = cntTwo + 1

    best_pitch_marks_y = []
    for i in best_pitch_marks_freq_chunk:
        best_pitch_marks_y.append(sndarray[i])

    new_pitch_marks_y = []
    for i in new_pitch_marks:
        new_pitch_marks_y.append(0.0)

    start = freq_chunk[0]
    end = freq_chunk[1]+1
    diff = end - start
    xplotTwo = []
    for i in range(start,end):
        xplotTwo.append(i)

    import matplotlib.pyplot as plt
    # plt.plot(best_pitch_marks_freq_chunk,best_pitch_marks_y,'x',markersize=60,color='red', label=" analysis pitch markers")
    # plt.plot(new_pitch_marks,new_pitch_marks_y,'o',markersize=10,color='black', label=" synthesis pitch markers")
    # # plt.plot(leftSegs,leftSegs_y,'o',markersize=20,color='red', label=" leftSegsStart")
    # # plt.plot(rightSegs,rightSegs_y,'o',markersize=20,color='black', label=" rightSegsStart")
    # plt.plot(xplotTwo,sndarray[start:end],'-',color='blue',label="amplitude old sound")
    # plt.plot(xplotTwo,new_snd_array,'-',color='red',label="amplitude new sound")
    # plt.xlim(start, end)
    # plt.legend()
    # plt.show()
    return numpy.asarray(new_snd_array)

#Analyzes a whole sndarray only in reference to its pitch marks without considering freq chunks nor voiced regions
#Yields much better result reducing the amount of jitter
def freq_shift_using_td_psola_helper_new_two(sndarray, best_pitch_marks, freqShiftFactor):
    best_pitch_marks =  numpy.sort(best_pitch_marks)
    diff_pitch_marks = numpy.diff(best_pitch_marks)

    if len(best_pitch_marks) == 0:
        return sndarray

    prepared_segments=[]
    segments_range = []
    cnt = 0
    for i in best_pitch_marks:
        if cnt != 0:
            scopeLeft = diff_pitch_marks[cnt-1]
        if cnt != len(diff_pitch_marks):
            scopeRight = diff_pitch_marks[cnt]
        if cnt == 0:
            scopeLeft = i
        if cnt == len(diff_pitch_marks):
            scopeRight = len(sndarray)-i
        leftSeg = i - scopeLeft
        rightSeg = i + scopeRight
        additional_len = numpy.abs(scopeLeft-scopeRight)
        hanning = numpy.asarray(numpy.hanning(scopeLeft +  scopeRight + additional_len))
        if scopeRight >= scopeLeft:
            new_hanning = hanning[additional_len:]
        else:
            new_hanning = hanning[0:scopeLeft+scopeRight]
        segment = numpy.asarray(sndarray[leftSeg:rightSeg])
        segmentHanning = segment * new_hanning

        segments_range.append([scopeLeft,scopeRight])
        prepared_segments.append(segmentHanning)
        cnt = cnt + 1

    new_pitch_mark = best_pitch_marks[0]
    new_snd_array = numpy.zeros(len(sndarray))
    new_pitch_marks =[]
    # new_segments_range = []
    # new_prepared_segments = []
    cntTwo = 0
    # print "best_pitch_marks " + str(best_pitch_marks)
    while new_pitch_mark < len(sndarray):
        new_pitch_marks.append(new_pitch_mark)
        diff_array = numpy.abs(numpy.asarray(best_pitch_marks)-new_pitch_mark)
        minArg = numpy.argmin(diff_array)
        newScopeRight = int(float(segments_range[minArg][1])/float(freqShiftFactor))
        # print "minArg " + str(minArg)
        if minArg == len(best_pitch_marks)-1 and cntTwo != 0:
            # newScopeRight = int(float(segments_range[minArg-1][1])/float(freqShiftFactor))
            newScopeRight = new_pitch_mark - new_pitch_marks[cntTwo-1]
        if cntTwo != 0:
            newScopeLeft = new_pitch_mark - new_pitch_marks[cntTwo-1]
        else:
            newScopeLeft = segments_range[0][0]

        newLeftSeg = new_pitch_mark  - newScopeLeft
        newRightSeg = new_pitch_mark + newScopeRight

        if newRightSeg >= len(new_snd_array)-1:
            # print " IM HERE " + str(new_pitch_mark)
            newScopeRight = len(new_snd_array) - new_pitch_mark
            newRightSeg = len(new_snd_array)-1

        # print "len of sndarray " + str(len(sndarray))

        # print "new_pitch_marks " + str(new_pitch_marks)
        # print "current syn pitch mark " + str(new_pitch_mark)
        # print "segments range " + str(segments_range)

        # print "newScopeRight " + str(newScopeRight)
        # print "newScopeLeft " + str(newScopeLeft)
        # print "newLeftSeg " + str(newLeftSeg)
        # print "newRightSeg " + str(newRightSeg)

        scopeLeft = segments_range[minArg][0]
        scopeRight = segments_range[minArg][1]
        if newScopeLeft != 0:
            newLeftFreqShiftFactor = float(scopeLeft)/float(newScopeLeft)
            if scopeLeft != 0:
                xLeft = numpy.arange(0,scopeLeft,newLeftFreqShiftFactor)
                xpLeft = numpy.arange(0,scopeLeft)
                newSegmentLeft = numpy.interp(xLeft,xpLeft,prepared_segments[minArg][0:scopeLeft])
                # print "lenxLeft "  + str(len(xLeft))
                # print "lenxpLeft " + str(len(xpLeft))
            else:
                newSegmentLeft = []
                newLeftSeg = new_pitch_mark
        else:
            newSegmentLeft = []

        if newScopeRight != 0:
            newRightFreqShiftFactor = float(scopeRight)/float(newScopeRight)
            if scopeRight != 0:
                xRight = numpy.arange(0,scopeRight,newRightFreqShiftFactor)
                xpRight = numpy.arange(0,scopeRight)
                newSegmentRight = numpy.interp(xRight,xpRight,prepared_segments[minArg][scopeLeft:])
                # print "lenxRight "  + str(len(xRight))
                # print "lenxpRight " + str(len(xpRight))
            else:
                newSegmentRight = []
                newRightSeg = new_pitch_mark
        else:
            newSegmentRight = []

        newSegmentHanning = numpy.concatenate([newSegmentLeft,newSegmentRight])
        newSegmentHanning = newSegmentHanning[0:newRightSeg-newLeftSeg]
        added_snd_array = newSegmentHanning
        # print "lenHanning " + str(len(newSegmentHanning))

        # new_segments_range.append([newScopeLeft,newScopeRight])
        # new_prepared_segments.append(newSegmentHanning)
        new_snd_array[newLeftSeg:newRightSeg] = numpy.asarray(new_snd_array[newLeftSeg:newRightSeg]) + numpy.asarray(added_snd_array)
        new_pitch_mark = new_pitch_mark + newScopeRight
        cntTwo = cntTwo + 1

    best_pitch_marks_y = []
    for i in best_pitch_marks:
        best_pitch_marks_y.append(sndarray[i])

    new_pitch_marks_y = []
    for i in new_pitch_marks:
        new_pitch_marks_y.append(0.0)

    start = 0
    end = 12000
    diff = end - start
    xplotTwo = []
    for i in range(start,end):
        xplotTwo.append(i)

    import matplotlib.pyplot as plt
    # plt.plot(best_pitch_marks,best_pitch_marks_y,'x',markersize=30,color='red', label=" analysis pitch markers")
    # plt.plot(new_pitch_marks,new_pitch_marks_y,'o',markersize=5,color='black', label=" synthesis pitch markers")
    # # plt.plot(leftSegs,leftSegs_y,'o',markersize=20,color='red', label=" leftSegsStart")
    # # plt.plot(rightSegs,rightSegs_y,'o',markersize=20,color='black', label=" rightSegsStart")
    # plt.plot(xplotTwo,sndarray[start:end],'-',color='blue',label="amplitude old sound")
    # plt.plot(xplotTwo,new_snd_array[start:end],'-',color='red',label="amplitude new sound")
    # plt.xlim(start, end)
    # plt.xticks()
    # # plt.legend()
    # plt.show()
    return numpy.asarray(new_snd_array)

#same function as helper_new_two but accepts an array for freqshiftfactor
def freq_shift_using_td_psola_helper_newest(sndarray,voiced_region, best_pitch_marks_freq_chunks,freqShiftFactor):
    best_pitch_marks = []
    pitch_marks_cnt = []
    for p_marks_freq_chunk in best_pitch_marks_freq_chunks:
        pitch_marks_cnt.append(len(p_marks_freq_chunk))
        for p_marks in p_marks_freq_chunk:
            best_pitch_marks.append(p_marks)

    best_pitch_marks =  numpy.sort(best_pitch_marks)
    diff_pitch_marks = numpy.diff(best_pitch_marks)

    if len(best_pitch_marks) == 0:
        return sndarray

    prepared_segments=[]
    segments_range = []
    cnt = 0
    # print "Voiced REGION " + str(voiced_region)
    # print "diff pitch marks" + str(diff_pitch_marks)
    # print "best pitch marks" + str(best_pitch_marks)
    for i in best_pitch_marks:
        if cnt != 0:
            scopeLeft = diff_pitch_marks[cnt-1]
        if cnt != len(diff_pitch_marks):
            scopeRight = diff_pitch_marks[cnt]
        if cnt == 0:
            scopeLeft = i - voiced_region[0]
        if cnt == len(diff_pitch_marks):
            scopeRight = len(sndarray)-(i - voiced_region[0])
        leftSeg = i - scopeLeft - voiced_region[0]
        rightSeg = i + scopeRight - voiced_region[0]

        # print "cnt " + str(cnt)
        # print "best pitch mark " + str(i)
        # print "scope right " + str(scopeRight)
        # print "scope left " + str(scopeLeft)
        # print "leftSeg " + str(leftSeg)
        # print "rightSeg " + str(rightSeg)
        additional_len = numpy.abs(scopeLeft-scopeRight)
        hanning = numpy.asarray(numpy.hanning(scopeLeft +  scopeRight + additional_len))
        if scopeRight >= scopeLeft:
            new_hanning = hanning[additional_len:]
        else:
            new_hanning = hanning[0:scopeLeft+scopeRight]
        segment = numpy.asarray(sndarray[leftSeg:rightSeg])
        segmentHanning = segment * new_hanning

        segments_range.append([scopeLeft,scopeRight])
        prepared_segments.append(segmentHanning)
        cnt = cnt + 1

    new_pitch_mark = best_pitch_marks[0]
    new_snd_array = numpy.zeros(len(sndarray))
    new_pitch_marks =[]
    cntTwo = 0
    cntFreqChunks = 0
    while new_pitch_mark < len(sndarray) + voiced_region[0]:
        new_pitch_marks.append(new_pitch_mark)
        freq_chunk_pitch_marks_sorted = numpy.sort(best_pitch_marks_freq_chunks[cntFreqChunks])
        dist_array = numpy.abs(freq_chunk_pitch_marks_sorted-new_pitch_mark)
        preMinArg = numpy.argmin(dist_array)

        lenSort = len(freq_chunk_pitch_marks_sorted)
        if new_pitch_mark > freq_chunk_pitch_marks_sorted[lenSort-1] and cntFreqChunks + 1 != len(best_pitch_marks_freq_chunks):
            freq_chunk_pitch_marks_sorted_next = numpy.sort(best_pitch_marks_freq_chunks[cntFreqChunks+1])
            dist_array_next = numpy.abs(freq_chunk_pitch_marks_sorted_next - new_pitch_mark)
            if dist_array_next[0] <= dist_array[len(dist_array)-1]:
                preMinArg = 0
                cntFreqChunks = cntFreqChunks + 1

        cntPrevPmarks = int(numpy.sum(pitch_marks_cnt[0:cntFreqChunks]))
        minArg = cntPrevPmarks + preMinArg
        # print "minArg " + str(minArg)
        # print "cntFreqChunks " + str(cntFreqChunks)
        # print best_pitch_marks_freq_chunks
        newScopeRight = int(float(segments_range[minArg][1])/float(freqShiftFactor[cntFreqChunks]))

        # comment below is another approach to dealing with ends of freq chunks
        # I move the new pitch mark immediately to the next freq chunk but needs more work

        # if preMinArg == len(best_pitch_marks_freq_chunks[cntFreqChunks])-1:
        #     cntFreqChunks = cntFreqChunks + 1
        #     while cntFreqChunks != len(best_pitch_marks_freq_chunks) and len(best_pitch_marks_freq_chunks[cntFreqChunks]) == 0:
        #         cntFreqChunks = cntFreqChunks + 1
        #     if cntFreqChunks == len(best_pitch_marks_freq_chunks):
        #         break
        #     p_marks_sorted = numpy.sort(best_pitch_marks_freq_chunks[cntFreqChunks])
        #     # if len(best_pitch_marks_freq_chunks[cntFreqChunks]) != 0:
        #     #     new_pitch_mark = p_marks_sorted[0]
        #     #     continue
        #     newScopeRight = p_marks_sorted[0] - new_pitch_mark

        if cntTwo != 0:
            newScopeLeft = new_pitch_mark - new_pitch_marks[cntTwo-1]
        else:
            newScopeLeft = segments_range[0][0]

        newLeftSeg = new_pitch_mark - newScopeLeft - voiced_region[0]
        newRightSeg = new_pitch_mark + newScopeRight - voiced_region[0]

        if newRightSeg >= len(new_snd_array)-1:
            newScopeRight = len(new_snd_array) - (new_pitch_mark - voiced_region[0])
            newRightSeg = len(new_snd_array)-1
            # print "len of sndarray " + str(len(sndarray))

        # print "new_pitch_marks " + str(new_pitch_marks)
        # print "current syn pitch mark " + str(new_pitch_mark)
        # print "segments range " + str(segments_range)

        # print "newScopeRight " + str(newScopeRight)
        # print "newScopeLeft " + str(newScopeLeft)
        # print "newLeftSeg " + str(newLeftSeg)
        # print "newRightSeg " + str(newRightSeg)

        scopeLeft = segments_range[minArg][0]
        scopeRight = segments_range[minArg][1]
        if newScopeLeft != 0:
            newLeftFreqShiftFactor = float(scopeLeft)/float(newScopeLeft)
            if scopeLeft != 0:
                xLeft = numpy.arange(0,scopeLeft,newLeftFreqShiftFactor)
                xpLeft = numpy.arange(0,scopeLeft)
                newSegmentLeft = numpy.interp(xLeft,xpLeft,prepared_segments[minArg][0:scopeLeft])
            else:
                newSegmentLeft = []
                newLeftSeg = new_pitch_mark
        else:
            newSegmentLeft = []

        if newScopeRight != 0:
            newRightFreqShiftFactor = float(scopeRight)/float(newScopeRight)
            if scopeRight != 0:
                xRight = numpy.arange(0,scopeRight,newRightFreqShiftFactor)
                xpRight = numpy.arange(0,scopeRight)
                newSegmentRight = numpy.interp(xRight,xpRight,prepared_segments[minArg][scopeLeft:])
            else:
                newSegmentRight = []
                newRightSeg = new_pitch_mark
        else:
            newSegmentRight = []

        newSegmentHanning = numpy.concatenate([newSegmentLeft,newSegmentRight])
        newSegmentHanning = newSegmentHanning[0:newRightSeg-newLeftSeg]
        added_snd_array = newSegmentHanning

        new_snd_array[newLeftSeg:newRightSeg] = numpy.asarray(new_snd_array[newLeftSeg:newRightSeg]) + numpy.asarray(added_snd_array)
        new_pitch_mark = new_pitch_mark + newScopeRight
        cntTwo = cntTwo + 1

    best_pitch_marks_y = []
    for i in best_pitch_marks:
        best_pitch_marks_y.append(sndarray[i-voiced_region[0]])

    new_pitch_marks_y = []
    for i in new_pitch_marks:
        new_pitch_marks_y.append(0.0)

    start = 16000
    end = len(sndarray)-3500
    diff = end - start
    xplotTwo = []
    for i in range(start,end):
        xplotTwo.append(i)

    import matplotlib.pyplot as plt
    # plt.plot(best_pitch_marks,best_pitch_marks_y,'x',markersize=30,color='red', label=" analysis pitch markers")
    # plt.plot(new_pitch_marks,new_pitch_marks_y,'o',markersize=10,color='black', label=" synthesis pitch markers")
    # # plt.plot(leftSegs,leftSegs_y,'o',markersize=20,color='red', label=" leftSegsStart")
    # # plt.plot(rightSegs,rightSegs_y,'o',markersize=20,color='black', label=" rightSegsStart")
    # plt.plot(xplotTwo,sndarray[start:end],'-',color='blue',label="amplitude old sound")
    # plt.plot(xplotTwo,new_snd_array[start:end],'-o',color='red',label="amplitude new sound")
    # plt.xlim(start, end)
    # plt.xticks()
    # # plt.legend()
    # plt.show()
    return new_snd_array

def freq_shift_using_td_psola_newest(sndarray,voiced_regions,best_pitch_marks,freqShiftFactor):
    sndarray_cp = []
    for snd in sndarray:
        sndarray_cp.append(snd)
    cnt = 0
    for voiced_region in voiced_regions:
        sndarray_in = sndarray_cp[voiced_region[0]:voiced_region[1]+1]
        best_pitch_marks_in = best_pitch_marks[cnt]
        # freq_chunks_in = freq_chunks[cnt]
        freqShiftFactor_in = freqShiftFactor[cnt]
        new_snd = freq_shift_using_td_psola_helper_newest(sndarray_in,voiced_region, best_pitch_marks_in, freqShiftFactor_in)
        cnt_two = 0
        for snd_out in new_snd:
            sndarray_cp[voiced_region[0]+cnt_two] = numpy.int16(snd_out)
            cnt_two = cnt_two + 1
        cnt = cnt + 1
    return sndarray_cp

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

def freq_shift_using_td_psola(sndarray,chunk_size,freqShiftFactor,best_pitch_marks_info,freq_chunks_info):
    # new_sndarray_zeros = numpy.zeros(len(sndarray))
    new_sndarray = []
    for i in sndarray:
        new_sndarray.append(numpy.int16(i))
    # f0 = voi.get_freq_array(new_sndarray,44100,chunk_size)
    cnt = 0
    spots = []
    # print "freq_chunks_info " + str(freq_chunks_info)
    # print len(sndarray)
    for freq_chunks_region in freq_chunks_info:
        cntTwo = 0
        new_sndarray_two = []
        # print "NEW REGION STARTED!!!!!!!!!!!!!!"
        # print len(sndarray)
        for freq_chunk in freq_chunks_region:
            # spot = freq_chunk[0]/chunk_size
            # freq = f0[spot]
            if len(best_pitch_marks_info[cnt][cntTwo]) != 0:
                x = freq_shift_using_td_psola_helper_new(sndarray,freq_chunk,best_pitch_marks_info[cnt][cntTwo], freqShiftFactor)
            else:
                x = sndarray[freq_chunk[0]:freq_chunk[1]+1]
            # diff_arrays = numpy.asarray(x) - numpy.asarray(sndarray[freq_chunk[0]:freq_chunk[1]+1])

            # print "diff array " + str(diff_arrays.tolist())
            #
            # best_pitch_marks_y = []
            # for i in best_pitch_marks_info[cnt][cntTwo]:
            #     best_pitch_marks_y.append(sndarray[i])
            #
            # start = freq_chunk[0]
            # end = freq_chunk[1]
            # diff = end - start
            # best_pitch_marks_new = []
            # for j in best_pitch_marks_info[cnt][cntTwo]:
            #     best_pitch_marks_new.append(j-start)
            #
            # import matplotlib.pyplot as plt
            # plt.plot(best_pitch_marks_new,best_pitch_marks_y,'o',markersize=10,color='red', label=" best pitch markers")
            # plt.plot(y[0:diff],sndarray[start:end],'-o',color='blue')
            # plt.plot(numpy.asarray(x),'-',color='red')
            # plt.xlim(0, len(x))
            # plt.legend()
            # plt.show()
            for i in range(0,len(x)):
                spot = i + freq_chunk[0]
                # print str(spot) + " " + str(int(x[i])) + " " + str(sndarray[spot]) + " " + str(new_sndarray[spot])
                # print type(x[i])
                # if x[i] <= 200 and x[i] >= 0 or x[i] < 0 and x[i] >= -200:
                #     new_sndarray_zeros[spot] = 0.0
                # else:
                # new_sndarray_zeros[spot] = int(x[i])
                new_sndarray[spot] = numpy.int16(x[i])
                # new_sndarray[spot] = x[i]
                spots.append(spot)
            cntTwo = cntTwo + 1
        cnt = cnt + 1

    # new_sndarray_y = []
    # for i in spots:
    #     new_sndarray_y.append(new_sndarray[i])

    # import matplotlib.pyplot as plt

    # diff = numpy.asarray(new_sndarray) - numpy.asarray(new_sndarray_zeros)
    # print freq_chunks_info[0][0][0]
    # print diff[0:freq_chunks_info[0][0][0]].tolist()
    # print new_sndarray[0:freq_chunks_info[0][0][0]]
    # print len(new_sndarray[0:freq_chunks_info[0][0][0]])
    # print new_sndarray_zeros[0:freq_chunks_info[0][0][0]].tolist()
    # print len(new_sndarray_zeros[0:freq_chunks_info[0][0][0]].tolist())
    # plt.plot(new_sndarray_zeros,'o',markersize=5,color='red', label=" amplitude_new sound")
    # # plt.plot(x[start:end],'-',color='black',label='amplitude old sound')
    # plt.xlim(0, len(sndarray))
    # plt.legend()
    # plt.show()
    return new_sndarray

if __name__ == "__main__":
    filename= "C:/Users/rediet/Documents/Vocie-samples/salli.wav"
    filenameFreqShift = "C:/Users/rediet/Documents/Vocie-samples/salliVoicedFreqShift1_5_bad_3.wav"

    fs, x = wavfile.read(filename)
    y = numpy.arange(0,len(x),1)
    x = voi.get_one_channel_array(x)
    chunk_size = 1024

    import time
    sTime = time.time()
    sTimeAll = sTime
    # voi.plot(y,x,len(x),"signal amplitude")
    f0 = voi.get_freq_array(x,fs,chunk_size)
    vSig = voi.get_signal_voiced_unvoiced_starting_info(x,f0,fs,chunk_size)
    lengthVoiced = voi.get_signal_voiced_length_info(x,vSig)
    # lengthUnvoiced = voi.get_signal_unvoiced_length_info(x,vSig)
    #
    # voiced_regions,unvoiced_regions,freq_array = voi.get_voiced_region_chunks_two(x,chunk_size)
    voiced_regions = voi.get_voiced_region_chunks(vSig,lengthVoiced)
    eTime = time.time()
    print "time taken by get voiced regions " + str(eTime - sTime)
    # unvoiced_regions = voi.get_unvoiced_region_chunks(vSig,lengthUnvoiced)
    # voiced_regions = []
    # vr_start = 45056
    # vr_stop = vr_start + (chunk_size*20) - 1
    # diff = vr_stop - vr_start + 1
    # voiced_regions.append([0,diff])
    # print " voiced regions " + str(voiced_regions)

    sTime = time.time()
    pitch_marks,voiced_region_freq_chunk_windows_pitch_marks_obj = pmfs.get_pitch_marks_regions(x, f0, voiced_regions,chunk_size, "voiced")
    eTime = time.time()
    print "time taken by pmfs is " + str(eTime-sTime)
    sTime = time.time()
    best_voiced_region_freq_chunk_windows_pitch_marks_obj = pmss.optimal_accumulated_log_probability(x,voiced_region_freq_chunk_windows_pitch_marks_obj)
    eTime = time.time()
    print "time taken by pmss is " + str(eTime - sTime)

    best_pitch_marks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["best_pitch_marks"]
    freq_chunks_info = best_voiced_region_freq_chunk_windows_pitch_marks_obj["freq_chunks"]
    # print " voiced regions " + str(voiced_regions)
    # print "best pitch marks " + str(best_pitch_marks_info)
    # print "freq chunks " + str(freq_chunks_info)

    # freq_shift_arr = create_constant_freq_shift_array(freq_chunks_info,1.5)
    sTime = time.time()
    freq_shift_arr = create_vibrato_freq_shift_array(freq_chunks_info,chunk_size)
    eTime = time.time()
    print "time taken by create vibrato is " + str(eTime-sTime)

    # print freq_shift_arr
    # best_pitch_marks = []
    # for best_pitch_marks_region in best_pitch_marks_info:
    #     for best_pitch_marks_freq_chunk in best_pitch_marks_region:
    #         for i in best_pitch_marks_freq_chunk:
    #             best_pitch_marks.append(i)
    #
    #
    # # new_x = freq_shift_using_td_psola(x,chunk_size,2,best_pitch_marks_info , freq_chunks_info)
    # new_x = freq_shift_using_td_psola_helper_new_two(x,best_pitch_marks,1.5)

    sTime = time.time()
    new_x = freq_shift_using_td_psola_newest(x,voiced_regions,best_pitch_marks_info,freq_shift_arr)
    eTime = time.time()
    eTimeAll = eTime
    print "time taken by create td_psola newest " + str(eTime-sTime)
    print "time taken by all modules is " + str(eTimeAll-sTimeAll)

    # new_sndarray = []
    # for i in range(0,len(new_x)):
    #     new_sndarray.append(numpy.int16(new_x[i]))
    #
    # new_sndarray = voi.make_two_channels(new_sndarray)
    # voi.write_to_new_file(filenameFreqShift,numpy.asarray(new_sndarray))



    # start = 0
    # end = len(x)
    # diff = end - start
    # xplotTwo = []
    # for i in range(start,end):
    #     xplotTwo.append(i)
    # import matplotlib.pyplot as plt
    # plt.plot(xplotTwo,x[start:end],'-o',markersize=5,color='blue',label="amplitude old sound")
    # plt.plot(xplotTwo,new_sndarray[start:end],'-o',markersize = 5, color='red',label="amplitude new sound")
    # plt.xlim(start, end)
    # plt.xticks()
    # plt.legend()
    # plt.show()
