import StringIO
import httplib
import subprocess
import threading
import urllib
import wave
import maryclient_http as mc

def convertSpeech(txtFileName, wavFileName):
    t = threading.Thread(target = mc.marytts_server)
    t.start()
    import time
    time.sleep(5)
    #marytts_server()
    client = mc.maryclient()
    client.set_locale ("en_US")
    #client.set_locale ("de")
    # english, male
    #client.set_voice ("dfki-spike")
    #client.set_voice ("dfki-obadiah")
    # client.set_voice ("dfki-obadiah-hsmm")
    #client.set_voice ("cmu-bdl-hsmm")
    # client.set_voice ("cmu-rms-hsmm")
    #english, female
    # client.set_voice ("dfki-poppy")
    #client.set_voice ("dfki-poppy-hsmm")
    #client.set_voice ("dfki-prudence")
    # client.set_voice ("dfki-prudence-hsmm")
    client.set_voice ("cmu-slt-hsmm")
    filename = "Voice-samples/maryTTS.wav"
    f = open('Voice-samples/readTTS.txt')

    for line in iter(f):
        line = line.strip('\n').lower()
        line = line.strip('.').lower()


    the_sound = client.generate(line)
    buf = StringIO.StringIO(the_sound)
    print buf.len
    wf = wave.open(buf, 'rb')
    print wf.getnframes()
    print wf.getframerate()
    print wf.getparams()
    print wf.getnchannels()
    print wf.getsampwidth()
    BUFFSIZE = 2
    g = 2.2
    import sys
    print sys.byteorder
    # print sys.getsizeof(BUFFSIZE)
    # print sys.getsizeof(g)
    snd_array = []
    for i in range(0,wf.getnframes()):
        import struct
        buf = wf.readframes(1)

        v = buf[0] + buf[1]
        val = struct.unpack('<h', v)[0]
        snd_array.append(val)
        # print str(int_no) + " " + str(buf) + " " + str(list(buf))
        # print str(ord(buf[0])) + " " + str(i) + " " + str(ord(buf[1]))


    # import struct
    # print struct.unpack("<H", buf)
    from scipy.io import wavfile
    import numpy
    import voiced_unvoiced as voi
    import EmotiveSpeech as ES

    # x = numpy.array(map(ord, list(buf)))
    # y = map(numpy.int16,x)
    # print x[0:10000].tolist()
    # print snd_array
    snd_array  = voi.make_two_channels(snd_array)
    sep = filename.split("/")
    name = sep[len(sep)-1].split(".")[0]
    from src.auditory.unused_files import Emotion_Extraction as ee
    from textblob import TextBlob
    # emotionType = "Neutral"
    from ems import em
    emotionType = "Neutral"
    f = open('Voice-samples/readTTS.txt')
    for line in iter(f):

        line = line.strip('\n').lower()
        line = line.strip('.').lower()
        sent_splited = line.split(' ')
        for x in em:

            # print str(sent_splited) + " " + str(sent_splited.__contains__(x)) + " " + str(x)
            if sent_splited.__contains__(x):
                sent_sentiment =ee.analyze_sentiment(TextBlob(line))
                emotionType = em[x]
                print line+'.'+' '+'<'+em[x]+'>'+' '+'<'+sent_sentiment+'>'
                break

    filenameChanged = "Voice-samples/" + str(name) + str(emotionType) + ".wav"
    wavfile.write(filenameChanged,44100,numpy.array(numpy.int16(snd_array)))
    ES.emotiveSpeech(filenameChanged,str(emotionType))

    # import winsound
    # winsound.PlaySound(the_sound,winsound.SND_MEMORY)