#!/usr/bin/env python
# -*- coding: utf-8 -*-

import StringIO
import httplib
import subprocess
import threading
import urllib
import wave


def marytts_server():
    subprocess.call(['libs/marytts-5.0/bin/marytts-server.sh'])

class maryclient:

    def __init__(self):

       	self.host = "127.0.0.1"
       	self.port = 59125
       	self.input_type = "TEXT"
       	self.output_type = "AUDIO"
        self.audio = "WAVE_FILE"
       	self.locale = "en_US"
        self.voice = "cmu-bdl-hsmm"

    def set_host(self, a_host):
        """Set the host for the TTS server."""
        self.host = a_host

    def get_host(self):
        """Get the host for the TTS server."""
        self.host

    def set_port(self, a_port):
        """Set the port for the TTS server."""
        self.port = a_port

    def get_port(self):
        """Get the port for the TTS server."""
        self.port

    def set_input_type(self, type):
        """Set the type of input being 
           supplied to the TTS server
           (such as 'TEXT')."""
        self.input_type = type

    def get_input_type(self):
        """Get the type of input being 
           supplied to the TTS server
           (such as 'TEXT')."""
        self.input_type

    def set_output_type(self, type):
        """Set the type of input being 
           supplied to the TTS server
           (such as 'AUDIO')."""
        self.output_type = type

    def get_output_type(self):
        """Get the type of input being 
           supplied to the TTS server
           (such as "AUDIO")."""
        self.output_type

    def set_locale(self, a_locale):
        """Set the locale
           (such as "en_US")."""
        self.locale = a_locale

    def get_locale(self):
        """Get the locale
           (such as "en_US")."""
        self.locale

    def set_audio(self, audio_type):
        """Set the audio type for playback
           (such as "WAVE_FILE")."""
        self.audio = audio_type

    def get_audio(self):
        """Get the audio type for playback
           (such as "WAVE_FILE")."""
        self.audio

    def set_voice(self, a_voice):
        """Set the voice to speak with
           (such as "dfki-prudence-hsmm")."""
        self.voice = a_voice

    def get_voice(self):
        """Get the voice to speak with
           (such as "dfki-prudence-hsmm")."""
        self.voice

    def generate(self, message):
        """Given a message in message,
           return a response in the appropriate
           format."""
        raw_params = {"INPUT_TEXT": message,
                "INPUT_TYPE": self.input_type,
                "OUTPUT_TYPE": self.output_type,
                "LOCALE": self.locale,
                "AUDIO": self.audio,
                "VOICE": self.voice,
                }
        params = urllib.urlencode(raw_params)
        headers = {}

        # Open connection to self.host, self.port.
        conn = httplib.HTTPConnection(self.host, self.port)

        #conn.set_debuglevel(5)
        
        conn.request("POST", "/process", params, headers)
        response = conn.getresponse()
        if response.status != 200:
            print response.getheaders()
            raise RuntimeError("{0}: {1}".format(response.status,
                response.reason))
        return response.read()

if __name__ == "__main__":

    t = threading.Thread(target = marytts_server)
    t.start()
    import time
    time.sleep(5)
    #marytts_server()
    client = maryclient()
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


