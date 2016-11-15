from __future__ import division
import os
import sys
import wave
sys.path.insert(0, '/opt/hansonrobotics/lib/python2.7/site-packages/')

from pocketsphinx.pocketsphinx import Decoder

MODELDIR = '/opt/hansonrobotics/share/pocketsphinx/model'

config = Decoder.default_config()
config.set_string('-hmm', os.path.join(MODELDIR, 'en-us/en-us'))
config.set_string('-allphone', os.path.join(MODELDIR, 'en-us/en-us-phone.lm.dmp'))
config.set_float('-lw', 2.0)
config.set_float('-beam', 1e-10)
config.set_float('-pbeam', 1e-10)

def audio2phoneme(audio_file):
    wave_read = wave.open(audio_file, 'rb')
    length = wave_read.getnframes()/wave_read.getframerate()
    wave_read.close()

    # Decode streaming data.
    decoder = Decoder(config)

    buf = bytearray(1024)
    with open(audio_file, 'rb') as f:
        decoder.start_utt()
        while f.readinto(buf):
            decoder.process_raw(buf, False, False)
        decoder.end_utt()

    nframes = decoder.n_frames()


    phonemes = []
    offset = None
    for seg in decoder.seg():
        if offset is None:
            offset = seg.start_frame
        start_frame = seg.start_frame - offset
        end_frame = seg.end_frame - offset
        phonemes.append((
            seg.word, start_frame/nframes*length, end_frame/nframes*length))

    return phonemes

if __name__ == '__main__':
    audio_file = os.path.join(os.path.expanduser('~/.hr/tts/numb'), 'acapelabox_813168.wav')
    #audio_file = os.path.join(os.path.expanduser('~/.hr/tts/numb'), 'hello.wav')
    phonemes = audio2phoneme(audio_file)
    print ('Phonemes: ', phonemes)
