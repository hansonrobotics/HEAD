import pyaudio
import wave
import threading
import logging

logger = logging.getLogger('hr.common.sound_file')

class SoundFile(object):

    def __init__(self):
        self.chunk = 1024
        self._interrupt = threading.Event()
        self.is_playing = False
        self.stream = None
        self.wf = None
        self.p = None
        self.lock = threading.RLock()

    def open(self, wavfile):
        self.wf = wave.open(wavfile, 'rb')
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=self.p.get_format_from_width(self.wf.getsampwidth()),
            channels=self.wf.getnchannels(),
            rate=self.wf.getframerate(),
            output=True)
        self.is_playing = False
        self._interrupt.clear()

    def play(self, wavfile):
        try:
            with self.lock:
                self.open(wavfile)
                data = self.wf.readframes(self.chunk)
                self.is_playing = True
                logger.info("Playing {}".format(wavfile))
                while len(data) > 0 and not self._interrupt.is_set():
                    self.stream.write(data)
                    data = self.wf.readframes(self.chunk)
        finally:
            self.is_playing = False
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()
            if self.wf:
                self.wf.close()
            if self.p:
                self.p.terminate()

    def interrupt(self):
        self._interrupt.set()
        logger.info("Sound file is interrupted")

if __name__ == '__main__':
    import time
    sound = SoundFile()
    threading.Timer(0, sound.play, ('sample.wav',)).start()
    threading.Timer(0.5, sound.interrupt).start()
    threading.Timer(1, sound.play, ('sample.wav',)).start()
    while True:
        print "Playing", sound.is_playing
        time.sleep(0.1)
