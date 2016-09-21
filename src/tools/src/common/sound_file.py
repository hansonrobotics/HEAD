import threading
import logging
import subprocess

logger = logging.getLogger('hr.common.sound_file')

class SoundFile(object):

    def __init__(self):
        self.is_playing = False
        self.lock = threading.RLock()
        self._interrupt = threading.Event()

    def play(self, wavfile):
        self._interrupt.clear()
        try:
            with self.lock:
                proc = subprocess.Popen(['aplay', wavfile])
                job = threading.Timer(0, proc.wait)
                job.daemon = True
                job.start()
                self.is_playing = True
                while proc.poll() is None:
                    if self._interrupt.is_set():
                        proc.terminate()
        finally:
            self.is_playing = False

    def interrupt(self):
        self._interrupt.set()
        logger.info("Sound file is interrupted")


if __name__ == '__main__':
    import time
    sound = SoundFile()
    fname = 'sample.wav'
    threading.Timer(0, sound.play, (fname,)).start()
    threading.Timer(0.5, sound.interrupt).start()
    threading.Timer(1, sound.play, (fname,)).start()
    while True:
        print "Playing", sound.is_playing
        time.sleep(0.1)
