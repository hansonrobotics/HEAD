import pyglet
import threading
import rospy

class SoundFile():

    is_playing = False

    def play(self):
        self.is_playing = True
        self.gletplayer.play()

    def stop(self):
        self.is_playing = False

    def start(self, filename):
        # Load file to pyglet, for playing
        gletsource = pyglet.media.load(filename, streaming=False)
        self.gletplayer.queue(gletsource)
        self.gletplayer.set_handler("on_eos", self.stop)
        self.play()

    def __init__(self):
        #Fix to get the pyglet in background thread exit gracefully on keyboard
        #interrupt (CTRL+C)
        pyglet.clock._get_sleep_time = pyglet.clock.get_sleep_time
        pyglet.clock.get_sleep_time = lambda sleep_idle: pyglet.clock._get_sleep_time(False)

        threading.Timer(0.0, pyglet.app.run).start()
        rospy.on_shutdown(pyglet.app.exit)
        self.gletplayer = pyglet.media.Player()
