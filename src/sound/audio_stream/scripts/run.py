#!/usr/bin/env python
import pyaudio
import contextlib
import rospy

from audio_stream.msg import AudioData

@contextlib.contextmanager
def record_audio(channels, rate, chunk):
    """Opens a recording stream in a context manager."""
    audio_interface = pyaudio.PyAudio()
    audio_stream = audio_interface.open(
        format=pyaudio.paInt16, channels=channels, rate=rate,
        input=True, frames_per_buffer=chunk,
    )

    yield audio_stream

    audio_stream.stop_stream()
    audio_stream.close()
    audio_interface.terminate()

if __name__ == '__main__':
    rospy.init_node('audio_stream')
    audio_publisher = rospy.Publisher('speech_audio', AudioData, queue_size=1)
    rate = rospy.get_param('audio_rate', 16000)
    channels = rospy.get_param('audio_channels', 1)
    chunk = int(rate / 10)  # 100ms
    with record_audio(channels, rate, chunk) as audio_stream:
        while not rospy.is_shutdown():
            data = audio_stream.read(chunk)
            if not data:
                raise StopIteration()
            audio_publisher.publish(data)
