__author__ = 'masresha'

from pyo import *

s = Server(audio='pa', nchnls=2).boot()
s.start()
file= "Voice-samples/amy.wav"
filenameRecord = 'Voice-samples/eric500Shifted.wav'
sf = SfPlayer(file, speed=1, loop=False)
# lf1 = Sine(freq=.04, mul=10)
# lf1= 508.355
# lf2 = Sine(freq=200, mul= 5,add=1)
lf2 = 185.27436
# b = FreqShift(sf, shift=lf2 , mul=2).out()
s.recstart(filenameRecord)
b = FreqShift(sf, shift=100 , mul=2).out()
time.sleep(10)
s.recstop()
s.gui()
# fr = Sine(.2, 0, 8000, 8000)
# boo = Sine([4, 4], 1, -12)
# out = EQ(b, freq=fr, q=1, boost=boo, type=2).out() #filtering
# out = EQ(b, freq=8000, q=1, boost=-12, type=2)

# freqs = [.254, .465, .657, .879, 1.23, 1.342, 1.654, 1.879]
#  # delay line center delays
# # @@ -19,8 +22,11 @@
# adelay = [.001, .0012, .0013, .0014, .0015, .0016, .002, .0023]
# # modulation depth
# cdelay = [.0087, .0102, .0111, .01254, .0134, .01501, .01707, .0178]
# lfos = Sine(freqs, mul=adelay, add=cdelay)
# delays = Delay(b, lfos, maxdelay=.01)
#
# pit = Randi(min=0.99, max=1.01, freq=100)

# s.gui()
