import numpy as np

def speedx(sound_array,factor):
    indices = np.round(np.arange(0,len(sound_array),factor))
    indices = indices[indices < len(sound_array)].astype(int)
    return sound_array[indices.astype(int)]

def stretch(sound_array,f,window_size,h):
    phase = np.zeros(window_size)
    hanning_window = np.hanning(window_size)
    result = np.zeros(len(sound_array)/f+ window_size)
    for i in np.arange(0,len(sound_array)-(window_size+h), h*f):
        i = int(i)
        window_size = int(window_size)
        a1 = sound_array[i:i+window_size]
        a2 = sound_array[i+h: i+window_size+h]

        s1 = np.fft.fft(hanning_window*a1)
        s2 = np.fft.fft(hanning_window*a2)
        phase = (phase+np.angle(s2/s1))%2*np.pi
        a2_rephased = np.fft.ifft(np.abs(s2)*np.exp(1j*phase))

        i2 = int(i/f)
        result[i2 : i2 + window_size] = (result[i2 : i2 + window_size] + hanning_window*a2_rephased).astype('float64')
    # print ((2**(12))* result/result.max())
    result =  (2**14)* (result/result.max())
    return result.astype('int16')

#n is in semitones
def pitchshift(snd_array,n,window_size=2**13, h=2**11):
    factor = 2**(1.0 * n/12.0)
    stretched =stretch(snd_array, 1.0/factor, window_size, h)
    return speedx(stretched[window_size:], factor)