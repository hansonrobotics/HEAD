"""
@package myWave provides functionality for reading and writing WAV files

@copyright GNU Public License
@author written 2009-2011 by Christian Herbst (www.christian-herbst.org) 
@author Supported by the SOMACCA advanced ERC grant, University of Vienna, 
	Dept. of Cognitive Biology

@note
This program is free software; you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation; either version 3 of the License, or (at your option) any later 
version.
@par
This program is distributed in the hope that it will be useful, but WITHOUT 
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
@par
You should have received a copy of the GNU General Public License along with 
this program; if not, see <http://www.gnu.org/licenses/>.

"""

import gc
import struct
import wave

import numpy

gc.enable()

from libs import scipy as sciWav


###############################################################################

def readWaveFile(fileName, useRobustButSlowAlgorithm = True):
	"""
	load a WAV file
	@param fileName the name of the WAV file that needs to be loaded
	@useRobustButSlowAlgorithm if True, we'll use older code that is slower but more robust
		vis-a-vis custom-generated WAV files that might have issues with chunks
		in the WAV file structure
	@return a list containing
		- the number of channels
		- the number of frames per channel
		- the sampling frequency [Hz]
		- a list containing one or more numpy array(s) containing the frame data 
			for each channel
	"""
	
	if useRobustButSlowAlgorithm:
		f = wave.open(fileName, "rb")
		numFrames = f.getnframes()
		numChannels = f.getnchannels()
		fs = f.getframerate()
		dataTmp = f.readframes(numFrames * numChannels)
		sampleWidth = f.getsampwidth()
		#print numChannels, numFrames, fs, sampleWidth, len(dataTmp)
		format = ''
		if sampleWidth == 1:
			format = 'B'
		elif sampleWidth == 2:
			format = 'h'
		elif sampleWidth == 4:
			format = 'i'
		if sampleWidth <> 2:
			raise Exception("we only support 16 bit data")
		out = struct.unpack_from (("%d" % (numFrames * numChannels)) + format, dataTmp)
		data = []
		divisor = float(2 ** 15)
		for i in range(numChannels):
			data.append(numpy.zeros(numFrames))
		for i in range(numChannels):
			arrFrameIdx = range(numFrames) # explicit indexing and garbage collection
			for j in arrFrameIdx:
				data[i][j] = out[j * numChannels + i] / divisor
			del arrFrameIdx
		f.close()
		del dataTmp, out, f
		gc.collect()
		return [numChannels, numFrames, fs, data]
	
	fs, dataRaw = sciWav.read(fileName)
	n = len(dataRaw)
	numChannels = 1
	try: numChannels = dataRaw.shape[1]
	except: pass
	arrChannels = []
	for chIdx in range(numChannels):
		tmp = numpy.zeros(n)
		if numChannels == 1:
			tmp = dataRaw.astype(numpy.float32)
		else:
			tmp = dataRaw[0:, chIdx].astype(numpy.float32)
		tmp /= float(2**15)
		arrChannels.append(tmp)
		del tmp
	del dataRaw
	gc.collect()
	return [numChannels, n, fs, arrChannels]

###############################################################################
	
def writeWaveFile(data, fileName, SRate = 44100.0, normalize = False, \
		removeDcWhenNormalizing = True
	): 
	""" 
	write an array of floats to a 16 bit wave file 
	@param data a list of lists or numpy array containing the frame data
	@param fileName the output file name
	@param SRate the sampling frequency [Hz]
	@param normalize if the parameter normalize is set to True, the signal 
		will be normalized to the maximally possible value (i.e. 1). if no
		normalization is performed, and if the input signal has a maximum 
		absolute ampitude greater than 1 (i.e. if the output would be clipped),
		the function throws an error. 
	@param removeDcWhenNormalizing if we're normalizing, this determines whether
		we should remove the DC offset before doing so.
	@return nothing 
	"""
	
	if not type(data).__name__ in ['list', 'ndarray']:
		raise Exception("expected a list data type")
	numChannels = 1
	valMin, valMax = None, None
	dataTmp = None
	dataType = type(data[0]).__name__
	if dataType in ['list', 'ndarray']:
		numChannels = len(data)
		n = len(data[0])
		dataTmp = numpy.zeros((n, numChannels))
		for chIdx in range(numChannels):
			dataTmp2 = None
			dType2 = type(data[chIdx]).__name__
			if dType2 == 'ndarray':
				dataTmp2 = data[chIdx]
			elif dType2 == 'list':
				dataTmp2 = numpy.array(data[chIdx], dtype=float32)
			else:
				raise Exception("channel data is not a list or a numpy array")
			dataTmp[0:, chIdx] = dataTmp2
			del dataTmp2
	else:
		# this is a mono file
		# force creating a copy, to avoid scaling the original data...
		dataTmp = numpy.array(data) 
	
	# normalize
	if normalize:
		if removeDcWhenNormalizing:
			dataTmp -= dataTmp.mean()
		valMin = dataTmp.min()
		valMax = dataTmp.max()
		absMax = abs(valMin)
		if abs(valMax) > absMax: absMax = abs(valMax)
		dataTmp /= absMax * 1.000001
	
	# save
	#print dataTmp.dtype, dataTmp.shape
	dataTmp *= float(2**15 - 1)
	dataTmp2 = numpy.asarray(dataTmp, dtype=numpy.int16)
	sciWav.write(fileName, SRate, dataTmp2)
	del dataTmp, dataTmp2
	gc.collect()
	
###############################################################################
