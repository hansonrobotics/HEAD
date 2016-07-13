
"""

@package praatUtil This module contains some utility functions to seamlessly
	incorporate Praat analysis functionality into Python

@copyright GNU Public License
@author written 2009-2014 by Christian Herbst (www.christian-herbst.org) 
@author Partially supported by the SOMACCA advanced ERC grant, University of Vienna, 
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

import praatTextGrid

import dspUtil
import generalUtility
import math
import myWave
import numpy
import os

PRAAT_SHORT_TEXT_FILE = 1
PRAAT_TEXT_FILE = 0

def convertToVector(dataX, dataY, duration, timeStep, valMin, valMax):
	"""
	@deprecated this function is obsolete
	"""
	data = []
	size = int(round(duration / float(timeStep)))
	for i in range(size):
		data.append(None)
	for i in range(len(dataX)):
		t = dataX[i]
		idx = int(t / timeStep)
		if dataY[i] >= valMin and dataY[i] <= valMax:
			data[idx] = dataY[i]
	return data

######################################################################

def readHarmonicityData(fileName):
	"""
	reads Praat Harmonicity data, saved as "short text file" within Praat
	@param fileName
	@return a tuple containing two lists: the time offset, and the 
		corresponding HNR data
	"""
	dataX, dataY, metaData = readPraatShortTextFile(fileName, 'Harmonicity 2')
	return dataX, dataY

######################################################################

def readIntensityTier(fileName):
	"""
	reads Praat Intensity data, saved as "short text file" within Praat
	@param fileName
	@return a tuple containing two lists: the time offset, and the 
		corresponding Intensity data
	"""
	dataX, dataY, metaData = readPraatShortTextFile(fileName, 'Intensity')
	return dataX, dataY

######################################################################

def readPitchTier(fileName):
	"""
	reads Praat PitchTier data, saved as "short text file" within Praat
	@param fileName
	@return a tuple containing two lists: the time offset, and the 
		corresponding F0 (inaccurately called "pitch" in Praat) data
	"""
	dataX, dataY, metaData = readPraatShortTextFile(fileName, 'PitchTier')
	return dataX, dataY

######################################################################

def readLtas(fileName):
	""" 
	read long-term average spectrum (LTAS) data, as saved in Praat as a 
	short text file
	@param fileName
	@return a tuple containing the sampling frequency [Hz], the analysis
		bandwidth, and a numpy array containing the energy in the individual 
		LTAS bins
	"""
	
	fs = None
	bandwidth = None
	f = open(fileName, 'r')
	arrData = []
	for i, line in enumerate(f):
		line = line.strip()
		#print i, line
		if i == 0:
			if line != 'File type = "ooTextFile"':
				raise Exception("not a LTAS short text file")
		if i == 1:
			if line != 'Object class = "Ltas 2"':
				raise Exception("not a LTAS short text file")
		if i == 4:
			fs = float(line)
		if i == 6:
			bandwidth = float(line)
		if i > 12:
			arrData.append(float(line))
	return fs, bandwidth, numpy.array(arrData)
	
######################################################################

def readSpectrum(fileName, convertToDb = True, fileType = PRAAT_TEXT_FILE):
	""" 
	this function reads a Praat spectrum file (saved as a 'text file')
	@param fileName
	@param convertToDb is True, the RMS data is converted to dB
	@param fileType the type of the Praat Spectrum file: either "text file" 
		(@ref PRAAT_TEXT_FILE, i.e., the default case) or "short text file"
		(@ref PRAAT_SHORT_TEXT_FILE, i.e., an experimental implementation)
	@return a tuple containing the sampling rate, the window size, and
		a one-dimensional array of floats 
	@todo the PRAAT_SHORT_TEXT_FILE support is experimental and needs to be 
		both debugged and tested thoroughly
	"""
	
	def parseLine(line, fileType):
		tmp = line.split('=')[-1]
		tmp2 = tmp.split('e')
		val = 0
		if len(tmp2) == 1: val = float(tmp2[0])
		elif len(tmp2) == 2:
			val = float(tmp2[0]) * math.pow(10.0, float(tmp2[1]))
		else:
			raise ("unable to read data value")
		if fileType == PRAAT_SHORT_TEXT_FILE:
			return val # float(line.strip())
		elif fileType == PRAAT_TEXT_FILE:
			return val # float(line.split('=')[1])
		else:
			raise Exception("Praat file type not recignized")

	if fileType == PRAAT_SHORT_TEXT_FILE:
		dataX, dataY, metaData = readPraatShortTextFile(fileName, 'Spectrum 2')
		fs = float(metaData[1])
		windowSize = float(metaData[2])

	file = open(fileName, "r")
	cnt = 0
	offset = 0
	data = None # real and imaginary data
	fs = None
	windowSize = None
	idx = 0 # only used for short text file

	for line in file:
		line = line.strip()
		cnt += 1
		#print cnt, line # debug information
		
		if cnt == 1:
			# 1 File type = "ooTextFile"
			if line <> 'File type = "ooTextFile"':
				raise Exception ("file " + fileName \
					+ " is not a Praat short text file")
				
		elif cnt == 2:
			# 2 Object class = "Spectrum 2"
			if line <> 'Object class = "Spectrum 2"':
				raise Exception ("file " + fileName \
					+ " is not a Praat spectrum file")
		
		elif cnt == 5:
			# 5 xmax = 22050
			#try:
				fs = parseLine(line, fileType) * 2.0
			#except:
			#	raise Exception('unable to determine sampling frequency')
	
		elif cnt == 6:
			# window size is incremented by 1 in the Praat short text file
			# 6 nx = 4097 		
			#try:
				windowSize = parseLine(line, fileType)
				data = numpy.zeros((2, windowSize))
			#except:
			#	raise Exception('unable to determine windowSize')
		
		elif (fileType == PRAAT_SHORT_TEXT_FILE and cnt > 13) \
				or (fileType == PRAAT_TEXT_FILE and cnt > 15):
			try:
				if fileType == PRAAT_SHORT_TEXT_FILE:
					idx1 = int(idx / windowSize)
					idx2 = int(idx % windowSize)
					val = parseLine(line, fileType)
					data[idx1][idx2] = val
					#print idx1, idx2, val
					idx += 1
				elif fileType == PRAAT_SHORT_TEXT_FILE:
					if line <> 'z [2]:':
						val = float(line.split('=')[1])
						tmp = line.split('=')[0].split('[')
						idx1 = int(tmp[1].strip().strip(']')) - 1
						idx2 = int(tmp[2].strip().strip(']')) - 1
						#print idx1, idx2, val
						#z [1] [1] = -0.00026495086386186666
						data[idx1][idx2] = val
			except:
				raise Exception('unable to read data value in line ' \
					+ str(cnt) + ' ("' + line + '")')
				
	file.close()	
		
	# some basic testing
	if len(data[0]) <> len(data[1]):
		raise Exception('ERROR when reading Praat spectrum file ' + fileName \
			+ ': size of real and imaginary data arrays does not match')
	if len(data[0]) <> windowSize:
		raise Exception ('ERROR when reading Praat spectrum file ' + fileName \
			+ ': expected ' + str(windowSize) + ' data points, but read ' \
			+ len(data))	
			
	# convert to power spectrum
	dataOut = numpy.zeros(windowSize)
	for i in range(len(data[0])):
		val = math.sqrt(data[0][i] * data[0][i] + data[1][i] * data[1][i])
		# convert to dB
		if convertToDb:
			if val > 0:
				val = dspUtil.rmsToDb(val)
			else:
				val = -300
		dataOut[i] = val 		
	
	return fs, windowSize, dataOut
	
######################################################################

def generateEmptyPraatTextGrid(fName, arrIntervalTierNames, 
		arrPointTierNames = None, overwriteExistingTextGrid = False,
		duration = None):
	"""
	generates an empty Praat TextGrid for the specified WAV file
	@param fName the name of the WAV file. MUST contain the full path information
	@param arrIntervalTierNames a list of names for interval tiers to be created.
		If None, no interval tiers will be generated
	@param arrPointTierNames a list of names for point tiers to be created.
		If None, no point tiers will be generated
	@param overwriteExistingTextGrid if False, an error is thrown if the 
		target TextGrid file already exists
	@param duration duration of the WAV file, given in seconds. if None, we'll 
		have to open the WAV file to determine the duration
	"""
	path, fileNameOnly, suffix = generalUtility.splitFullFileName(fName)
	outputFileName = path + fileNameOnly + '.TextGrid'
	if not overwriteExistingTextGrid:
		if os.path.isfile(outputFileName):
			raise Exception("TextGrid %s already exists" % outputFileName)
	if duration is None:
		numChannels, numFrames, fs, data = myWave.readWaveFile(\
			fName, useRobustButSlowAlgorithm = False)
		duration = float(numFrames) / float(fs)
	
	textGrid = praatTextGrid.PraatTextGrid(0, duration)
	if not arrIntervalTierNames is None:
		for label in arrIntervalTierNames:
			intervalTier = praatTextGrid.PraatIntervalTier(label)
			intervalTier.add(0, duration, "")
			textGrid.add(intervalTier)
	if not arrPointTierNames is None:
		for label in arrPointTierNames:
			pointTier = praatTextGrid.PraatPointTier(label)
			textGrid.add(pointTier)
	textGrid.save(outputFileName)

######################################################################

def extractSegments(wavFileName, label = None, tierIndex = 0, 
		textGridFileName = None, outputPath = None
	):
	"""
	extract segments from an annotated WAV file. we'll store the extracted 
		segments as WAV files, appending an underscore and a four-digit 
		extraction index (starting at 1) to the file file (e.g., 
		"ORIGINALWAVFILENAME_0001.wav")
	@param wavFileName the name of the WAV file. MUST include the full path
	@param label the label of the segment that we should extract. if None, 
		we'll extract all non-empty segments
	@param tierIndex the index (zero-based) of the IntervalTier that contains
		the annotation
	@param textGridFileName the name of the accompanying text grid. if None, 
		we'll assume that the TextGrid file has the same name as the WAV file,
		but with the suffix TextGrid
	@param outputPath if None, we'll store the segmented files in the path
		where the original WAV file is stored
	@return a list of tuples with the start/end offset [seconds] of the segment
	"""
	
	path, fileNameOnly, suffix = generalUtility.splitFullFileName(wavFileName)
	if textGridFileName is None:
		textGridFileName = "%s%s.TextGrid" % (path, fileNameOnly)
	if outputPath is None:
		outputPath = path
	numChannels, numFrames, fs, channelData = myWave.readWaveFile(wavFileName)
	textGrid = praatTextGrid.PraatTextGrid(0, 0)
	arrTiers = textGrid.readFromFile(textGridFileName)
	tier = arrTiers[tierIndex]
	cnt = 0
	arrSegmentInfo = []
	for i in range(tier.getSize()):
		doExtract = False
		if label is None:
			if tier.getLabel(i).strip() != '':
				doExtract = True
		else:
			if tier.getLabel(i).strip() != label:
				doExtract = True
		if doExtract:
			cnt += 1
			interval = tier.get(i)
			tStart = interval[0]
			tEnd = interval[1]
			offset1 = int(round(tStart * float(fs)))
			offset2 = int(round(tEnd * float(fs)))
			arrDataTmp = []
			for chIdx in range(numChannels):
				arrDataTmp.append(channelData[chIdx][offset1:offset2])
			outputFileName = "%s%s_%04d.wav" % (outputPath, fileNameOnly, cnt)
			myWave.writeWaveFile(arrDataTmp, outputFileName, SRate = fs, 
				normalize = False, removeDcWhenNormalizing = True)
			arrSegmentInfo.append((tStart, tEnd))
	return arrSegmentInfo

######################################################################

def readPraatShortTextFile(fileName, obj):
	""" 
	this function reads a Praat pitch tier file (saved as a 'short text file')
	@param fileName
	@param obj the file type. Currently we support these file types (as defined
		internally by Praat):
			- Harmonicity 2
			- PitchTier
			- Intensity
			- SpectrumTier
			- Spectrum 2
			- Cepstrum 1
	@return a two-dimensional array of floats, the first row 
		(index = 0) representing the time offsets of data values, and the 
		second row representing the detected fundamental frequency values 
	"""
	file = open(fileName, "r")
	cnt = 0
	numDataPoints = 0
	offset = 0
	dataX = []
	dataY = []
	dataIdx = 0
	timeStep = 0
	timeOffset = 0
	
	arrFileTypes = [
		'Harmonicity 2', 'PitchTier', 'Intensity', 'SpectrumTier', \
			'Spectrum 2', 'Cepstrum 1'
	]
	
	if not obj in arrFileTypes:
		raise Exception('readPraatShortTextFile - file type must be: ' 
			+ ', '.join(arrFileTypes))
	metaData = []
	for line in file:
		line = line.strip()
		cnt += 1
		#print cnt, line # debug information
		if cnt > 6:
			if obj == 'Harmonicity 2' or obj == 'Intensity 2':
				if cnt > 13:
					val = float(line)
					if val > -100:
						dataY.append(val)
					else:
						dataY.append(None)
					dataX.append(timeOffset + float(dataIdx) * timeStep)
					dataIdx += 1
				else:
					if cnt == 7:
						timeStep = float(line)
					if cnt == 8:
						timeOffset = float(line)
			else:
			# read data here
				if cnt % 2 == 0:
					dataY.append(float(line))
					dataIdx += 1
				else:
					dataX.append(float(line))
		else:
			if cnt > 3:
				metaData.append(line)
			# error checking and loop initialization
			if cnt == 1:
				if line != "File type = \"ooTextFile\"":
					raise Exception ("file " + fileName \
						+ " is not a Praat pitch" + " tier file")
			if cnt == 2:
				err = False
				#print line 
				if obj == 'Harmonicity':
					if line != "Object class = \"Harmonicity\"" \
							and line != "Object class = \"Harmonicity 2\"":
						err = True
				elif obj == 'Intensity':
					if line != "Object class = \"IntensityTier\"" \
							and line != "Object class = \"Intensity 2\"":
						err = True
				else:
					if line != "Object class = \"" + obj + "\"":
						err = True
				if err == True:
					raise Exception ("file " + fileName + " is not a Praat " 	
						+ obj + " file")
			if cnt == 6:
				if line[0:15] == 'points: size = ':
					numDataPoints = int(line.split('=')[1].strip())
					raise Exception (\
						"only the 'short text file' type is supported. " \
						+ " Save your Praat " + obj \
						+ " with 'Write to short text file.") 
				else:
					numDataPoints = int(line)
	return (numpy.array(dataX), numpy.array(dataY), metaData)

######################################################################

class PraatFormants:

	"""
	a class to store/process Praat formants
	"""

	DO_DEBUG = False
	
	# ---------------------------------------------------------------------- #

	def __init__(self):
		self.clear()
	
	# ---------------------------------------------------------------------- #
	
	def clear(self):
		"""
		resets the object's state 
		"""
		self.xmin = None
		self.xmax = None
		self.nx = None
		self.dx = None
		self.x1 = None
		self.arrX = []
		self.arrData = []
	
	# ---------------------------------------------------------------------- #
		
	def getNumFrames(self):
		"""
		@return the number of frames
		"""
		return self.nx
	
	# ---------------------------------------------------------------------- #
	
	def get(self, idx):
		"""
		@param idx 
		@return a tuple containing the time offset and the formant data at 
			that particular temporal offset. the formant data is a list of
			dictionaries (one for each formant), the latter contaning a 
			'bandwidth' and a 'frequency' parameter (both indicated in Hz)
		"""
		if idx < 0 or idx >= self.nx:
			raise Exception("index out of range")
		return self.arrX[idx], self.arrData[idx]
	
	# ---------------------------------------------------------------------- #
	
	def decodeParam(self, txt, param, line = -1, fileName = ''):
		"""
		internally used ("pseudo-private") function used for reading Praat
		Formant files 
		@param txt the text (i.e., line from a file) that is being parsed.
			must have the structure 'paramName = paramValue'
		@param line only used for reporting errors (in case an error 
			actually arises during parsing txt)
		@param fileName only used for reporting errors (in case an error 
			actually arises during parsing txt)
		@return a floating point value
		"""
		data = txt.split('=')
		errMsg = ''
		if fileName <> '':
			errMsg = ' of file "' + fileName + '"'
		if line > 0:
			errMsg = ' in line ' + str(line) + errMsg
		if len(data) <> 2:
			raise Exception('cannot decode text "' + txt \
					+ '" - invalid structure' + errMsg)
		if data[0].strip() <> param:
			raise Exception('expected parameter "' + param \
				+ '" but found "' + data[0].strip() + '"' + errMsg)
		return float(data[1])
	
	# ---------------------------------------------------------------------- #
	
	def readFile(self, fileName):
		"""
		@todo bug when opening a "long text file"
		@todo refactor this code, it's ugly to look at (too many if 
			statements and indentations)
		"""
		f = open(fileName)
		cnt = 0
		maxnFormants = None
		isShortTextFile = False
		insideDataStructure = False
		dataCnt = 0
		intensity = None
		nFormants = None
		frequency = None
		bandwidth = None
		frameIdx = 0
		arrFormants = []
		self.clear()
		
		for line in f:			
			cnt += 1
			errMsg = ' in line ' + str(cnt) + ' of file "' + fileName + '"'
			txt = line.strip()
			#print str(cnt) + ' _' + txt + '_'
			if cnt == 1:
				if txt <> 'File type = "ooTextFile"':
					raise Exception('expected \'File type = "ooTextFile"\'' \
						+ errMsg)
			elif cnt == 2:
				if txt <> 'Object class = "Formant 2"':
					raise Exception('expected \'Object class = "Formant 2"\'' \
						+ errMsg)
			elif cnt == 4: # xmin
				if len(txt.split('=')) > 1:
					isShortTextFile = False
					if txt.split('=')[0].strip() <> 'xmin':
						raise Exception('invalid file structure.' + errMsg)
					self.xmin = self.decodeParam(txt, 'xmin', cnt, fileName)
				else:
					isShortTextFile = True
					self.xmin = float(txt)
			elif cnt == 5: # xmax
				if isShortTextFile: self.xmax = float(txt)
				else: self.xmax = self.decodeParam(txt, 'xmax', cnt, fileName)
			elif cnt == 6: # nx
				if isShortTextFile: self.nx = int(txt)
				else: self.nx = int(self.decodeParam(txt, 'nx', cnt, fileName))
			elif cnt == 7: # dx
				if isShortTextFile: self.dx = float(txt)
				else: self.dx = self.decodeParam(txt, 'dx', cnt, fileName)
			elif cnt == 8: # x1
				if isShortTextFile: self.x1 = float(txt)
				else: self.x1 = self.decodeParam(txt, 'x1', cnt, fileName)
			elif cnt == 9: # maxnFormants
				if isShortTextFile: self.maxnFormants = float(txt)
				else: self.maxnFormants = self.decodeParam(txt, \
					'maxnFormants', cnt, fileName)
			elif cnt > 9:
				if isShortTextFile:
				
					# -------------------------------------------------- #
					# short text file
					# -------------------------------------------------- #
					
					#print cnt, txt
					if insideDataStructure:
						dataCnt += 1
						if dataCnt == 1:
							nFormants = int(txt)
							if self.DO_DEBUG: print "\t\tnFormants:", \
								nFormants
						else:
							tmpCnt = dataCnt - 2
							formantCount = tmpCnt / 2 + 1
							if tmpCnt % 2 == 0:
								frequency = float(txt)
								if self.DO_DEBUG: 
									print "\t\tformant:", formantCount
								if self.DO_DEBUG: 
									print "\t\t\tfrequency:", frequency
							else:
								bandwidth = float(txt)
								if self.DO_DEBUG: 
									print "\t\t\tbandwidth:", bandwidth
								arrFormants.append({
									'frequency':frequency,
									'bandwidth':bandwidth,
								})
								if formantCount == nFormants:
									# add the data here
									x = self.x1 + self.dx * (frameIdx - 1)
									self.arrX.append(x)
									self.arrData.append(arrFormants)
									insideDataStructure = False
					else:
						dataCnt = 0
						insideDataStructure = True
						arrFormants = []
						intensity = float(txt)
						frameIdx += 1
						if self.DO_DEBUG: 
							print "\tframeIdx:", frameIdx
							print "\t\tintensity:", intensity
						
				else:
				
					# -------------------------------------------------- #
					# long text file
					# -------------------------------------------------- #
					if cnt == 10:
						if txt <> 'frame []:':
							raise Exception('invalid file structure' + errMsg)
					else:
						if insideDataStructure:
							dataCnt += 1
							#if self.DO_DEBUG: print "\t\t\t\t\t", cnt, dataCnt, txt
							if dataCnt == 1:
								intensity = self.decodeParam(txt, \
									'intensity', cnt, fileName)
							elif dataCnt == 2:
								nFormants = int(self.decodeParam(txt, \
									'nFormants', cnt, fileName))
							elif dataCnt == 3:
								if txt <> 'formant []:':
									raise Exception('invalid file structure' \
										+ errMsg)
							else:
								tmpCnt = (dataCnt - 4)
								formantCount = tmpCnt / 3 + 1
								if tmpCnt % 3 == 0:
									if txt[0:9] <> 'formant [':
										raise Exception('invalid file structure' \
											+ errMsg)
									formantIdx = int(txt.split('[')[1].split(']')[0])
									if self.DO_DEBUG: 
										print "\t\tformant:", formantIdx
									if formantIdx <> formantCount:
										raise Exception('invalid file structure' \
											+ errMsg)
								elif tmpCnt % 3 == 1:
									frequency = self.decodeParam(txt, \
										'frequency', cnt, fileName)
									if self.DO_DEBUG: 
										print "\t\t\tfrequency:", frequency
								elif tmpCnt % 3 == 2:
									bandwidth = self.decodeParam(txt, \
										'bandwidth', cnt, fileName)
									if self.DO_DEBUG: 
										print "\t\t\tbandwidth:", bandwidth
									arrFormants.append({
										'frequency':frequency,
										'bandwidth':bandwidth,
									})
									if formantCount == nFormants:
										# add the data here
										x = self.x1 + self.dx * (frameIdx - 1)
										self.arrX.append(x)
										self.arrData.append(arrFormants)
										insideDataStructure = False
									
						else:
							dataCnt = 0
							insideDataStructure = True
							arrFormants = []
							if txt[0:7] <> 'frame [':
								raise Exception('invalid file structure' \
									+ errMsg)
							frameIdx = int(txt.split('[')[1].split(']')[0])
							if self.DO_DEBUG: print "\tframeIdx:", frameIdx
					
		f.close()
		
		# check the data
		if len(self.arrX) <> len(self.arrData):
			raise Exception("data array sizes don't match!")
		if self.nx <> len(self.arrX):
			raise Exception('file "' + fileName + '" promised to contain ' + \
				str(self.nx) + ' frames, but only ' + str(len(self.arrX)) + \
				' were found')

######################################################################
				
def readPraatFormantData(fileName):
	"""
	@deprecated This function is obsolete. Keep it for now for backwards
		compatibility, but remove it soon.
	"""
    #print fileName
	formants = PraatFormants()
	formants.clear()
	#arrX = []
	#arrData = []
	f = open(fileName, "r")
	cnt = 0
	for line in f:
		line = line.strip()
		if line <> '':
			cnt += 1
			if cnt > 1:
				arrTmp = []
				data = line.split('\t')
				if len(data) > 2:
					t = data[0]
					try:
						if cnt == 2:
							formants.x1 = float(t)
						nformants = data[1]
						for i in range(2, len(data)):
							arrTmp.append({
								'frequency':float(data[i]),
								'bandwidth':1,
							})
						formants.arrX.append(float(t))
						formants.arrData.append(arrTmp)
					except Exception as e:
						print e
						print t, data[i]
						cnt -= 1
				else:
					cnt -= 1
	formants.nx = len(formants.arrX)
	return formants	

######################################################################

def changeSamplingRate(
		waveFileName, 
		newSamplingRate, 
		outputFileName = '', 
		sincWidth = 200, 
		normalize = False
	):
	"""
	utility function to change a wave file's sampling frequency
	@param waveFileName name of the input file
	@param newSamplingRate Hz
	@param outputFileName (include a path!) if empty string, the existing 
		file is overwritten
	@param sincWidth increase for better accuracy and less speed (Praat's default is 50)
	@param normalize In some cases (when the input file is already 
		normalized to +-1) resampling can result in a clipped output file.
		In these cases it is useful to normalize the generated file.
	@todo refactor so that this function uses the new @ref runPraatScript(...) 
		function
	@todo add tmpDataPath parameter
	"""
	fileNameOnly = '.'.join(waveFileName.split('/')[-1].split('.')[:-1])
	inputPath = '/'.join(waveFileName.split('/')[:-1]) + '/'
	praatControlFileName = inputPath + 'tmp.praat'
	f = open(praatControlFileName, 'w')
	f.write('Read from file... ' + waveFileName + '\n')
	f.write('Resample... ' + str(newSamplingRate) + ' ' + str(sincWidth) + '\n')
	if outputFileName == '': outputFileName = waveFileName
	if normalize: f.write('Scale peak... 0.99\n')
	f.write('Write to WAV file... ' + outputFileName + '\n')
	f.close()
	args = ['Praat', praatControlFileName]
	print "\tcalling Praat to resample data"
	msg = generalUtility.makeSystemCall(args)	
	return msg
	
######################################################################

def changeSamplingRateOfSignal(
		data, 
		oldSamplingFrequency, 
		newSamplingFrequency, 
		tmpPath = None,
		tmpFileName = 'tmp.wav',
		sincWidth = 200, 
		normalize = False
	):
	"""
	change the sampling frequency of a given signal. similar to @ref 
	changeSamplingRate(), but does not operate on a saved sound file, but on a 
	signal in memory.
	@param data a numpy array or a list of floats
	@param oldSamplingFrequency [Hz]
	@param newSamplingFrequency [Hz]
	@param tmpDataPath the path for temporary files. if None, see @ref 
		runPraatScript
	@param tmpFileName temporary file name. DO NOT PROVIDE A PATH
	@param sincWidth see @ref changeSamplingRate
	@param normalize see @ref changeSamplingRate
	@return a numpy array containing the resampled input data
	"""
	
	if tmpPath is None:
		tmpPath = generalUtility.getUserTmpDir()
	tmpFileName = tmpPath + tmpFileName
	myWave.writeWaveFile(data, tmpFileName, SRate = oldSamplingFrequency, \
		normalize = False)
	changeSamplingRate(tmpFileName, newSamplingFrequency, sincWidth = sincWidth)
	numChannels, numFrames, fs, data = myWave.readWaveFile(tmpFileName)
	if normalize:
		return dspUtil.normalize(data[0], 0.999, -0.999)
	return data[0]

######################################################################
		
def calculateF0(
		waveFileName, 
		readProgress = 0.01, 
		acFreqMin = 60, 
		voicingThreshold = 0.45, 
		veryAccurate = False, 
		fMax = 2000, 
		octaveJumpCost = 0.35
):
	"""
	Utility function to calculate the time-varying fundamental frequency of
	the specified wave file using Praat's 
	<a href="http://www.fon.hum.uva.nl/praat/manual/Sound__To_Pitch__ac____.html">
	To Pitch (ac)...</a> method
	@param waveFileName input file name 
	@param readProgress see Praat's manual
	@param acFreqMin see Praat's manual
	@param voicingThreshold see Praat's manual
	@param veryAccurate see Praat's manual
	@param fMax see Praat's manual
	@param octaveJumpCost see Praat's manual
	@todo refactor so that this function uses the new @ref runPraatScript(...) 
		function
	@todo add tmpDataPath parameter
	"""
	fileNameOnly = '.'.join(waveFileName.split('/')[-1].split('.')[:-1])
	inputPath = '/'.join(waveFileName.split('/')[:-1]) + '/'
	praatControlFileName = inputPath + 'tmp.praat'
	f = open(praatControlFileName, 'w')
	f.write('Read from file... ' + waveFileName + '\n')
	txtAccurate = 'no'
	if veryAccurate: txtAccurate = 'yes'
	f.write('To Pitch (ac)...  ' + str(readProgress) + ' ' + str(acFreqMin) \
		+ ' 15 ' + txtAccurate + ' 0.03 ' + str(voicingThreshold) + ' 0.01 ' \
		+ str(octaveJumpCost) + ' 0.14 ' + str(fMax) + '\n')
	f.write('Down to PitchTier\n')
	pitchTierFileName = inputPath + fileNameOnly + '.PitchTier'
	f.write('Write to short text file... ' + pitchTierFileName + '\n')
	f.close()
	args = ['Praat', praatControlFileName]
	print "\tcalling Praat to calculate F0"
	msg = generalUtility.makeSystemCall(args)	
	return msg

######################################################################

def calculateF0OfSignal(
		data, 
		samplingFrequency, 
		tmpDataPath = None,
		readProgress = 0.01, 
		acFreqMin = 60, 
		voicingThreshold = 0.45, 
		veryAccurate = False, 
		fMax = 2000, 
		octaveJumpCost = 0.35,
		tStart = None, 
		tEnd = None
	):
	"""
	calcuate the fundamental frequency (F0) of a given signal. similar to 
	@ref calculateF0(), but does not operate on a saved sound file, but on a 
	signal in memory.

	@param data a numpy array or a list of floats
	@param samplingFrequency sampling frequency of the input signal [Hz]
	@param tmpDataPath the path for temporary files. if None, see @ref 
		runPraatScript
	@param readProgress see Praat's manual
	@param acFreqMin see Praat's manual
	@param voicingThreshold see Praat's manual
	@param veryAccurate see Praat's manual
	@param fMax see Praat's manual
	@param octaveJumpCost see Praat's manual
	@param tStart if not None, this parameter specifies the start time of the 
		segment of the file that should be analyzed
	@param tEnd if not None, this parameter specifies the end time of the 
		segment of the file that should be analyzed
	@return a tuple containing two numpy arrays: one containing the temporal 
		offsets of the HNR measurements, and one containing the respective
		HNR values
	"""
	
	if tmpDataPath is None:
		tmpDataPath = generalUtility.getUserTmpDir()
	tmpFileName = tmpDataPath + 'tmp.wav'
	n = len(data)
	idx1 = 0
	idx2 = n
	if not tStart is None and not tEnd is None:
		idx1 = int(round(tStart * float(fs)))
		idx2 = int(round(tEnd * float(fs)))
		if idx1 < 0: idx1 = 0
		if idx2 >= n: idx2 = n
	if dspUtil.getAbsMax(data) >= 1:
		data -= data.min()
		data /= data.max()
		data -= 0.5
		data *= 1.99
	myWave.writeWaveFile(data[idx1:idx2], tmpFileName, SRate = samplingFrequency, normalize = True)
	
	calculateF0(tmpFileName, readProgress = readProgress, acFreqMin = acFreqMin, voicingThreshold = voicingThreshold, veryAccurate = veryAccurate, fMax = fMax, octaveJumpCost = octaveJumpCost)
	dataX, dataY = readPitchTier(tmpDataPath + 'tmp.PitchTier')
	os.remove(tmpDataPath + 'tmp.PitchTier')
	return dataX, dataY

######################################################################
		
def calculateHNR(
		waveFileName, 
		readProgress = 0.01, 
		acFreqMin = 60, 
		voicingThreshold = 0.1, 
		numPeriodsPerWindow = 4.5, 
		tStart = None, 
		tEnd = None, 
		outputFileName = None
	):
	"""
	Utility function to calculate the time-varying harmonics-to-noise ratio
	(HNR) of the specified wave file using Praat's 
	<a href="http://www.fon.hum.uva.nl/praat/manual/Sound__To_Harmonicity__ac____.html">
	To Harmonicity (ac)...</a> method. The result is stored as a Praat "short
	text file".
	@param waveFileName input file name 
	@param readProgress see Praat's manual
	@param acFreqMin see Praat's manual
	@param voicingThreshold see Praat's manual
	@param numPeriodsPerWindow see Praat's manual
	@param tStart if not None, this parameter specifies the start time of the 
		segment of the file that should be analyzed
	@param tEnd if not None, this parameter specifies the end time of the 
		segment of the file that should be analyzed
	@param name of the Praat Harmonicity file that is being generated. If 
		None the file name of the input file (minus the suffix) plus 
		.Harmonicity is used
	@todo refactor so that this function uses the new @ref runPraatScript(...) 
		function
	@todo add tmpDataPath parameter
	@return a tuple containing two numpy arrays: one containing the temporal 
		offsets of the HNR measurements, and one containing the respective
		HNR values
	"""
	fileNameOnly = '.'.join(waveFileName.split('/')[-1].split('.')[:-1])
	inputPath = '/'.join(waveFileName.split('/')[:-1]) + '/'
	praatControlFileName = inputPath + 'tmp.praat'
	f = open(praatControlFileName, 'w')
	f.write('Read from file... ' + waveFileName + '\n')
	if (not tStart is None) and (not tEnd is None):
		f.write('Extract part... %f %f rectangular 1 no\n' % (float(tStart), \
			float(tEnd)))
	#f.write('To Harmonicity (ac)... %f %f %f %f\n' % (float(readProgress), \
	#	float(acFreqMin), float(voicingThreshold), float(numPeriodsPerWindow)))
	f.write("do (\"To Harmonicity (ac)...\", %f, %f, %f, %f)\n" % (float(readProgress), \
		float(acFreqMin), float(voicingThreshold), float(numPeriodsPerWindow)))
	if outputFileName is None:
		outputFileName = inputPath + fileNameOnly + '.Harmonicity'
	f.write('Write to short text file... ' + outputFileName + '\n')
	f.write('Remove All\n')
	f.close()
	args = ['Praat', praatControlFileName]
	print "\tcalling Praat to calculate HNR"
	msg = generalUtility.makeSystemCall(args)	
	dataX, dataY = readHarmonicityData(outputFileName)
	return dataX, dataY
    
######################################################################

def calculateHNROfSignal(
		data, 
		samplingFrequency, 
		tmpDataPath = None,
		readProgress = 0.01, 
		acFreqMin = 60, 
		voicingThreshold = 0.1, 
		numPeriodsPerWindow = 4.5, 
		tStart = None, 
		tEnd = None
	):
	"""
	calcuate the harmonics-to-noise ratio (HNR) of a given signal. similar to 
	@ref calculateHNR(), but does not operate on a saved sound file, but on a 
	signal in memory.

	@param data a numpy array or a list of floats
	@param samplingFrequency sampling frequency of the input signal [Hz]
	@param tmpDataPath the path for temporary files. if None, see @ref 
		runPraatScript
	@param readProgress time interval by which the analysis window is advanced 
	@param acFreqMin see Praat's manual
	@param voicingThreshold see Praat's manual
	@param numPeriodsPerWindow see Praat's manual
	@param tStart if not None, this parameter specifies the start time of the 
		segment of the file that should be analyzed
	@param tEnd if not None, this parameter specifies the end time of the 
		segment of the file that should be analyzed
	@return a tuple containing two numpy arrays: one containing the temporal 
		offsets of the HNR measurements, and one containing the respective
		HNR values
	"""
	
	if tmpDataPath is None:
		tmpDataPath = generalUtility.getUserTmpDir()
	tmpFileName = tmpDataPath + 'tmp.wav'
	if dspUtil.getAbsMax(data) >= 1:
		data -= data.min()
		data /= data.max()
		data -= 0.5
		data *= 1.99
	myWave.writeWaveFile(data, tmpFileName, SRate = samplingFrequency, normalize = False)
	calculateHNR(tmpFileName, readProgress = readProgress, acFreqMin = acFreqMin, voicingThreshold = voicingThreshold, numPeriodsPerWindow = numPeriodsPerWindow, tStart = tStart, tEnd = tEnd)
	dataX, dataY = readHarmonicityData(tmpDataPath + 'tmp.Harmonicity')
	return dataX, dataY

######################################################################

def calculateIntensity(
		inputFileName, 
		fMin = 100,
		timeStep = 0,
		subtractMean = True,
		tmpDataPath = None,
		keepIntensityTierFile = False,
		keepPraatScriptFile = False
	):
	"""
	call Praat's 
	<a href="http://www.fon.hum.uva.nl/praat/manual/Sound__To_Intensity___.html">
	To Intensity...</a> function to calculate the specified file's
	intensity.
	@param inputFileName the name of the input file. needs to have a full path
		name if we should keep the IntensityTier file
	@param fMin [Hz] - see Praat's manual
	@param timeStep [s] - see Praat's manual
	@param subtractMean see Praat's manual
	@param tmpDataPath the path for temporary files. if None, see @ref runPraatScript
	@param keepIntensityTierFile if False, we'll remove the generated 
		IntensityTier file and just return the Intensity data
	@param keepPraatScriptFile if False, we'll remove the temporary Praat
		script file
	@return temporal and intensity information as returned by 
		@ref readIntensityTier()
	"""
	if tmpDataPath is None:
		tmpDataPath = generalUtility.getUserTmpDir()
	path, fileNameOnly, suffix = generalUtility.splitFullFileName(inputFileName)
	sSubtractMean = 'no'
	if subtractMean: sSubtractMean = 'yes'
	intensityTierFileName = path + fileNameOnly + '.IntensityTier'
	if not keepIntensityTierFile:
		intensityTierFileName = path + fileNameOnly + '.IntensityTier'
	script = ''
	script += "Read from file... %s\n" % inputFileName
	#script += "selectObject: \"Sound tmp\"\n"
	script += "To Intensity: %f, %f, \"%s\"\n" % (fMin, timeStep, sSubtractMean)
	script += "Down to IntensityTier\n"
	script += "Save as short text file... %s\n" % intensityTierFileName
	scriptFileName = 'tmp.praat'
	
	runPraatScript(script, scriptFileName, keepPraatScriptFile, tmpDataPath)
	dataT, dataI = readIntensityTier(intensityTierFileName)
	if not keepIntensityTierFile:
		os.remove(intensityTierFileName)
	return dataT, dataI

######################################################################

def calculateIntensityOfSignal(
		data, 
		samplingFrequency, 
		fMin = 100,
		timeStep = 0,
		subtractMean = True,
		tmpDataPath = None,
		keepIntensityTierFile = False,
		keepPraatScriptFile = False,
		tStart = None, 
		tEnd = None
	):
	"""
	calcuate the intensity of a given signal. similar to 
	@ref calculateIntensity(), but does not operate on a saved sound file, but on a 
	signal in memory.

	@param data a numpy array or a list of floats
	@param samplingFrequency sampling frequency of the input signal [Hz]
	@param fMin [Hz] - see Praat's manual
	@param timeStep [s] - see Praat's manual
	@param subtractMean see Praat's manual
	@param tmpDataPath the path for temporary files. if None, see @ref runPraatScript
	@param keepIntensityTierFile if False, we'll remove the generated 
		IntensityTier file and just return the Intensity data
	@param keepPraatScriptFile if False, we'll remove the temporary Praat
		script file
	@param tStart if not None, this parameter specifies the start time of the 
		segment of the file that should be analyzed
	@param tEnd if not None, this parameter specifies the end time of the 
		segment of the file that should be analyzed
	@return a tuple containing two numpy arrays: one containing the temporal 
		offsets of the HNR measurements, and one containing the respective
		HNR values
	"""
	
	if tmpDataPath is None:
		tmpDataPath = generalUtility.getUserTmpDir()
	tmpFileName = tmpDataPath + 'tmp.wav'
	if dspUtil.getAbsMax(data) >= 1:
		raise Exception("abs max > 1")
	n = len(data)
	idx1 = 0
	idx2 = n
	if not tStart is None and not tEnd is None:
		idx1 = int(round(tStart * float(fs)))
		idx2 = int(round(tEnd * float(fs)))
		if idx1 < 0: idx1 = 0
		if idx2 >= n: idx2 = n
	myWave.writeWaveFile(data[idx1:idx2], tmpFileName, SRate = samplingFrequency, normalize = False)
	dataX, dataY = calculateIntensity(tmpFileName, fMin = fMin, timeStep = timeStep, subtractMean = subtractMean, tmpDataPath = None, keepIntensityTierFile = keepIntensityTierFile, keepPraatScriptFile = keepPraatScriptFile)
	return dataX, dataY

######################################################################
		
def calculateSpectrum(waveFileName, fast = False):
	"""
	Utility function to calculate the overall spectrum of
	the specified wave file using Praat's 
	<a href="http://www.fon.hum.uva.nl/praat/manual/Sound__To_Spectrum___.html">
	To Spectrum...</a> method. The result is stored as a Praat "short text
	file".
	@param waveFileName input file name 
	@param fast see Praat's manual
	@todo refactor so that this function uses the new @ref runPraatScript(...) 
		function
	@todo add tmpDataPath parameter
	"""
	fileNameOnly = '.'.join(waveFileName.split('/')[-1].split('.')[:-1])
	inputPath = '/'.join(waveFileName.split('/')[:-1]) + '/'
	praatControlFileName = inputPath + 'tmp.praat'
	f = open(praatControlFileName, 'w')
	f.write('Read from file... ' + waveFileName + '\n')
	yesNo = 'yes'
	if not fast: yesNo = 'no'
	f.write('To Spectrum... ' + yesNo + '\n')
	spectrumFileName = inputPath + fileNameOnly + '.Spectrum'
	f.write('Write to short text file... ' + spectrumFileName + '\n')
	f.close()
	args = ['Praat', praatControlFileName]
	print "\tcalling Praat to calculate spectrum"
	msg = generalUtility.makeSystemCall(args)	

	fs, windowSize, dataOut = readSpectrum(spectrumFileName, True, \
		fileType = PRAAT_SHORT_TEXT_FILE)
	n = len(dataOut)
	spectrumX = numpy.zeros(n)
	spectrumY = numpy.zeros(n)
	binWidth = (fs / 2) / float(n)
	for i in range(n):
		spectrumX[i] = i * float(binWidth)
		spectrumY[i] = dataOut[i]
	return spectrumX, spectrumY
	
######################################################################

def calculateFormants(
	waveFileName, 
	maxNumFormants = 5, 
	fMax = 5000, # [Hz]
	windowLength = 	0.025, # [s]
	preEmphasis = 50, # [Hz]
	timeStep = 0, # [s], 0 is auto-selection in Praat
	lpcSamplingFrequency = None, # [Hz]; if not None, we'll calculate the LPC and save it
	arrSpectralSlices = None, # either None or an array of time offsets where spectral slices of the LPC are calculated
	spectralSliceFrequencyResolution = 20, # [Hz]
	spectralSliceBandwidthReduction = 0, # [Hz]
	spectralSliceDeEmphasisFrequency = 50, # [Hz]
):
	"""
	Utility function to calculate the formants of the specified wave file 
	using Praat's 
	<a href="http://www.fon.hum.uva.nl/praat/manual/Sound__To_Formant__burg____.html">
	To Formant (burg)...</a> method. The result is stored as a Praat 
	"short text file".
	@param waveFileName input file name 
	@param maxNumFormants see Praat's manual
	@param fMax see Praat's manual
	@param windowLength see Praat's manual
	@param preEmphasis see Praat's manual
	@param timeStep see Praat's manual
	@param lpcSamplingFrequency if not None, we'll calculate the LPC and save 
		it
	@param arrSpectralSlices either None or an array of time offsets where 
		spectral slices of the LPC are calculated and stored as Praat "short 
			text" files
	@param spectralSliceFrequencyResolution [Hz] - only used if the parameter
		arrSpectralSlices is not None
	@param spectralSliceBandwidthReduction [Hz] - only used if the parameter
		arrSpectralSlices is not None
	@param spectralSliceDeEmphasisFrequency [Hz] - only used if the parameter
		arrSpectralSlices is not None
	@return a tuple containing: formant data as returned by the @ref readFile()
		function of the @ref PraatFormants class; an array containing the 
		center frequencies of the spectral slice bins; and a list containing
		the spectral data values of the calculated spectral slices. If 
		arrSpectralSlices is None, the last two return values are set to None.
	@todo refactor so that this function uses the new @ref runPraatScript(...) 
		function
	@todo add tmpDataPath parameter
	"""
	# To Formant (burg)... 0 5 5000 0.025 50
	# To LPC... 16000
	# To Spectrum (slice)... 1 20 0 50
	# select LPC aeiou
	# Write to short text file... /Users/ch/data/research/sataloff_textbook/figures/data/aeiou.LPC
	# select Spectrum aeiou
	# Write to short text file... /Users/ch/data/research/sataloff_textbook/figures/data/aeiou.Spectrum
	fileNameOnly = '.'.join(waveFileName.split('/')[-1].split('.')[:-1])
	inputPath = '/'.join(waveFileName.split('/')[:-1]) + '/'
	praatControlFileName = inputPath + 'tmp.praat'
	f = open(praatControlFileName, 'w')
	f.write('Read from file... ' + waveFileName + '\n')
	cmd = 'To Formant (burg)... ' + str(timeStep) + ' ' + str(maxNumFormants) \
		+ ' ' + str(fMax) + ' ' + str(windowLength) + ' ' + str(preEmphasis)
	f.write(cmd + '\n')
	formantDataFileName = inputPath + fileNameOnly + '.Formant'
	f.write('Write to short text file... ' + formantDataFileName + '\n')
	
	if not lpcSamplingFrequency is None:
		f.write('To LPC... ' + str(lpcSamplingFrequency) + '\n')
		lpcFileName = inputPath + fileNameOnly + '.LPC'
		f.write('Write to short text file... ' + lpcFileName + '\n')
		if not arrSpectralSlices is None:
			for t in arrSpectralSlices:
				f.write('select LPC ' + fileNameOnly + '\n')
				f.write('To Spectrum (slice)... ' + str(t) + ' ' \
					+ str(spectralSliceFrequencyResolution) + ' ' \
					+ str(spectralSliceBandwidthReduction) + ' ' \
					+ str(spectralSliceDeEmphasisFrequency) + '\n')
				spectrumFileName = inputPath + fileNameOnly + '_' \
				+ str(t) + '.Spectrum'
				f.write('Write to short text file... ' + spectrumFileName \
					+ '\n')	
	
	f.close()
	args = ['Praat', praatControlFileName]
	print "\tcalling Praat to calculate formants"
	msg = generalUtility.makeSystemCall(args)	
	
	#formants = readPraatFormantData(formantDataFileName)
	formants = PraatFormants()
	formants.readFile(formantDataFileName)


	arrSpectrum = []
	fs = None
	windowSize = None
	if not lpcSamplingFrequency is None:
		#lpcFileName = inputPath + fileNameOnly + '.LPC'
		if not arrSpectralSlices is None:
			if len(arrSpectralSlices) > 0:
				for t in arrSpectralSlices:
					spectrumFileName = inputPath + fileNameOnly + '_' \
					+ str(t) + '.Spectrum'
					fs, windowSize, dataOut = readSpectrum(spectrumFileName, \
						convertToDb = True, fileType = PRAAT_SHORT_TEXT_FILE)
					arrSpectrum.append(dataOut)
				n = len(arrSpectrum[0])
				spectrumX = numpy.zeros(n)
				binWidth = (fs / 2) / float(n)
				for i in range(n):
					spectrumX[i] = i * float(binWidth)
				return formants, spectrumX, arrSpectrum
			
	# if no spectral slices are to be extracted, return only the formant data
	return formants, None, None
	
######################################################################

def applyBandPassFilterToSignal(
		signal, # a numpy vector
		fs, # sampling frequency 
		lowerFreqBoundary, # [Hz]
		upperFreqBoundary, # [Hz] 
		smoothing, # [Hz]
		preservePhase = True,
		tmpDataPath = None,
		tmpFileName = 'tmp.wav',
		keepPraatScriptFile = False
):
	"""
	Apply a bandpass filter to the specified sound, using Praat's 
	<a href="http://www.fon.hum.uva.nl/praat/manual/Sound__Filter__pass_Hann_band____.html">
	Filter (pass Hann band)...</a> function. The result will be stored in the same
	directory as the filtered file, and the extension '_band' is appended
	to the file name (just as in Praat). 
	@param signal a numpy vector containing the sound data
	@param fs sampling frequency [Hz]
	@param lowerFreqBoundary [Hz] - see Praat's manual
	@param upperFreqBoundary [Hz] - see Praat's manual
	@param smoothing [Hz] - see Praat's manual
	@param preservePhase if true, we'll filter twice in a row, the second
		treating the time-inverted signal
	@param tmpDataPath the path for temporary files. if None, see @ref runPraatScript
	@param tmpFileName the name of the temporary file
	@param keepPraatScriptFile = False
	@return the filtered data
	@todo add tmpDataPath parameter
	"""
	
	if tmpDataPath is None:
		tmpDataPath = generalUtility.getUserTmpDir()
	fileName = tmpDataPath + tmpFileName
	myWave.writeWaveFile(signal, fileName, SRate = fs, normalize = False)
	
	newFileName = applyBandPassFilter(fileName, lowerFreqBoundary, \
		upperFreqBoundary, smoothing, keepPraatScriptFile = keepPraatScriptFile)
	numChannels, numFrames, fs, data = myWave.readWaveFile(newFileName)
	
	if preservePhase == False:
		return data[0]
		
		
	n = len(data[0])
	dataTmp = numpy.zeros(n)
	for i in range(n):
		dataTmp[i] = data[0][n - (i + 1)]
	myWave.writeWaveFile(dataTmp, fileName, SRate = fs, normalize = False)
	newFileName = applyBandPassFilter(fileName, lowerFreqBoundary, upperFreqBoundary, smoothing, keepPraatScriptFile = keepPraatScriptFile)
	numChannels, numFrames, fs, data = myWave.readWaveFile(newFileName)
	n = len(data[0])
	dataTmp = numpy.zeros(n)
	for i in range(n):
		dataTmp[i] = data[0][n - (i + 1)]
	return dataTmp
		
	
######################################################################

def applyBandPassFilter(
		fileName, # path and file name 
		lowerFreqBoundary, # [Hz]
		upperFreqBoundary, # [Hz] 
		smoothing, # [Hz]
		tmpDataPath = None,
		keepPraatScriptFile = False
):
	"""
	Apply a bandpass filter to the specified sound file, using Praat's 
	<a href="http://www.fon.hum.uva.nl/praat/manual/Sound__Filter__pass_Hann_band____.html">
	Filter (pass Hann band)...</a> function. The result will be stored in the same
	directory as the filtered file, and the extension '_band' is appended
	to the file name (just as in Praat). 
	@param fileName path and file name 
	@param lowerFreqBoundary [Hz] - see Praat's manual
	@param upperFreqBoundary [Hz] - see Praat's manual
	@param smoothing [Hz] - see Praat's manual
	@param tmpDataPath the path for temporary files. if None, see @ref runPraatScript
	@param keepPraatScriptFile = False
	@return the name of the newly written file
	@todo refactor so that this function uses the new @ref runPraatScript(...) 
		function
	@todo add tmpDataPath parameter
	"""


	if tmpDataPath is None:
		tmpDataPath = generalUtility.getUserTmpDir()
	fileNameOnly = '.'.join(fileName.split('/')[-1].split('.')[:-1])
	inputPath = '/'.join(fileName.split('/')[:-1]) + '/'
	praatControlFileName = 'tmp.praat'
	script = ''
	script += 'Read from file... ' + fileName + '\n'
	script += 'Filter (pass Hann band)... ' + str(lowerFreqBoundary) + ' ' \
		+ str(upperFreqBoundary) + ' ' + str(smoothing) + '\n'
	newFileName = inputPath + fileNameOnly + '_band.wav'
	script += 'Write to WAV file... ' + newFileName + '\n'
	
	runPraatScript(script, scriptFileName = praatControlFileName, \
		keepPraatScriptFile = keepPraatScriptFile, tmpDataPath = tmpDataPath)

		
	return newFileName

##############################################################################

def extractSegmentsObsolete(
		inputFileName, 
		outputPath,
		textGridFileName = None,
		tierNumber = 1,
		label = None,
		tmpDataPath = None,
		keepPraatScriptFile = False
	):
	"""
	create and execute a temporary Praat script that extracts segments from
	the specified sound file that was previously annotated with PraatTextGrids.
	The generated segment files will have the extraction time offset encoded
	in their file name.
	
	@param inputFileName sound file that should be analyzed (need to specify a 
		full path name). Note that we also need a corresponding TextGrid file
	@param outputPath path where the file segments should be written to. If 	
		None, we'll take the input path
	@param textGridFileName if None, we'll take the sound file name and 
		substitute the suffix with 'TextGrid'
	@param tierNumber  the tier number in the praat TextGrid file. 1 is default
		(note that Praat starts to count with 1 and not with 0). Useful if your
		TextGrid file contains multiple tiers
	@param label if None, we'll extract any annotated segment where the 
		annotation text is not an empty string. If the label is specified (as
		a string), we'll look for exactly that annotation
	@param inputPath
	@param tmpDataPath the path for temporary files. if None, see @ref runPraatScript
	@param keepPraatScriptFile if False, we'll remove the temporary Praat
		script file
	@return a list with the file names of the created files

	
	###########################################################################
	path$ = "/Users/ch/data/research/univie/excisedlarynxLab/deers/audioData/recordings/F4/"
	outputPath$ = "/Users/ch/data/research/univie/excisedlarynxLab/deers/audioData/segments/F4/"
	do ("Read from file...", "'path$'" + "700_audio.wav")
	selectedSound$ = selected$("Sound")
	do ("Read from file...", "/Users/ch/data/research/univie/excisedlarynxLab/deers/audioData/recordings/F4/700_audio.TextGrid")
	select Sound 'selectedSound$'
	plus TextGrid 'selectedSound$'
	do ("Extract intervals where...", 1, "yes", "is equal to", "data")
	n = numberOfSelected ("Sound")
	for i to n
		sound'i' = selected("Sound", i)
	endfor
	for i to n
		select sound'i' 
		starttime = Get start time
		filename$ = "'outputPath$'" + "'selectedSound$'" + "_" + "'starttime'" + "s.wav"
		printline 'filename$' 'tab$' 'i' 'tab$' 'starttime:4'
		do ("Save as WAV file...", "'filename$'")
		fileappend 'outputPath$''selectedSound$'_log.txt 'filename$''newline$'
	endfor
	###########################################################################
	"""
	
	if tmpDataPath is None:
		tmpDataPath = generalUtility.getUserTmpDir()
	inputPath, fileNameOnly, suffix = \
		generalUtility.splitFullFileName(inputFileName)

	textGridSelection = "'selectedSound$'"
	if textGridFileName is None:
		textGridFileName = inputPath + fileNameOnly + ".TextGrid"
	else:
		dummy1, textGridFileNameOnly, dummy2 = \
			generalUtility.splitFullFileName(textGridFileName)
		textGridSelection = textGridFileNameOnly

	script = ""
	script += "path$ = \"" + inputPath + "\"\n"
	script += "outputPath$ = \"" + outputPath + "\"\n"
	script += "do (\"Read from file...\", \"'path$'\" + \"" + fileNameOnly \
		+ "." + suffix + "\")\n"
	script += "selectedSound$ = selected$(\"Sound\")\n"
	script += "do (\"Read from file...\", \"" + textGridFileName + "\")\n"
	script += "select Sound 'selectedSound$'\n"
	script += "plus TextGrid " + textGridSelection + "\n"
	if label:
		script += "do (\"Extract intervals where...\", " + str(tierNumber) \
			+ ", \"yes\", \"is equal to\", \"" + label + "\")\n"
	else:
		script += "do (\"Extract non-empty intervals...\", " + str(tierNumber) \
			+ ", \"yes\")\n"
	script += "n = numberOfSelected (\"Sound\")\n"
	script += "for i to n\n"
	script += "	sound'i' = selected(\"Sound\", i)\n"
	script += "endfor\n"
	script += "for i to n\n"
	script += "	select sound'i' \n"
	script += "	starttime = Get start time\n"
	script += "	filename$ = \"'outputPath$'\" + \"'selectedSound$'\" + \"_\" \
		+ \"'starttime'\" + \"s.wav\"\n"
	script += "	printline 'filename$' 'tab$' 'i' 'tab$' 'starttime'\n"
	script += "	do (\"Save as WAV file...\", \"'filename$'\")\n"
	script += "	fileappend 'outputPath$''selectedSound$'_log.txt 'filename$''newline$'\n"
	script += "endfor\n"
	scriptFileName = 'tmp.praat'
	runPraatScript(script, scriptFileName, keepPraatScriptFile, tmpDataPath)

##############################################################################

def runPraatScript(
		script, 
		scriptFileName = 'tmp.praat', 
		keepPraatScriptFile = False,
		tmpDataPath = None
):
	"""
	write the specified Praat script to a (temporary) script file and execute
	that script within Praat by making a system call. In order for this to 
	work properly, Praat must be installed and available at the command line.

	@param script a valid Praat script. lines are separated by newline 
		characters (backslash n)
	@param scriptFileName the name of the Praat script file that
		should be executed. DO NOT SUPPLY A PATH HERE!
	@param keepPraatScriptFile if False, the temporary script file will be
		deleted after execution
	@param tmpDataPath where the temporary script file is saved. If None, Python
		will look for the current user's path and append 'tmp' (and if that 
		resulting path is not found, it will be created automatically)
	@throw throws an error if the script execution fails
	@return the time (milliseconds) it took to execute the Praat script
	"""
	if tmpDataPath is None:
		tmpDataPath = generalUtility.getUserTmpDir()
	f = open(tmpDataPath + scriptFileName, 'w')
	f.write(script)
	f.close()
	args = ['Praat', tmpDataPath + scriptFileName]
	now = os.times()[4]
	msg = generalUtility.makeSystemCall(args)	
	if msg[1] != '':
		raise Exception("Error executing Praat script: " + str(msg[1]))
	if not keepPraatScriptFile:
		os.remove(tmpDataPath + scriptFileName)
	return os.times()[4] - now

######################################################################

def getPerturbationMeasures(
		fName, 
		tmpDataPath = None,
		keepAnalysisFile = False
	):
	"""
	performes perturbation measures using Praat's default settings
	@param fName the name of the WAV file that should be analyzed
	@param tmpDataPath the path for temporary files. if None, see @ref runPraatScript
	@param keepAnalysisFile is True, we will not automatically delete the 
		output file generated by the Praat script
	@returns a dict with Praat's measures
	"""
	
	inputPath, fileNameOnly, suffix = generalUtility.splitFullFileName(fName)
	
	if tmpDataPath is None:
		tmpDataPath = generalUtility.getUserTmpDir()
		
	outputFile = tmpDataPath + fileNameOnly + ".txt"
	if os.path.isfile(outputFile):
		os.remove(outputFile)
	
	script = ""
	script += "do (\"Read from file...\", \"" + fName + "\")\n"
	script += "do (\"To Pitch (ac)...\", 0, 75, 15, \"no\", 0.03, 0.45, 0.01, 0.35, 0.14, 600)\n"
	script += "do (\"To PointProcess\")\n"
	script += "mean = do (\"Get mean period...\", 0, 0, 0.0001, 0.02, 1.3)\n"
	script += "stdev = do (\"Get stdev period...\", 0, 0, 0.0001, 0.02, 1.3)\n"
	script += "j1 = do (\"Get jitter (local)...\", 0, 0, 0.0001, 0.02, 1.3)\n"
	script += "j2 = do (\"Get jitter (local, absolute)...\", 0, 0, 0.0001, 0.02, 1.3)\n"
	script += "j3 = do (\"Get jitter (rap)...\", 0, 0, 0.0001, 0.02, 1.3)\n"
	script += "j4 = do (\"Get jitter (ppq5)...\", 0, 0, 0.0001, 0.02, 1.3)\n"
	script += "j5 = do (\"Get jitter (ddp)...\", 0, 0, 0.0001, 0.02, 1.3)\n"
	
	script += "select Sound " + fileNameOnly.replace(".", "_") + "\n"
	script += "do (\"To Harmonicity (cc)...\", 0.01, 75, 0.1, 1)\n"
	script += "hnr = do (\"Get mean...\", 0, 0)\n"

	script += "\n"
	script += "sep$ = \", \"\n"
	script += "txt$ = string$(mean) + sep$ + string$(stdev) + sep$ + string$(j1) + sep$ + string$(j2) + sep$ + string$(j3) + sep$ + string$(j4) + sep$ + string$(j5) + sep$ + string$(hnr) + newline$\n"
	script += "fileappend \"" + outputFile + "\" 'txt$'\n"
	
	runPraatScript(
		script, 
		scriptFileName = 'tmp.praat', 
		keepPraatScriptFile = False,
		tmpDataPath = tmpDataPath
	)
	
	arrData = []
	f = open(outputFile, 'r')
	for idx, line in enumerate(f):
		if idx == 0:
			tmp = line.split(',')
			for val in tmp:
					arrData.append(float(val))
				
	if not keepAnalysisFile:
		os.remove(outputFile)
		
	return {
		'mean': arrData[0],
		'stddev': arrData[1],
		'jitterLocal': arrData[2],
		'jitterAbsolute': arrData[3],
		'rap': arrData[4],
		'ppq5': arrData[5],
		'ddp': arrData[6],
		'hnr': arrData[7],
	}

######################################################################
