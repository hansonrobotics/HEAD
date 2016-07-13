
"""
@package praatTextGrid Utility classes for reading and writing Praat TextGrids. 

This software supports both point and interval tiers, stored as either Praat
Text files or ShortText files.

When reading text grids:
@code
	fileName = 'path/file.TextGrid'
	textGrid = praatTextGrid.PraatTextGrid(0, 0)
	# arrTiers is an array of objects (either PraatIntervalTier or PraatPointTier)
	arrTiers = textGrid.readFromFile(fileName)
	for tier in arrTiers:
		print tier
		for i in range(tier.getSize()):
			if tier.getLabel(i) == 'sounding':
				interval = tier.get(i)
				print "\t", interval
@endcode

When reading a CSV file and saving results as an IntervalTier in an TextGrid:
@code 
# we assume that the CSV file has this structure for each row: 
# startOffset (seconds), endOffset (seconds), label (string), e.g.
# 1.2837, 2.8237, soundSegment
textGrid = praatTextGrid.PraatTextGrid()
intervalTier = praatTextGrid.PraatIntervalTier()
f = open(csvFileName, 'r')
for row in f:
	data = row.split(',')
		intervalTier.add(data[0], data[1], data[2])
textGrid.add(intervalTier)
textGrid.save(outputFileName)
@endcode

######################################################################

@copyright GNU Public License
@author written 2010 - 2014 by Christian Herbst (www.christian-herbst.org) 
@author Partially supported by the SOMACCA advanced ERC grant, 
	University of Vienna, Dept. of Cognitive Biology

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

######################################################################

"""
	
######################################################################
# constants
######################################################################

UNKNOWN_TIER = -1
POINT_TIER = 1
INTERVAL_TIER = 2

	
######################################################################

class PraatTextGrid:
	
	def __init__(self, tStart = 0, tEnd = 0):
		self.arrTiers = []
		self.tStart = tStart
		self.tEnd = tEnd
	
	def add(self, tier):
		self.arrTiers.append(tier)	
		
	def addEmptyTier(self, tStart, tEnd, label = '', tierType = INTERVAL_TIER):
		# use this function to add an empty tier and then immediately save 
		# the file, i.e. if you want to add data manually with praat
		tier = None
		self.tStart = tStart
		self.tEnd = tEnd
		if tierType == POINT_TIER:
			tier = PraatPointTier()
		elif tierType == INTERVAL_TIER:
			tier = PraatIntervalTier()
		else:
			raise Exception('invalid tier type specified')
		tier.setName(label)
		tier.add(self.tStart, self.tEnd, '')
		self.add(tier)	
		
	def save(self, fileName):
		f = open(fileName, 'w')
		f.write('File type = "ooTextFile"\n')
		f.write('Object class = "TextGrid"\n')
		f.write('\n')
		#valMin, valMax = self.getMinMax()
		#f.write('xmin = ' + str(valMin) + ' \n')
		#f.write('xmax = ' + str(valMax) + ' \n')
		f.write('xmin = ' + str(self.tStart) + ' \n')
		f.write('xmax = ' + str(self.tEnd) + ' \n')
		f.write('tiers? <exists> \n')
		numTiers = len(self.arrTiers)
		f.write('size = ' + str(numTiers) + ' \n')
		f.write('item []: \n')
		for tierIdx in range(numTiers):
			tier = self.arrTiers[tierIdx]
			f.write('    item [' + str(tierIdx + 1) + ']:\n')
			txt = None
			tierType = tier.getType()
			if tierType == POINT_TIER:
				txt = 'TextTier'
			elif tierType == INTERVAL_TIER:
				txt = 'IntervalTier'
			else:
				raise Exception("unknown tier type")
			f.write('        class = "' + txt + '" \n')
			f.write('        name = "' + str(tier.getName()) + '" \n')
			f.write('        xmin = ' + str(self.tStart) + ' \n')
			f.write('        xmax = ' + str(self.tEnd) + ' \n')
			tierSize = tier.getSize()
			if tierType == POINT_TIER:
				f.write('        points: size = ' + str(tierSize) + ' \n')
			elif tierType == INTERVAL_TIER:
				f.write('        intervals: size = ' + str(tierSize) + ' \n')
			for i in range(tierSize):
				if tierType == POINT_TIER:
					tOffset, label = tier.get(i)
					f.write('        points [' + str(i + 1) + ']:\n')
					f.write('            time = ' + str(tOffset) + '\n')
					f.write('            mark = "' + str(label) + '"\n')
				elif tierType == INTERVAL_TIER:
					tStart, tEnd, label = tier.get(i)
					if i >= tierSize - 1:
						if tEnd < self.tEnd: tEnd = self.tEnd
					f.write('        intervals [' + str(i + 1) + ']:\n')
					f.write('            xmin = ' + str(tStart) + '\n')
					f.write('            xmax = ' + str(tEnd) + '\n')
					f.write('            text = "' + str(label) + '"\n')
				
            
		f.close()
		
	def getMinMax(self):
		valMin = None
		valMax = None
		for tierIdx in range(len(self.arrTiers)):
			tier = self.arrTiers[tierIdx]
			if tierIdx == 0:
				valMin = tier.getXMin()
				valMax = tier.getXMax()
			else:
				tmp = tier.getXMin()
				if tmp < valMin: valMin = tmp
				tmp = tier.getXMax()
				if tmp > valMax: valMax = tmp
		return valMin, valMax
				
		
	def readFromFile(self, fileName):
		#import codecs
		#f = codecs.open(fileName, 'r', encoding='utf-16')
		f = open(fileName, 'r')
		cnt = 0
		errMsg = 'PraatTextGrid::readFromFile(' + fileName + ') - '
		isShortTextFile = False
		xMin = -1
		xMax = -1
		numTiers = 0
		offset = 7
		tierIdx = 0
		boundaryIdx = 0
		t = 0
		t2 = 0 # used if parsing an interval tier (boundary end time)
		tierSize = 0
		insideTier = False
		tierName = ''
		self.arrTiers = []
		tierStart = 0
		tierEnd = 0
		tierNumLines = 0
		tierType = UNKNOWN_TIER
		for line in f:
			txt = line.strip()
			cnt += 1
			errMsg = 'readPraatAbstractTier(' + fileName + ' - line ' + str(cnt) + ' [' + txt + ']) - '
			#print cnt, tierIdx, txt
			if cnt == 1:
				if txt <> 'File type = "ooTextFile"':
					raise Exception(errMsg + 'not a Praat TextGrid')
			elif cnt == 2:
				if txt <> 'Object class = "TextGrid"':
					raise Exception(errMsg + 'not a Praat TextGrid')
			elif cnt == 4:
				if txt[0:7] == 'xmin = ':
					isShortTextFile = False
					xMin = float(txt.split('=')[1].strip())
				else:
					isShortTextFile = True
					tierNumLines = 2
					xMin = float(txt)
				self.tStart = xMin
			elif cnt == 5:
				if isShortTextFile:
					xMax = float(txt)
				else:
					xMax = float(txt.split('=')[1].strip())
				self.tEnd = xMax
			elif cnt == 7:
				if isShortTextFile:
					numTiers = int(txt)
				else:
					numTiers = int(txt.split('=')[1].strip())
			elif isShortTextFile and cnt > 7:
				if cnt > 7:
					if tierIdx == 0:
						boundaryIdx = 0
						if txt == '"TextTier"':
							tierType = POINT_TIER
							self.arrTiers.append(PraatPointTier())
							tierNumLines = 2
						elif txt == '"IntervalTier"':
							tierType = INTERVAL_TIER
							self.arrTiers.append(PraatIntervalTier())
							tierNumLines = 3
						else:
							raise Exception(errMsg + 'invalid file structure: tier type not supported')
					elif tierIdx == 1:
						tierName = txt[1:-1]
						self.arrTiers[-1].setName(tierName)
					elif tierIdx == 2:
						tierStart = float(txt)
					elif tierIdx == 3:
						tierEnd == float(txt)
					elif tierIdx == 4:
						tierSize = int(txt)
					else:
						if (boundaryIdx % tierNumLines) == 0:
							t = float(txt)
						elif tierType == INTERVAL_TIER \
								and boundaryIdx % tierNumLines == 1:
							t2 = float(txt)
						else:
							if tierType == POINT_TIER:
								self.arrTiers[-1].add(t, txt.strip("'"))
							elif tierType == INTERVAL_TIER:
								self.arrTiers[-1].add(t, t2, txt.strip("'"))
						boundaryIdx += 1
						if boundaryIdx / tierNumLines >= tierSize:
							tierIdx = -1
							#print "tierIdx = -1"
							boundaryIdx = 0
					tierIdx += 1
			else:
				if cnt > 8:
					if tierIdx == 0:
						boundaryIdx = 0
					elif tierIdx == 1:
						tmp = txt.split('=')[1].strip()	
						if tmp == '"TextTier"':
							self.arrTiers.append(PraatPointTier())
							tierType = POINT_TIER
							tierNumLines = 3
						elif tmp == '"IntervalTier"':
							self.arrTiers.append(PraatIntervalTier())
							tierType = INTERVAL_TIER
							tierNumLines = 4
						else:
							raise Exception(errMsg \
								+ 'invalid file structure: tier type not supported')
					elif tierIdx == 2:
						tierName = txt.split('=')[1].strip()[1:-1]
						self.arrTiers[-1].setName(tierName)
					elif tierIdx == 3:
						tierStart = float(txt.split('=')[1].strip())
					elif tierIdx == 4:
						tierEnd = float(txt.split('=')[1].strip())
					elif tierIdx == 5:
						tierSize = int(txt.split('=')[1].strip())
						if tierSize == 0:
							tierIdx = -1
					else:
						#print cnt, tierIdx, boundaryIdx, tierType, txt
						if boundaryIdx % tierNumLines == 0:
							pass
						elif boundaryIdx % tierNumLines == 1:
							#print cnt, txt
							t = float(txt.split('=')[1].strip())
						elif tierType == INTERVAL_TIER \
								and boundaryIdx % tierNumLines == 2:
							t2 = float(txt.split('=')[1].strip())
						else:
							if tierType == POINT_TIER:
								self.arrTiers[-1].add(t, txt.split('=')[1].strip().strip("'"))
							elif tierType == INTERVAL_TIER:
								self.arrTiers[-1].add(t, t2, txt.split('=')[1].strip().strip("'"))
						boundaryIdx += 1
						if int(boundaryIdx / tierNumLines) >= tierSize:
							tierIdx = -1
							#print "tierIdx = -1"
							boundaryIdx = 0
					tierIdx += 1
						
				
		f.close()
			
		return self.arrTiers	
		
######################################################################

class PraatAbstractTier:

	""" abstract base class """
	
	_name = ''
	_arrLabel = []
	_tierType = UNKNOWN_TIER

	def __init__(self):
		self._tierType = UNKNOWN_TIER
		self.clear()
		
	def _clear(self):
		self._name = ''
		self._arrLabel = []
		
	def __str__(self):
		txt = ''
		if self._tierType == POINT_TIER:
			txt = 'Text Tier'
		elif self._tierType == INTERVAL_TIER:
			txt = 'Interval Tier'
		else:
			txt = 'Unknown Tier'
		txt += ': ' + self._name + ' (' + str(self.getSize()) + ' elements)'
		return txt	
		
	def _setTierType(self, tierType):
		self._tierType = tierType
		
	def setName(self, name):
		self._name = name
		
	def getName(self):
		return self._name
		
	def getSize(self):
		return len(self._arrLabel)
		
	def getType(self):
		return self._tierType
		
	def checkIndex(self, idx):
		if idx < 0 or idx >= self.getSize():
			raise Exception('idx ' + str(idx) + ' is out of range')
		return True
		
	
######################################################################

class PraatIntervalTier(PraatAbstractTier):

	def __init__(self, name = ''):
		self._clear()
		self._setTierType(INTERVAL_TIER)
		self.setName(name)
		
	def _clear(self):
		self._arrStartOffset = []
		self._arrEndOffset = []
		PraatAbstractTier._clear(self)
	
	def add(self, startOffset, endOffset, label):
		self._arrStartOffset.append(startOffset)
		self._arrEndOffset.append(endOffset)
		if label <> '':
			if label[0] == '"' and label[-1] == '"':
				label = label[1:-1]
		self._arrLabel.append(label)
		
	def get(self, idx):
		self.checkIndex(idx)
		return self._arrStartOffset[idx], self._arrEndOffset[idx], \
			self._arrLabel[idx]
		
	def getLabel(self, idx):
		self.checkIndex(idx)
		return self._arrLabel[idx]
		
	def getXMin(self):
		valMin = None
		for i in range(len(self._arrStartOffset)):
			if i == 0:
				valMin = self._arrStartOffset[i]
			else:
				if self._arrStartOffset[i] < valMin:
					valMin = self._arrStartOffset[i]
		return valMin
		
	def getXMax(self):
		valMax = None
		for i in range(len(self._arrEndOffset)):
			if i == 0:
				valMax = self._arrEndOffset[i]
			else:
				if self._arrEndOffset[i] > valMax:
					valMax = self._arrEndOffset[i]
		return valMax

######################################################################

class PraatPointTier(PraatAbstractTier):

	def __init__(self, name = ''):
		self._clear()
		self._setTierType(POINT_TIER)
		self.setName(name)
		
	def _clear(self):
		self._arrOffset = []
		PraatAbstractTier._clear(self)
	
	def add(self, offset, label):
		if label[0] == '"' and label[-1] == '"':
			label = label[1:-1]
		self._arrOffset.append(offset)
		self._arrLabel.append(label)
		
	def get(self, idx):
		self.checkIndex(idx)
		return self._arrOffset[idx], self._arrLabel[idx]
		
	def getXMin(self):
		valMin = None
		for i in range(len(self._arrOffset)):
			if i == 0:
				valMin = self._arrOffset[i]
			else:
				if self._arrOffset[i] < valMin:
					valMin = self._arrOffset[i]
		return valMin
		
	def getXMax(self):
		valMax = None
		for i in range(len(self._arrOffset)):
			if i == 0:
				valMax = self._arrOffset[i]
			else:
				if self._arrOffset[i] > valMax:
					valMax = self._arrOffset[i]
		return valMax
		
	def getLabel(self, idx):
		self.checkIndex(idx)
		return self._arrLabel[idx]
	
######################################################################
	
def main():
	arrFiles = [
		'/Users/ch/data/research/univie/excisedLarynxLab/elephant/analysis/syncSignals/418_1_stereo.TextGrid',
		'/Users/ch/data/research/univie/excisedLarynxLab/elephant/analysis/syncSignals/442_1_stereo.TextGrid',
	]
	for fileName in arrFiles:
		textGrid = PraatTextGrid()
		arrTiers = textGrid.readFromFile(fileName)
		print '\n', fileName	
		for tier in arrTiers:
			print tier
			for i in range(tier.getSize()):
				print "\t", tier.get(i)
	print "\ndone.\n"
	
######################################################################
	
if __name__ == "__main__":
   main()
	
	
	