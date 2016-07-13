
"""

@package matplotlibUtil A module for creating beautiful graphs

@copyright GNU Public License
@author written 2013, 2014 by Christian Herbst (www.christian-herbst.org) 
@author Sponsored by the Dept. of Cognitive Biology, University of Vienna, 
	
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


import numpy
import matplotlib
from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
import matplotlib.cm as cm
import generalUtility
import dspUtil


NUMERATOR_TYPE_ALPHA_LOWER = 0
NUMERATOR_TYPE_ALPHA_UPPER = 1
NUMERATOR_TYPE_ARABIC = 2	
NUMERATOR_TYPE_ROMAN = 3	

################################################################################

class CGraph:
	"""
	the main class, acting as a container for the graph
	"""
	def __init__(self,
		width = 8,
		height = 8,
		dpi = 72,
		lineWidth = 2,
		padding = 0.06,
		fontSize = 16,
		fontFamily = 'serif',
		fontFace = 'Times New Roman',
		fontWeight = 'normal',
		numeratorType = NUMERATOR_TYPE_ALPHA_UPPER
	
	):
		"""
		be default, the setup of the graph has one panel (i.e., only one row 
		and one column)
		@param width graph width [inches]
		@param height graph height [inches]
		@param dpi resolution, dots per inch (DPI)
		@param lineWidth graph line width
		@param padding graph padding [0..1]
		@param fontSize 
		@param fontFamily serif, sans-serif, etc.
		@param fontFace
		@param fontWeight normal, bold, etc...
		@param numeratorType numerator type for panels:
			NUMERATOR_TYPE_ALPHA_LOWER: lowercase letter
			NUMERATOR_TYPE_ALPHA_UPPER: uppercase letter
			NUMERATOR_TYPE_ARABIC: arabic numeral
			NUMERATOR_TYPE_ROMAN: roman numeral
		"""
		self.width = width
		self.height = height
		self.dpi = dpi
		self.lineWidth = lineWidth
		self.padding = padding
		self.fontSize = fontSize
		self.fontFamily = fontFamily
		self.fontSerif = fontFace
		self.fontWeight = fontWeight
		self.fig = None
		self.numeratorType = numeratorType
		self.arrAx = [] # the array that will store the individual panels
		self.arrAxPerRow = [] 
		self.arrRowRatios = []
		self.arrRowColumns = []
		
		self.setRowRatios([1])
	
	# ---------------------------------------------------------------------- #
	
	def setRowRatios(self,
		arrRowRatios
	):
		"""
		use this function to define how many panel rows contained in the graph,
		and what their respective relative panel height will be. calling this 
		function will reset any previously defined panel column information 
		(i.e., call setColumns immediately after the call to setRowRatios if 
		you want to have multi-column rows)
		
		@param arrRowRatios a list of relative row heights
		
		"""
		if not type(arrRowRatios).__name__ in ['list', 'ndarray']:
			raise Exception("arrRowRatios must either be a list or a numpy array")
		self.arrRowRatios = numpy.array(arrRowRatios)
		self.arrRowColumns = numpy.ones(len(arrRowRatios))
	
	# ---------------------------------------------------------------------- #
	
	def setColumns(self,
		arrColumnInfo
	):
		"""
		use this function if you need rows that contain more than one panel
		(i.e., multi-column rows). For practical reasons the column number needs
		to be restricted to 1 or a particular number of rows, such as, e.g.:
		[4, 1, 1, 4, 1]. Multiple numbers of colums (other than one) for
		different rows (such as, e.g., [4, 1, 2, 4, 1]) are not allowed
		
		@param a list containing information how many columns each row will have
		
		"""
		if not type(arrColumnInfo).__name__ in ['list', 'ndarray']:
			raise Exception("arrRowRatios must either be a list or a numpy array")
		if len(arrColumnInfo) != len(self.arrRowRatios):
			raise Exception("arrColumnInfo must have the same number of " \
				+ "entries as the previously defined self.arrRowRatios")
		for i in range(len(arrColumnInfo)):
			if arrColumnInfo[i] < 1:
				raise Exception("row " + str(i + 1) \
					+ " must have at least one column")
		numCols = None
		for n in arrColumnInfo:
			if n != int(n):
				raise Exception("column numbers need to be integers")
			if n != 1:
				if numCols is None:
					numCols = n
				else:
					if n != numCols:
						raise Exception("column information is too complex")
		self.arrRowColumns = numpy.array(arrColumnInfo)
		
	# ---------------------------------------------------------------------- #
		
	def createFigure(self, 
			adjustFont = True
	):
		"""
		creates the figure and takes care of the layout
		
		@param adjustFont if False, we'll take the system default font settings
		"""
		if len(self.arrRowRatios) < 1:
			raise Exception("need to define row ratios and columns first")
	
		if adjustFont:
			font = {
				'family' : self.fontFamily,
				'serif'  : self.fontSerif,
		        'weight' : self.fontWeight,
		        'size'   : self.fontSize,
			}
			matplotlib.rc('font', **font)
		plt.close()
		#plt.clf()
		self.fig = plt.figure(figsize=(self.width, self.height), dpi=self.dpi)
		
		self.arrAx = []
		self.arrAxPerRow = []
		numRows = len(self.arrRowRatios)
		numCols = int(self.arrRowColumns.max())
		gs = gridspec.GridSpec(numRows, numCols, \
			height_ratios = self.arrRowRatios)
		for row in range(numRows):
			numColsTmp = self.arrRowColumns[row]
			arrAxTmp = []
			if numColsTmp == 1:
				x = row * numCols
				y = row * numCols + (numCols - 1)
				ax = plt.subplot(gridspec.SubplotSpec(gs, x, y))
				self.arrAx.append(ax)
				arrAxTmp.append(ax)
			else:
				for col in range(numCols):
					ax = plt.subplot(gs[col + row * numCols])
					self.arrAx.append(ax)
					arrAxTmp.append(ax)
			self.arrAxPerRow.append(arrAxTmp)
		return self.arrAx
		
		
	# ---------------------------------------------------------------------- #
	
	def adjustPadding(self,
		left = 1.0, # factor by which self.padding is multiplied
		right = 1.0, # factor by which self.padding is multiplied
		top = 1.0, # factor by which self.padding is multiplied
		bottom = 1.0, # factor by which self.padding is multiplied
		hspace = 0.5, # hspace parameter for plt.subplots_adjust function
		wspace = 0.5, # wspace parameter for plt.subplots_adjust function
	):
		""" 
		convenience function to adjust padding in the graph
		
		@param left factor by which self.padding is multiplied
		@param right factor by which self.padding is multiplied
		@param top factor by which self.padding is multiplied
		@param bottom factor by which self.padding is multiplied
		@param hspace hspace parameter for plt.subplots_adjust function
		@param wspace wspace parameter for plt.subplots_adjust function
		"""
		plt.subplots_adjust(hspace=hspace, wspace=wspace, 
			bottom=self.padding * bottom, top=1.0 - self.padding * top, 
			left=self.padding * left, right = 1.0 - self.padding * right)
		
	# ---------------------------------------------------------------------- #

	def addPanelNumbers(self,
		numeratorType = None,
		fontSize = 14,
		fontWeight = 'bold',
		countEveryPanel = True,
		format = '(%s)',
		offsetLeft = 0.06,
		offsetTop = 0.00,
		horizontalAlignment = 'center',
		verticalAlignment = 'top',
	):
		"""
		add numbers for the panels (top right corner). make sure this function
		is only called AFTER adjustPadding(...)
		
		@param numeratorType several numbering types are available: 
			- NUMERATOR_TYPE_ALPHA_LOWER: lowercase letter
			- NUMERATOR_TYPE_ALPHA_UPPER: uppercase letter
			- NUMERATOR_TYPE_ARABIC: arabic numeral
			- NUMERATOR_TYPE_ROMAN: roman numeral
			If None, we'll use numerator type that was specified during object
			instantiation.
		@param fontSize
		@param fontWeight 'normal' | 'bold' | 'heavy' | 'light' | 'ultrabold' | 
			'ultralight'
		@param countEveryPanel if False, only one label is added per row
		@param format format as used in the printf statement. 
		@param offsetLeft [0..1] offset of panel numbers from top-left \
			corner of axis
		@param offsetTop [0..1] offset of panel numbers from top-left \
			corner of axis
		@param horizontalAlignment 'center' | 'right' | 'left'
		@param verticalAlignment 'center' | 'top' | 'bottom' | 'baseline'
		"""
		
		if numeratorType is None:
			numeratorType = self.numeratorType
		cnt = 0
		for rowIdx, arrAxTmp in enumerate(self.arrAxPerRow):
			for colIdx, ax in enumerate(self.arrAxPerRow[rowIdx]):
				if colIdx == 0 or countEveryPanel:
					s = ''
					cnt += 1
					if numeratorType == 0:
						s = chr(96 + cnt)
					elif numeratorType == 1:
						s = chr(64 + cnt)
					elif numeratorType == 2:
						s = str(cnt)
					elif numeratorType == 3:
						s = generalUtility.intToRoman(cnt)
					label = format % s
					bbox = ax.get_position()
					x = bbox.x0 - offsetLeft
					y = bbox.y0 + bbox.height + offsetTop
					plt.figtext(x, y, label, size=fontSize, weight=fontWeight, \
						ha = horizontalAlignment, va = verticalAlignment)
					
	# ---------------------------------------------------------------------- #
	
	def getArrAx(self):
		return self.arrAx
	
	# ---------------------------------------------------------------------- #
		
	def getFig(self):
		return self.fig	
	
	# ---------------------------------------------------------------------- #
	

################################################################################

def formatAxisTicks(
		ax,
		axisType,
		majorAxisDivisor,
		format='%-1.2f',
		minorAxisRelativeDivisor = 5.0
	): 
	"""
	change the format to the axis ticks
	@param ax matplotlib axes object
	@param axisType 0 or 'x': xaxis; 1 or 'y': yaxis
	@param majorAxisDivisor the major axis divisor
	@param format formatting string
	@param minorAxisRelativeDivisor value by which the major axis divisor will 
		be divided in order to obtain the minor axis divisor
	"""
	majorLocator   = MultipleLocator(majorAxisDivisor)
	majorFormatter = FormatStrFormatter(format)
	minorLocator   = MultipleLocator(majorAxisDivisor \
		/ float(minorAxisRelativeDivisor))
	if axisType == 0 or axisType == 'x':
		ax.xaxis.set_major_locator(majorLocator)
		ax.xaxis.set_major_formatter(majorFormatter)
		ax.xaxis.set_minor_locator(minorLocator)
	elif axisType == 1 or axisType == 'y':
		ax.yaxis.set_major_locator(majorLocator)
		ax.yaxis.set_major_formatter(majorFormatter)
		ax.yaxis.set_minor_locator(minorLocator)
	else:
		raise Exception("axis type not recignized (must be either 0 [xaxis] " \
			+ "or 1 [yaxis]")
		
################################################################################

def setLimit(
		ax, 
		arrData,
		axis = 'x',
		rangeMultiplier = 0.1
	):
	"""
	sets the x-axis limit of a graph centred on the respective data values, 
	adding some extra headspace on both the minimum and maximum end
	@param ax matplotlib axes object
	@param arrData either a list or a 1D numpy array (from which we get the \
		min/max values)
	@param axis either x-axis ('x' or 0) or y-axis ('y' or 1)
	@param rangeMultiplier leeway (extending the data range) on both ends of 
		the y axis
	"""
	valMin = min(arrData)
	valMax = max(arrData)
	valRange = valMax - valMin
	leeway = valRange * rangeMultiplier
	if axis in ['x', 0]:
		ax.set_xlim(valMin - leeway, valMax + leeway)
	elif axis in ['y', 1]:
		ax.set_ylim(valMin - leeway, valMax + leeway)
	else:
		raise Exception("undefined axis")

################################################################################

def plotPolyFit(ax, dataX, dataY, degrees = 1, fontSize = 10, lineSize = 3, 
		lineColor = 'red', txtX = None, txtY = None, numDigitsEq = 4, lineStyle = 'k--'
):
	"""
	performs a polynomial fit through the given data
	@param ax matplotlib axes object
	@param dataX x-axis data array. must not contain NaN or Inf values
	@param dataY y-axis data array. must not contain NaN or Inf values
	@param degrees degrees of the fitted polynomial
	@param fontSize font size for printing the polynomial equation
	@param lineSize line width of the fit
	@param lineColor color of the fitted polynomial on the graph
	@param txtX x-axis co-ordinate (data value) of the equation 
	@param txtY y-axis co-ordinate (data value) of the equation  
	@param numDigitsEq numer of decimal digits in the printed equation
	"""
	
	if len(dataX) < 2:
		raise Exception("data count must be 2 or larger")
		
	if dspUtil.containsNanInf(dataX):
		raise Exception("x-axis data contains NaN or Inf values. aborting.")
	if dspUtil.containsNanInf(dataY):
		raise Exception("y-axis data contains NaN or Inf values. aborting.")
		
	p = numpy.polyfit(dataX, dataY, degrees)
	#print "\t\t", p
	y1 = numpy.polyval(p,dataX)
	#plt.plot(x, y1, "k-", color='red', linewidth = 2)
	def getY(p, x):
		y = 0
		for i in range(len(p)):
			y += p[len(p) - (i + 1)] * (x ** float(i))
		return y
	arrX = []
	arrY = []
	xMin = min(dataX)
	xMax = max(dataX)
	step = (xMax - xMin) / 1000.0
	for x in numpy.arange(xMin, xMax, step):
		y = getY(p, x)
		arrX.append(x)
		arrY.append(y)
	ax.plot(arrX, arrY, lineStyle, color=lineColor, linewidth = lineSize)	

	# calculate the variance and R2
	SSerr = 0
	SStot = 0
	mean = sum(dataY) / float(len(dataY))
	for i in range(len(dataX)):
		y_i = dataY[i]
		f_i = getY(p, dataX[i])
		SSerr += (y_i - f_i) * (y_i - f_i)
		SStot += (y_i - mean) * (y_i - mean)
	R2 = 1 - (SSerr / SStot)
	#print "\t\tR2:", R2

	if fontSize > 0:
		txt = '$y = '
		for i in range(len(p)):
			factor = 1
			if i > 0:
				if p[i] > 0:
					txt += '+ '
				else:
					txt += '- '
					factor = -1
			txt += (('%.' + str(numDigitsEq) + 'f ') % (p[i] * factor))
			idx = len(p) - (i + 1)
			if idx == 0:
				pass
			elif idx == 1:
				txt += 'x '
			else: txt += 'x%d '%idx
		#print txt
		txt += (('$\n$R^2 = %.' + str(numDigitsEq) + 'f$') % R2)
		yMin = min(dataY)
		yMax = max(dataY)
		if txtX is None:
			txtX = xMin + (xMax - xMin) / 20.0
		if txtY is None:
			txtY = yMin + (yMax - yMin) / 25.0
			if len(p) == 2 and p[0] > 0:
				txtY = yMax - (yMax - yMin) / 6.0
		ax.text(txtX, txtY, txt, fontsize = fontSize)
	#print "\t\tdone"
	return p, R2, xMin, xMax

################################################################################

def plotIsocontours(
	ax, # the axis object on which we'll plot
	arrX, # x-axis grid values, 1D vector
	arrY, # y-axis grid values, 1D vector
	arrZ, # z-axis data values, 2D array
	colorMap = None, # colour map
	numIsocontours = 6, # number of isocountours
	paintAlpha = 0.75, 
	contourFontSize = 0, # if 0 or None, we won't add contour lables
	isoContourColor = 'black', 
	isoContourLineWidth = 0.5, 
	isoContourAlpha = 0.75
):
	""" 
	draw a 2D grid with respective values (i.e., a 3D data object) and add
	isocontour lines
	@param ax the axis object on which we'll plot
	@param arrX x-axis grid values, 1D vector
	@param arrY y-axis grid values, 1D vector
	@param arrZ z-axis data values, 2D array
	@param colorMap colour map
	@param numIsocontours number of isocountours
	@param paintAlpha 
	@param contourFontSize if 0 or None, we won't add contour lables
	@param isoContourColor 
	@param isoContourLineWidth 
	@param isoContourAlpha 
	"""
	CS = ax.contour(arrX, arrY, arrZ, numIsocontours, colors = isoContourColor, \
		linewidth = isoContourLineWidth, alpha = isoContourAlpha)
	#plt.pcolormesh(x, y, z, cmap = plt.get_cmap('rainbow'))
	if colorMap is None: colorMap = cm.afmhot
	CS2 = ax.contourf(arrX, arrY, arrZ, numIsocontours, \
		alpha = isoContourAlpha, cmap = colorMap)
	if contourFontSize > 0:
		ax.clabel(CS, inline = 1, fontsize = contourFontSize)

################################################################################

def plotSpectrogram(
		ax, 
		data, 
		fs, 
		windowSize = 4096, 
		readProgress = 0.05, 
		fMax = 5000, 
		dynamicRange = 50,
		spectrumType = dspUtil.POWER_SPECTRUM
	):
	""" 
	convenience function for generating a spectrogram (by calling 
	@ref calculateSpectrogram() from the @ref dspUtil module, and plotting the 
	contents at the same time
	@ax matplotlib axes object, on which the spectrogram is plotted
	@param data a list or numpy array
	@param fs sampling frequency [Hz]
	@param windowSize see @ref calculateFFT()
	@param readProgress time interval by which the analysis window is advanced.
	@param fMax maximum frequency [Hz]. no data above this frequency is being
		returned by this function (even though it may be calculated, depending
		on the sampling frequency), as a convenience for plotting the results
	@param dynamicRange inidcated in dB. similar as in Praat, see 
		<a href="http://www.fon.hum.uva.nl/praat/manual/Intro_3_2__Configuring_the_spectrogram.html">there</a>
	@param spectrumType see @ref calculateFFT()
	@return a three-dimensional array [y][x][z] ready to be plotted with the
		matplotlib <a href="
http://matplotlib.org/api/pyplot_api.html#matplotlib.pyplot.imshow">imshow</a>
		function. Note that the third (i.e., z) dimension is set to grayscale,
		i.e., all three RGB values have the same value, which is a bit redundant.
	"""
	spectrogramData = dspUtil.calculateSpectrogram(data, fs, windowSize, \
		readProgress, fMax, dynamicRange, spectrumType)
	duration = len(data) / float(fs)
	ax.imshow(spectrogramData, aspect='auto', origin='lower', 
		extent=[0, duration,0,fMax])
	return spectrogramData

################################################################################
	


def plotData(
	ax,
	data, 
	fs,
	labelY,
	tStart = 0, 
	color='blue',
	linewidth = 1, 
	valMin = None,
	valMax = None,
	alpha = 0.8,
	dontPlotData = False
):
	"""
	convenience function to plot time-series data. no rocket science here, just
	emulating the funcionality of a few function calls in one single function
	@param ax matplotlib axes object, onto which the graph is drawn
	@param data a numpy array or a list of numpy arrays
	@param fs sampling freuquency [Hz]
	@param labelY a string containing the label of the y axis
	@param tStartOffset useful when the time axis should not start at t = 0, but
		at a later moment in time
	@param color graph colour
	@param linewidth graph line width
	@param valMin minimum display value. if None, we'll take the minimum of the
		provided data
	@param valMax maximum display value. if None, we'll take the maximum of the
		provided data
	@param alpha transparency of the graph [0..1]
	@param dontPlotData for some very large files it might be useful to only 
		plot labels and grid, but not the actual data, if the graph is 
		imported into Adobe Illustrator (would take forever to load)
	@return nothing
	"""
	arrData = None
	dataType = type(data).__name__
	if dataType == 'list':
		arrData = data
	else:
		arrData = [data]
	numFrames = len(arrData[0])
	dataT = numpy.zeros(numFrames)
	for i in range(numFrames):
		dataT[i] = float(i) / float(fs)
	for data in arrData:
		ax.plot(dataT + tStart, data, linewidth=linewidth, alpha=alpha, color=color)
	ax.grid()
	if (not valMin is None) and (not valMax is None):
		ax.set_ylim(valMin, valMax)
	ax.set_xlim(tStart, tStart + dataT[-1])
	ax.set_xlabel('Time [s]')
	ax.set_ylabel(labelY)
	return
	
################################################################################
	
def plotVerticalMarkers(
	ax, arrMarkerOffset,
	color = 'red',
	valMin = -1, 
	valMax = 1,
	linestyle = '--',
	linewidth = 1,
	alpha = 1
):
	"""
	plot vertical markers on a graph, e.g. indicating the temporal offset of
	an interesting event, or highlighting data extraction
	@param ax matplotlib axes object
	@param arrMarkerOffset x-axis value
	@param color a matplotlib colour
	@param valMin minimum value of the graph
	@param valMax maximum value of the graph
	@param linestyle a valid matplotlib linestyle
	@param linewidth line width
	@param alpha transparency setting [0..1]
	"""
	dataType = type(arrMarkerOffset).__name__
	if dataType <> 'list':
		arrMarkerOffset = [arrMarkerOffset]
	for markerOffset in arrMarkerOffset:
		ax.plot([markerOffset, markerOffset], [valMin, valMax], linestyle, \
			color=color, linewidth = linewidth, alpha = alpha)
	
################################################################################