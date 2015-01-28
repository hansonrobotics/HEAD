import importlib
import re

def soft_import(name):
	try:
		return importlib.import_module(name)
	except ImportError:
		return None

def underscorize(string):
	''' E.g. underscorize('getEmotionStates') == get_emotion_states '''
	return re.sub(r'([A-Z]+)', r'_\1', string).lower()
