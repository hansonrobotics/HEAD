# -*- coding: utf-8 -*-

from __future__ import division
import os
import re
import logging
import hashlib
import pinyin
from scipy.io import wavfile
import shutil
import yaml
import xml.etree.ElementTree
from audio2phoneme import audio2phoneme
from visemes import Numb_Visemes

CWD = os.path.dirname(os.path.realpath(__file__))
logger = logging.getLogger('hr.tools.common.ttsbase')

# User data class to store information
class TTSData:
    def __init__(self):
        self.wavout =  None
        self.phonemes = []
        self.visemes = []

    def __repr__(self):
        return "<TTSData: wavout {}>".format(self.wavout)

class TTSException(Exception):
    def __init__(self, msg):
        self.msg = msg
    def __str__(self):
        return self.msg

class TTSBase(object):
    def __init__(self, output_dir):
        self.output_dir = os.path.expanduser(output_dir)
        self.wavout = os.path.join(self.output_dir, 'tmp.wav')
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)

    def get_tts_session_params(self):
        raise NotImplementedError("get_tts_session_params is not implemented")

    def set_voice(self, voice):
        raise NotImplementedError("set_voice is not implemented")

    def _tts(self, text):
        raise NotImplementedError("_tts is not implemented")

    def get_duration(self):
        raise NotImplementedError("get_duration is not implemented")

    def tts(self, text):
        logger.info("TTS text {}".format(text))
        try:
            return self._tts(text)
        except Exception as ex:
            logger.error(ex)
        logger.info("TTS finished")

class NumbTTS(TTSBase):

    def get_visemes(self, phonemes):
        visemes = []
        for ph in phonemes:
            v = self.get_viseme(ph)
            if v is not None:
                visemes.append(v)
        logger.debug("Get visemes {}".format(visemes))
        return visemes

    def _tts(self, text):
        fname = '{}.wav'.format(os.path.join(self.output_dir, text.strip()))
        if os.path.isfile(fname):
            shutil.copy(fname, self.wavout)
            tts_data = TTSData()
            tts_data.wavout = self.wavout
            try:
                tts_data.phonemes = self.get_phonemes(fname)
                viseme_config = Numb_Visemes()
                tts_data.visemes = viseme_config.get_visemes(tts_data.phonemes)
                logger.info(tts_data.visemes)
            except Exception as ex:
                logger.error(ex)
                tts_data.phonemes = []
            return tts_data

    def get_phonemes(self, fname):
        timing = '{}.yaml'.format(os.path.splitext(fname)[0])
        if os.path.isfile(timing):
            with open(timing) as f:
                phonemes = yaml.load(f)
            logger.info("Get timing info from file")
        else:
            phonemes = [
                {'name': phoneme[0], 'start': phoneme[1], 'end': phoneme[2]}
                    for phoneme in audio2phoneme(fname)]
            with open(timing, 'w') as f:
                yaml.dump(phonemes, f)
            logger.info("Write timing info to file")
        return phonemes

    def get_duration(self):
        rate, data = wavfile.read(self.wavout)
        duration = data.shape[0]/rate
        logger.info("Duration of {} is {}".format(self.wavout, duration))
        return duration

class OnlineTTS(TTSBase):

    def __init__(self, output_dir):
        super(OnlineTTS, self).__init__(output_dir)
        self.cache_dir =  os.path.expanduser('{}/cache'.format(output_dir))
        if not os.path.isdir(self.cache_dir):
            os.makedirs(self.cache_dir)

    def get_cache_file(self, text):
        suffix = hashlib.sha1(text+str(self.get_tts_session_params())).hexdigest()[:6]
        filename = os.path.join(self.cache_dir, suffix+'.wav')
        return filename

    def offline_tts(self, text):
        cache_file = self.get_cache_file(text)
        if os.path.isfile(cache_file):
            shutil.copy(cache_file, self.wavout)
            logger.info("Get offline tts for {} {}".format(text, cache_file))
        else:
            raise TTSException("Offline tts failed, no such file {}".format(
                    self.get_cache_file(text)))

    def _tts(self, text):
        try:
            self.offline_tts(text)
        except Exception as ex:
            logger.error(ex)
            self.online_tts(text)
        tts_data = TTSData()
        tts_data.wavout = self.wavout
        return tts_data

    def online_tts(self, text):
        return NotImplemented

class ChineseTTSBase(OnlineTTS):
    def __init__(self, output_dir):
        super(ChineseTTSBase, self).__init__(output_dir)

    def get_cache_file(self, text):
        delimiter = '#'
        pys = pinyin.get(text, delimiter=delimiter)
        if pys:
            pys = pys.split(delimiter)
            pys=[py for py in pys if re.match('[a-zA-Z]', py)]
            pys = ''.join(pys)
        pys = pys[:251-6-1]
        suffix = hashlib.sha1(text+str(self.get_tts_session_params())).hexdigest()[:6]
        filename = os.path.join(self.cache_dir, pys+'_'+suffix+'.wav')
        return filename

    def nonchinese2pinyin(self, text):
        """replace non-Chinese characters to pinyins"""
        NON_CHN_MAP = {
            '0': 'ling', '1': 'yi', '2': 'er', '3': 'san', '4': 'si', '5': 'wu',
            '6': 'liu', '7': 'qi', '8': 'ba', '9': 'jiu',
        }
        pattern = re.compile('|'.join(NON_CHN_MAP.keys()))
        new_text = ''
        last_point = 0
        it = re.finditer('[0-9]+', text)
        for i in it:
            new_text += text[last_point:i.span()[0]]
            new_text += pattern.sub(lambda x: NON_CHN_MAP[x.group()]+' ', i.group()).strip()
            last_point = i.span()[1]
        new_text += text[last_point:]
        return new_text

    def is_ssml(self, text):
        try:
            el = xml.etree.ElementTree.XML(text)
            if el.tag == 'speak':
                return True
            else:
                return False
        except Exception as ex:
            logger.info("parse error {}".format(ex))
            return False

    def strip_tag(self, text):
        text = re.sub('<[^<]+>', '', text)
        text = re.sub('\s{1,}', ' ', text)
        return text.strip()

    def get_phonemes(self, txt):
        phonemes = []
        regexp = re.compile("""^(?P<initial>b|p|m|f|d|t|n|l|g|k|h|j|q|x|zh|ch|sh|r|z|c|s|y|w*)(?P<final>\w+)$""")
        if self.is_ssml(txt):
            txt = self.strip_tag(txt)
        pys = pinyin.get(txt, delimiter=' ')
        pys = self.nonchinese2pinyin(pys)
        pys = pys.strip().split(' ')
        logger.info('Get pinyin {}'.format(pys))
        if not pys:
            return []
        duration = self.get_duration()
        unit_time = float(duration)/len(pys)
        start_time = 0
        for py in pys:
            match = regexp.match(py)
            if match:
                mid_time = start_time + unit_time/2
                # Use 2 phonemes for a Chinese character
                initial = match.group('initial')
                final = match.group('final')
                if initial:
                    phonemes.append({
                        'name': initial.lower(),
                        'start': start_time,
                        'end': mid_time,
                    })
                if final:
                    phonemes.append({
                        'name': final.lower(),
                        'start': mid_time,
                        'end': start_time + unit_time,
                    })
                start_time += unit_time
        logger.debug('phonemes {}'.format(phonemes))
        return phonemes

    def get_duration(self):
        rate, data = wavfile.read(self.wavout)
        duration = data.shape[0]/rate
        logger.info("Duration of {} is {}".format(self.wavout, duration))
        return duration

    def _tts(self, text):
        tts_data = super(ChineseTTSBase, self)._tts(text)
        tts_data.phonemes = self.get_phonemes(text)
        return tts_data

