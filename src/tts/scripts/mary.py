#!/usr/bin/env python
import os
import requests
import logging

from common.ttsbase import TTSBase, TTSData
from common.visemes import BaseVisemes

logger = logging.getLogger('hr.tts.marytts')

class MaryTTSVisemes(BaseVisemes):
    default_visemes_map = {
        'A-I': ['aa','ae','ah','ao','ax','axr','ih','iy'],
        'E': ['ay','eh','ey'],
        'O': ['aw','ow','oy'],
        'U': ['uh','uw'],
        'C-D-G-K-N-S-TH': ['ch','dh','dx','g','h','jh','k','s','sh','th','y','z','zh','hh'],
        'F-V': ['f','hv', 'v'],
        'L': ['d','el','er','l','r','t'],
        'M': ['b','em','en','m','n','nx','ng','p'],
        'Q-W': ['w'],
        'Sil': ['pau', 'brth']
    }

class MaryTTSClient(TTSBase):
    def __init__(self, output_dir):
        super(MaryTTSClient, self).__init__(output_dir)
        self.host = "http://127.0.0.1"
        self.port = 59125
        self.params = {
            "INPUT_TYPE": "TEXT",
            "OUTPUT_TYPE": "AUDIO",
            "LOCALE": "en_GB",
            "AUDIO": "WAVE_FILE",
            "VOICE": "cmu-slt-hsmm",
        }
        self.url = '{}:{}'.format(self.host, self.port)
        self.visemes_config = MaryTTSVisemes()

    def get_tts_session_params(self):
        return self.params

    def _tts(self, text):
        params = {
            "INPUT_TEXT": text,
        }
        params.update(self.params)
        response = requests.get('{}/process'.format(self.url), params=params)
        if response.status_code != 200:
            raise RuntimeError("{}".format(response.status_code))
        with open(self.wavout, 'wb') as f:
            f.write(response.content)
        tts_data = TTSData()
        tts_data.wavout = self.wavout
        return tts_data

if __name__ == "__main__":
    logging.basicConfig()
    MaryTTSClient('.').tts("hello from Mary Text to Speech, with Python.")
