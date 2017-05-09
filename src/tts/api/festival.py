#!/usr/bin/env python
import os
import logging
import subprocess
from collections import defaultdict

from ttsserver.ttsbase import TTSBase, TTSData

logger = logging.getLogger('hr.tts.festival')

class FestivalTTS(TTSBase):
    def __init__(self):
        super(FestivalTTS, self).__init__()
        # avialable voices are:
        #   cmu_us_slt_arctic_hts (female)
        #   lp_diphone (female)
        #   pc_diphone (male)
        #   kal_diphone (male)
        #   rab_diphone (male)
        self.params = {
            "voice": "cmu_us_slt_arctic_hts",
        }
        self.timing = os.path.join(self.output_dir, 'timing')
        self.script = os.path.join(self.output_dir, 'tts.scm')

    def get_tts_session_params(self):
        return self.params

    def get_phonemes(self):
        phonemes = []
        with open(self.timing) as f:
            lines = f.read().splitlines()
            last_tick = 0
            for line in lines:
                if line.strip().startswith('#'):
                    continue
                tick, vol, phoneme  = line.strip().split(' ')
                tick = float(tick)
                phonemes.append({'name': phoneme, 'start': last_tick, 'end': tick})
                last_tick = tick
        return phonemes

    def do_tts(self, tts_data):
        try:
            with open(self.script, 'w') as f:
                f.write("""
(voice_{voice})
(set! utt1 (Utterance Text "{text}"))
(utt.synth utt1)
(utt.save.segs utt1 "{timingfile}")
(utt.save.wave utt1 "{audiofile}")""".format(
                    voice=self.params['voice'],
                    text=tts_data.text, timingfile=self.timing, audiofile=tts_data.wavout)
                )
            subprocess.Popen(
                ['festival', '-b', self.script], stdout=subprocess.PIPE,
                stderr=subprocess.PIPE).communicate()
        except Exception as ex:
            import traceback
            logger.error('TTS error: {}'.format(traceback.format_exc()))
            return

        tts_data.phonemes = self.get_phonemes()
        os.remove(self.timing)
        os.remove(self.script)

def load_voices():
    voices = defaultdict(dict)
    for voice in ['cmu_us_slt_arctic_hts', 'lp_diphone', 'pc_diphone', 'kal_diphone', 'rab_diphone']:
        try:
            logger.info("Adding {}:{}".format('festival', voice))
            api = FestivalTTS()
            api.params['voice'] = voice
            voices['festival'][voice] = api
            logger.info("{}:{} added".format('festival', voice))
        except Exception as ex:
            logger.error(ex)
    return voices

voices = load_voices()

if __name__ == "__main__":
    logging.basicConfig()
    voices['festival']['cmu_us_slt_arctic_hts'].tts(
        "hello from Festival Text to Speech, with Python.")
