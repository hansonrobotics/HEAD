from mary import MaryTTSClient
from festival import FestivalTTSClient
import logging
import os

logger = logging.getLogger('hr.tts.ttsapi')

tts_apis = {
    'mary': MaryTTSClient('~/.hr/tts/marytts'),
    'festival': FestivalTTSClient('~/.hr/tts/festival'),
}

def get_api(name):
    return tts_apis.get(name, None)

def set_voice(name, voice):
    api = get_api(name)
    if api is not None:
        api.set_voice(voice)
        logger.info("Set {} voice to {}".format(name, voice))

