# -*- coding: utf-8 -*-
import logging
logger = logging.getLogger('hr.tts.visemes')

class BaseVisemes:

    # Params for each visime passed to blender
    # duration: multiplier opf the actual visime time
    # rampin: time to ramp in in % of duration
    # rampout: time to ramp in in % of duration
    # magnitude: magnitude in blender
    visemes_param = {
       'A-I':               {'duration': 2.2, 'rampin': 0.35, 'rampout': 0.25, 'magnitude': 0.99},
       'E':                 {'duration': 2.5, 'rampin': 0.40, 'rampout': 0.25, 'magnitude': 0.99},
       'F-V':               {'duration': 1.0, 'rampin': 0.25, 'rampout': 0.25, 'magnitude': 0.99},
       'Q-W':               {'duration': 1.0, 'rampin': 0.25, 'rampout': 0.25, 'magnitude': 0.99},
       'L':                 {'duration': 1.0, 'rampin': 0.25, 'rampout': 0.25, 'magnitude': 0.99},
       'C-D-G-K-N-S-TH':    {'duration': 1.5, 'rampin': 0.35, 'rampout': 0.25, 'magnitude': 0.99},
       'M':                 {'duration': 1.2, 'rampin': 0.35, 'rampout': 0.25, 'magnitude': 0.99},
       'O':                 {'duration': 2.3, 'rampin': 0.40, 'rampout': 0.25, 'magnitude': 0.99},
       'U':                 {'duration': 2.0, 'rampin': 0.35, 'rampout': 0.25, 'magnitude': 0.99},
    }

    def __init__(self):
        self.set_visemes_map(self.default_visemes_map)

    def set_visemes_map(self, visemes_map):
        if visemes_map is not None:
            self.phonemes = {}
            for v, s in visemes_map.iteritems():
                for p in s:
                    self.phonemes[p] = v

    def get_visemes(self, phonemes):
        visemes = []
        for ph in phonemes:
            try:
                v = {}
                v['name'] = self.phonemes[ph['name']]
                v['start'] =ph['start']
                v['duration'] = ph['end']-ph['start']
            except KeyError:
                logger.error("Unknown phoneme "+ph['name'])
                continue
            visemes.append(v)
        logger.debug("Get visemes {}".format(visemes))
        return visemes


class English_Visemes(BaseVisemes):
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

class Pinyin_Viseme(BaseVisemes):

    #b玻p坡m摸f佛d得t特n讷l勒g哥k科h喝j基q欺x希y医w屋zh知ch吃sh诗r日z资c雌s思
    initials = [
        'b','p','m','f','d','t','n','l','g','k','h','j','q','x','y','w',
        'zh','ch','sh','r','z','c','s']
    #i衣u乌ü迂a啊ia呀ua蛙喔uo窝e鹅ie耶üe/ue约ai爱uai歪ei诶uei威ao熬iao腰
    #ou欧iou忧an安ian烟uan弯üan冤en恩in因uen温ün/un晕ang昂iang央uang汪
    #eng亨ing英ueng翁ong轰iong雍
    finals = [
        'i','u','v','a','ia','ua','o','uo','e','ie','ve','ue','ai','uai','ei',
        'uei','ao','iao','ou','iou','an','ian','uan','van','en','in','uen',
        'vn','ang','iang','uang','eng','ing','ueng','ong','iong',
        ]

    default_visemes_map = {
        'A-I': ['a','ai','ao','an','ang'],
        'E': ['e','ei','er','en','eng','t','x','ia','iao','iou','ian','iang','i','iu','ie','in','ing','iong','y'],
        'F-V': ['f'],
        'Q-W': ['w'],
        'L': ['l','r'],
        'C-D-G-K-N-S-TH': ['ch','d','g','h','k','n','s','sh','z','zh','j','c'],
        'M': ['b','m','p'],
        'O': ['o','uo','ou','ong'],
        'U': ['u','ui','un','v','ve','ue','vn','q','ua','uai','uan','van','uang','uei','uen','ueng'],
    }

