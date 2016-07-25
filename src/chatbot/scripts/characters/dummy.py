# -*- coding: utf-8 -*-
from server.character import Character
import random

NO_ANSWER = {
    'en': ["Sorry, I can't answer that", "Sorry, I don't know"],
    'zh': [u"对不起，我回答不了这个问题。", u"对不起，我不太明白。"]
}

REPEAT_ANSWER = {
    'en': ["I've already answered this question, let's talk about something else.", "I've already answered this question"],
    'zh': [u"这个问题我已经回答过了，换个问题吧。", u"这个问题我已经回答过了", u"我不爱回答相同的问题。"]
}
class DummyCharacter(Character):
    def __init__(self, id, name, level=99):
        super(DummyCharacter, self).__init__(id, name, level)
        self.languages = ['en', 'zh']

    def respond(self, question, lang, session=None):
        ret = {}
        ret['text'] = ''
        ret['botid'] = self.id
        ret['botname'] = self.name
        if lang not in self.languages:
            return ret
        if question == "REPEAT_ANSWER":
            ret['text'] = random.sample(REPEAT_ANSWER[lang], 1)[0]
        else:
            ret['text'] = random.sample(NO_ANSWER[lang], 1)[0]
        return ret

dummy = DummyCharacter('dummy', 'dummy')

characters = [dummy]
