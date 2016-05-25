# -*- coding: utf-8 -*-
from server.character import Character

class DummyCharacter(Character):
    def __init__(self, id, name, level=99):
        super(DummyCharacter, self).__init__(id, name, level)
        self.languages = ['en', 'zh']

    def respond(self, question, lang, session=None):
        ret = {}
        if lang not in self.languages:
            ret['text'] = ''
        if lang=='en':
            answer = "Sorry, I can't answer that"
        elif lang=='zh':
            answer = u"对不起，我好像不明白。"
        ret['text'] = answer
        ret['botid'] = self.id
        ret['botname'] = self.name
        return ret

dummy = DummyCharacter('dummy', 'dummy')

characters = [dummy]
