# -*- coding: utf-8 -*-
import requests
import logging
from chatbot.server.character import Character

logger = logging.getLogger('hr.duckduckgo')

class DDG(Character):
    def __init__(self, id, name):
        super(DDG, self).__init__(id, name)
        self.languages = ['en']
        self.level = 95
        self.weight = 0.25
        self.non_repeat = False

    def ask(self, question):
        response = requests.get('http://api.duckduckgo.com', params={'q': question, 'format': 'json'})
        return response.json()['Abstract'] or response.json()['Answer']

    def respond(self, question, lang, *args, **kwargs):
        ret = {}
        ret['botid'] = self.id
        ret['botname'] = self.name
        if lang not in self.languages:
            ret['text'] = ''
        else:
            ret['text'] = self.ask(question)
        return ret

character = DDG('ddg', 'sophia')

characters = [character]

if __name__ == '__main__':
    logging.basicConfig()
    logging.getLogger().setLevel(logging.INFO)
    print character.respond(u'what is duckduckgo', 'en')['text']
    print character.respond(u'how many days a year', 'en')['text']
