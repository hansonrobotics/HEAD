# -*- coding: utf-8 -*-
import os
import logging
from server.character import Character
from server.solr_match import solr3col

logger = logging.getLogger('hr.chatbot.server.characters.solr_bot')

SCT = 30 # solr character threshold
SMT = 1 # solr match threshold
LEN = 140 # limit of response length

def shorten(text, cutoff):
    if len(text) < cutoff:
        return text
    sens = text.split('.')
    ret = ''
    for sen in sens:
        if len(ret) > cutoff:
            return ret
        ret += (sen+'.')
    return ret

class SolrCharacter(Character):
    def __init__(self, id, name):
        super(SolrCharacter, self).__init__(id, name)

    def respond(self, question, lang, session=None):
        if len(question) <= SCT:
            logger.info("Skip short question")
            return {}
        if isinstance(question, unicode):
            question = question.encode('utf-8')
        ret = {}
        try:
            solr_respone = solr3col(question)
            if solr_respone is not None:
                score = solr_respone[2]
                if score < SMT:
                    logger.info("Discard low score response {}".format(solr_respone))
                    return {}
                ret['solr'] = solr_respone[0]
                ret['text'] = shorten(solr_respone[1], LEN)
                ret['botid'] = self.id
                ret['botname'] = self.name
        except Exception as ex:
            logger.error(ex)
        return ret

solr_bot = SolrCharacter('solr_bot', 'solr_bot')
solr_bot.weight = 0.5

characters = [solr_bot]

if __name__ == '__main__':
    questions = ['do you have animal friends?']
    for q in questions:
        print solr_bot.respond(q, 'en')
