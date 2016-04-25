# -*- coding: utf-8 -*-
import os
import logging
from server.character import Character
from server.solr_match import solr3col

logger = logging.getLogger('hr.chatbot.server.characters.solr_bot')

class SolrCharacter(Character):
    def __init__(self, id, name):
        super(SolrCharacter, self).__init__(id, name)

    def respond(self, question, session=None):
        if isinstance(question, unicode):
            question = question.encode('utf-8')
        ret = {}
        try:
            solr_respone = solr3col(question)
            if solr_respone is not None:
                ret['solr'] = solr_respone[0]
                ret['text'] = solr_respone[1]
                ret['botid'] = self.id
                ret['botname'] = self.name
        except Exception as ex:
            logger.error(ex)
        return ret

solr_bot = SolrCharacter('solr_bot', 'solr_bot')

characters = [solr_bot]

if __name__ == '__main__':
    questions = ['do you have animal friends?']
    for q in questions:
        print solr_bot.respond(q)
