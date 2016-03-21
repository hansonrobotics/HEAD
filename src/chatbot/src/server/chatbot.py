from characters import CHARACTERS
from httplib import HTTPConnection
import urllib
import json
import logging

SUCCESS=0
WRONG_CHARACTER_NAME=1

useSOLR = True

logger = logging.getLogger('hr.chatbot.server.chatbot')

def get_character(name):
    for character in CHARACTERS:
        if character.name == name:
            return character

def list_character():
    return [c.name for c in CHARACTERS]

def solr(text):
    # No match, try improving with SOLR
    conn = HTTPConnection('localhost', 8983)
    headers = {'Content-type': 'application/json'}
    url = '/solr/aiml/select?indent=true&wt=json&fl=*,score&rows=20&qf=title&q=' + text.replace(' ', '%20')
    conn.request('GET', url)
    lucResponse = conn.getresponse()
    conn.close()
    # logger.warn('STATUS: %i', lucResponse.status)
    lucText = lucResponse.read()

    if len(lucText)>0:
        logger.warn('RESPONSE: ' + lucText)
        jResp = json.loads(lucText)
        if jResp['response']['numFound'] > 0:
            doc = jResp['response']['docs'][0]
            lucResult = doc['title'][0]
            return lucResult

def ask(name, question, session=None):
    """
    return (response dict, return code)
    """
    character = get_character(name)
    if not character:
        return None, WRONG_CHARACTER_NAME

    answer = character.respond(question, session)
    if answer.get('response', None):
        return answer, SUCCESS

    if useSOLR and len(question) > 40:
        lucResult = None
        try:
            lucResult = solr(question)
        except Exception as ex:
            logger.warn(ex)

        if lucResult:
            answer = character.respond(lucResult)
            logger.warn('LUCENE: %s -> %s' % (lucResult, answer))
            if answer.get('response', None):
                return answer, SUCCESS

    generic = get_character('generic')
    generic.set_properties(character.get_properties())
    answer = generic.respond(question, session)
    return answer, SUCCESS

if __name__ == '__main__':
    for character in CHARACTERS:
        print ask(character.name, 'what is your name')

