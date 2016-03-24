from characters import CHARACTERS
from httplib import HTTPConnection
import urllib
import json
import logging

SUCCESS=0
WRONG_CHARACTER_NAME=1

useSOLR = True

logger = logging.getLogger('hr.chatbot.server.chatbot')

def get_character(id):
    for character in CHARACTERS:
        if character.id == id:
            return character

def list_character():
    return [c.id for c in CHARACTERS]

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

def ask(id, question, session=None):
    """
    return (response dict, return code)
    """
    answer = {'response': '', 'emotion': '', 'botid': '', 'botname': ''}
    character = get_character(id)
    if not character:
        return answer, WRONG_CHARACTER_NAME

    first_tier = character.respond(question, session)
    if first_tier.get('response', None):
        assert isinstance(first_tier, dict), "Response must be a dict"
        answer.update(first_tier)
        return answer, SUCCESS

    if useSOLR and len(question) > 40:
        lucResult = None
        try:
            lucResult = solr(question)
        except Exception as ex:
            logger.warn(ex)

        if lucResult:
            solr_answer = character.respond(lucResult)
            logger.warn('LUCENE: %s -> %s' % (lucResult, solr_answer))
            if solr_answer.get('response', None):
                assert isinstance(solr_answer, dict), "Response must be a dict"
                answer.update(solr_answer)
                return answer, SUCCESS

    generic = get_character('generic')
    generic.set_properties(character.get_properties())
    second_tier = generic.respond(question, session)
    assert isinstance(second_tier, dict), "Response must be a dict"
    answer.update(second_tier)
    logger.info("Ask {}, answer {}".format(question, answer))
    return answer, SUCCESS

if __name__ == '__main__':
    for character in CHARACTERS:
        print ask(character.id, 'what is your name')

