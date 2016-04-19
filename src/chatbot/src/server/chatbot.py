from characters import CHARACTERS
from httplib import HTTPConnection
from character import SheetAIMLCharacter
import urllib
import json
import logging
import server
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

SUCCESS=0
WRONG_CHARACTER_NAME=1

useSOLR = True

logger = logging.getLogger('hr.chatbot.server.chatbot')

def get_character(id, create=False):
    for character in CHARACTERS:
        if character.id == id:
            return character
    if create:
        character = SheetAIMLCharacter(id)
        CHARACTERS.append(character)
        logger.info("Create SheetAIMLCharacter {}".format(character))
        return character

def list_character():
    return [c.id for c in CHARACTERS]

def update_character(id, csv_version):
    character = get_character(id)
    if not character:
        return False, "Character {} is not found".format(id)
    if isinstance(character, server.character.SheetAIMLCharacter) or \
            isinstance(character, SheetAIMLCharacter):
        try:
            character.load_csv_files(csv_version)
        except Exception as ex:
            logger.error(ex)
            return False, "Update {} failed {}".format(id, ex)
        return True, "{} is updated".format(id)
    else:
        return False, "Character {} doesn't support update".format(id)
    return False

def load_sheet_keys(id, sheet_keys):
    character = get_character(id, True)
    if not character:
        return False, "Character {} is not found".format(id)
    if not sheet_keys:
        return False, "No sheet key is set"
    if isinstance(character, server.character.SheetAIMLCharacter) or \
            isinstance(character, SheetAIMLCharacter):
        return character.load_sheet_keys(sheet_keys)
    else:
        return False, "Character doesn't support sheet keys"
    return False, "Unknown error"

def commit_character(id):
    character = get_character(id)
    if not character:
        return False, "Character {} is not found".format(id)
    if isinstance(character, server.character.SheetAIMLCharacter) or \
            isinstance(character, SheetAIMLCharacter):
        return character.commit()
    else:
        return False, "Character {} doesn't support committing".format(character)

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
    response = {'text': '', 'emotion': '', 'botid': '', 'botname': ''}
    character = get_character(id)
    if not character:
        return response, WRONG_CHARACTER_NAME

    first_tier = character.respond(question, session)
    if first_tier.get('text', None):
        assert isinstance(first_tier, dict), "Response must be a dict"
        response.update(first_tier)
        return response, SUCCESS

    if useSOLR and len(question) > 40:
        lucResult = None
        try:
            lucResult = solr(question)
        except Exception as ex:
            logger.warn(ex)

        if lucResult:
            solr_response = character.respond(lucResult)
            logger.warn('LUCENE: %s -> %s' % (lucResult, solr_response))
            if solr_response.get('text', None):
                assert isinstance(solr_response, dict), "Response must be a dict"
                response.update(solr_response)
                return response, SUCCESS

    generic = get_character('generic')
    if generic is not None:
        generic.set_properties(character.get_properties())
        second_tier = generic.respond(question, session)
        assert isinstance(second_tier, dict), "Response must be a dict"
        response.update(second_tier)
    logger.info("Ask {}, response {}".format(question, response))
    return response, SUCCESS

if __name__ == '__main__':
    for character in CHARACTERS:
        print ask(character.id, 'what is your name')

