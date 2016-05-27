# -*- coding: utf-8 -*-
import logging
import random
import os
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import atexit

SUCCESS=0
WRONG_CHARACTER_NAME=1
NO_PATTERN_MATCH=2
INVALID_SESSION=3

useSOLR = True
CWD = os.path.dirname(os.path.realpath(__file__))

logger = logging.getLogger('hr.chatbot.server.chatbot')

from loader import load_characters
DEFAULT_CHARACTER_PATH = os.path.join(CWD, 'characters')
CHARACTER_PATH = os.environ.get('HR_CHARACTER_PATH', DEFAULT_CHARACTER_PATH)
CHARACTERS = load_characters(CHARACTER_PATH)

def get_character(id, lang=None):
    for character in CHARACTERS:
        if character.id != id:
            continue
        if lang is None:
            return character
        elif lang in character.languages:
            return character

def add_character(character):
    if character.id not in [c.id for c in CHARACTERS]:
        CHARACTERS.append(character)
        return True, "Character added"
    #TODO: Update character
    else:
        return False, "Character exists"

def is_local_character(character):
    return character.local

def get_characters_by_name(name, local=True, lang=None, user=None):
    characters = []
    _characters = [c for c in CHARACTERS if c.name == name]
    if local:
        _characters = [c for c in _characters if is_local_character(c)]
    if lang is not None:
        _characters = [c for c in _characters if lang in c.languages]

    if user is not None:
        for c in _characters:
            toks = c.id.split('/')
            if len(toks) == 2:
                if toks[0] == user:
                    characters.append(c)
            else:
                characters.append(c)
    else:
        characters = _characters
    return characters

def list_character(lang, sid):
    sess = session_manager.get_session(sid)
    if sess is None:
        return []
    responding_characters = get_responding_characters(lang, sid)
    if hasattr(sess.sdata, 'weights'):
        return [(c.id, w, c.level) for c, w in zip(
                responding_characters, sess.sdata.weights)]
    else:
        return [(c.id, c.weight, c.level) for c in responding_characters]

def list_character_names():
    names = list(set([c.name for c in CHARACTERS if c.name != 'dummy']))
    return names

def set_weights(weights, lang, sid):
    sess = session_manager.get_session(sid)
    if sess is None:
        return False, "No session"
    try:
        weights = [float(w.strip()) for w in weights.split(',')]
    except Exception:
        return False, "Wrong weight format"
    responding_characters = get_responding_characters(lang, sid)
    if len(weights) != len(responding_characters):
        return False, "Number of weights doesn't match number of tiers {}".format(weights)
    sess.sdata.weights = weights
    return True, "Weights are updated"

from session import SessionManager
session_manager = SessionManager()
MAX_CHAT_TRIES = 5
NON_REPEAT = True
def _ask_characters(characters, question, lang, sid):
    chat_tries = 0
    sess = session_manager.get_session(sid)
    if sess is None:
        return

    data = sess.get_session_data()
    if hasattr(data, 'weights'):
        weights = data.weights
    else:
        weights = [c.weight for c in characters]

    _question = question.lower().strip()
    _question = ' '.join(_question.split()) # remove consecutive spaces
    while chat_tries < MAX_CHAT_TRIES:
        chat_tries += 1
        for c, weight in zip(characters, weights):
            _response = c.respond(_question, lang, sid)
            assert isinstance(_response, dict), "Response must be a dict"
            answer = _response.get('text', '')
            if not answer:
                continue

            # Each tier has weight*100% chance to be selected.
            # If the chance goes to the last tier, it will be selected anyway.
            if random.random()<weight:
                if not NON_REPEAT or sess.check(_question, answer, lang):
                    sess.add(_question, answer)
                    return _response

    # Ask the same question to every tier to sync internal state
    [c.respond(_question, lang, sid) for c in characters]

    dummy_character = get_character('dummy', lang)
    if dummy_character:
        _response = dummy_character.respond(_question, lang, sid)
        answer = _response.get('text', '')
        sess.add(_question, answer)
        return _response

def get_responding_characters(lang, sid):
    sess = session_manager.get_session(sid)
    if sess is None:
        return []
    if not hasattr(sess.sdata, 'botname'):
        return []

    botname = sess.sdata.botname
    user = sess.sdata.user

    # current character > local character with the same name > solr > generic
    responding_characters = get_characters_by_name(botname, local=False, lang=lang, user=user)
    responding_characters = sorted(responding_characters, key=lambda x: x.level)
    character = responding_characters[0]

    if useSOLR:
        solr_character = get_character('solr_bot', lang)
        if solr_character:
            if solr_character not in responding_characters:
                responding_characters.append(solr_character)
        else:
            logger.warn("Solr character is not found")
        solr_matcher = get_character('solr_matcher', lang)
        if solr_matcher:
            if solr_matcher not in responding_characters:
                solr_matcher.set_character(character)
                responding_characters.append(solr_matcher)
        else:
            logger.warn("Solr matcher is not found")

    generic = get_character('generic', lang)
    if generic:
        if generic not in responding_characters:
            generic.set_properties(character.get_properties())
            responding_characters.append(generic)
    else:
        logger.warn("Generic character is not found")

    responding_characters = sorted(responding_characters, key=lambda x: x.level)

    return responding_characters

def ask(question, lang, sid):
    """
    return (response dict, return code)
    """
    response = {'text': '', 'emotion': '', 'botid': '', 'botname': ''}

    sess = session_manager.get_session(sid)
    if sess is None:
        return response, INVALID_SESSION

    responding_characters = get_responding_characters(lang, sid)
    if not responding_characters:
        logger.error("Wrong characer name")
        return response, WRONG_CHARACTER_NAME

    sess.characters = responding_characters
    if question and question.lower().strip() in ['hi', 'hello']:
        session_manager.reset_session(sid)
        logger.info("Session is cleaned by hi")

    # TODO: Sync session data

    logger.info("Responding characters {}".format(responding_characters))
    _response = _ask_characters(responding_characters, question, lang, sid)

    context = {}
    for c in responding_characters:
        context.update(c.get_context(sid))
    for c in responding_characters:
        try:
            c.set_context(sid, context)
        except NotImplementedError:
            pass

    for c in responding_characters:
        try:
            c.check_reset_topic(sid)
        except Exception:
            continue

    if _response is not None:
        response.update(_response)
        logger.info("Ask {}, response {}".format(question, response))
        return response, SUCCESS
    else:
        logger.error("No pattern match")
        return response, NO_PATTERN_MATCH

def dump_history():
    session_manager.dump()

atexit.register(dump_history)

if __name__ == '__main__':
    sid = session_manager.start_session()
    for character in CHARACTERS:
        print ask('what is your name', 'en', sid)

