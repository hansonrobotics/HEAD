# -*- coding: utf-8 -*-
import logging
import random
import os
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import atexit
from collections import defaultdict

from threading import RLock
sync = RLock()

SUCCESS = 0
WRONG_CHARACTER_NAME = 1
NO_PATTERN_MATCH = 2
INVALID_SESSION = 3
INVALID_QUESTION = 4

useSOLR = False
logger = logging.getLogger('hr.chatbot.server.chatbot_agent')

from loader import load_characters
from config import CHARACTER_PATH
CHARACTERS = load_characters(CHARACTER_PATH)
REVISION = os.environ.get('HR_CHATBOT_REVISION')

from session import ChatSessionManager
session_manager = ChatSessionManager()
DISABLE_QUIBBLE = True

from chatbot.utils import shorten
from chatbot.server.character import TYPE_AIML, TYPE_CS

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
    # TODO: Update character
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
    if not characters:
        logger.warn('No character is satisfied')
    return characters


def list_character(lang, sid):
    sess = session_manager.get_session(sid)
    if sess is None:
        return []
    responding_characters = get_responding_characters(lang, sid)
    if hasattr(sess.sdata, 'weights'):
        return [(c.id, w, c.level, c.dynamic_level) for c, w in zip(
                responding_characters, sess.sdata.weights)]
    else:
        return [(c.id, c.weight, c.level, c.dynamic_level) for c in responding_characters]


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

def set_context(prop, sid):
    sess = session_manager.get_session(sid)
    if sess is None:
        return False, "No session"
    for c in CHARACTERS:
        try:
            c.set_context(sess, prop)
        except Exception:
            pass
    return True, "Context is updated"

def preprocessing(question):
    question = question.lower().strip()
    question = ' '.join(question.split())  # remove consecutive spaces
    question = question.replace('sofia', 'sophia')
    return question

def _ask_characters(characters, question, lang, sid, query):
    sess = session_manager.get_session(sid)
    if sess is None:
        return

    used_charaters = []
    data = sess.get_session_data()
    user = getattr(data, 'user')
    botname = getattr(data, 'botname')
    if hasattr(data, 'weights'):
        weights = data.weights
    else:
        weights = [c.weight for c in characters]
    weighted_characters = zip(characters, weights)

    _question = preprocessing(question)
    response = {}
    hit_character = None
    answer = None
    cross_trace = []

    cached_responses = defaultdict(list)

    control = get_character('control')
    if control is not None:
        _response = control.respond(_question, lang, sess, query=True)
        cross_trace.append((control.id, 'control', _response.get('trace') or 'No trace'))
        if _response.get('text') == '[tell me more]':
            if sess.last_used_character:
                if sess.cache.that_question is None:
                    sess.cache.that_question = sess.cache.last_question
                context = sess.last_used_character.get_context(sess)
                if 'continue' in context and context.get('continue'):
                    _answer, res = shorten(context.get('continue'), 140)
                    response['text'] = answer = _answer
                    response['botid'] = sess.last_used_character.id
                    response['botname'] = sess.last_used_character.name
                    sess.last_used_character.set_context(sess, {'continue': res})
                    hit_character = sess.last_used_character
                    cross_trace.append((sess.last_used_character.id, 'continuation', 'Non-empty'))
                else:
                    _question = sess.cache.that_question.lower().strip()
                    cross_trace.append((sess.last_used_character.id, 'continuation', 'Empty'))
        else:
            for c in characters:
                try:
                    c.remove_context(sess, 'continue')
                except NotImplementedError:
                    pass
            sess.cache.that_question = None

    # If the last input is a question, then try to use the same tier to
    # answer it.
    if not answer:
        if sess.open_character and sess.open_character in characters:
            logger.info("Using open dialog character {}".format(sess.open_character.id))
            response = sess.open_character.respond(_question, lang, sess, query)
            used_charaters.append(sess.open_character.id)
            answer = response.get('text', '').strip()
            if answer and not response.get('bad') and not response.get('gambit'):
                hit_character = sess.open_character
                cross_trace.append((sess.open_character.id, 'question', response.get('trace') or 'No trace'))
            else:
                if response.get('repeat'):
                    cross_trace.append((sess.open_character.id, 'question', 'Repetitive answer'))
                    cached_responses['repeat'].append((response, response.get('repeat'), sess.open_character))
                elif response.get('bad'):
                    cross_trace.append((sess.open_character.id, 'question', 'Bad answer'))
                else:
                    cross_trace.append((sess.open_character.id, 'question', 'No answer'))

    # Try the first tier to see if there is good match
    if not answer:
        c, weight = weighted_characters[0]
        if c.type == TYPE_AIML:
            _response = c.respond(_question, lang, sess, query=True)
            if _response.get('exact_match') or _response.get('ok_match'):
                logger.info("{} has good match".format(c.id))
                response = c.respond(_question, lang, sess, query)
                used_charaters.append(c.id)
                _answer = response.get('text', '').strip()
                if _answer:
                    if random.random() < weight:
                        hit_character = c
                        answer = _answer
                        cross_trace.append((c.id, 'priority', response.get('trace') or 'No trace'))
                    else:
                        cross_trace.append((c.id, 'priority', 'Pass through'))
                        cached_responses['pass'].append((response, _answer, c))
                else:
                    cross_trace.append((c.id, 'priority', 'No answer'))
            else:
                logger.info("{} has no good match".format(c.id))
                cross_trace.append((c.id, 'priority', 'No good match: {}'.format(_response.get('trace') or 'No trace')))

    # Check the last used character
    if not answer:
        if sess.last_used_character and sess.last_used_character.dynamic_level:
            for c, weight in weighted_characters:
                if sess.last_used_character.id == c.id:
                    _response = c.respond(_question, lang, sess, query=True)
                    if _response.get('exact_match') or _response.get('ok_match'):
                        logger.info("Last used tier {} has good match".format(c.id))

                        if sess.last_used_character.type == TYPE_CS:
                            response = _response
                        else:
                            response = c.respond(_question, lang, sess, query)
                        used_charaters.append(c.id)
                        _answer = response.get('text', '').strip()
                        if _answer:
                            if random.random() < weight:
                                hit_character = c
                                answer = _answer
                                cross_trace.append((c.id, 'last used', response.get('trace') or 'No trace'))
                            else:
                                cross_trace.append((c.id, 'last used', 'Pass through'))
                                cached_responses['pass'].append((response, _answer, c))
                        else:
                            if response.get('repeat'):
                                cross_trace.append((c.id, 'last used', 'Repetitive answer'))
                                cached_responses['repeat'].append((response, response.get('repeat'), c))
                            else:
                                cross_trace.append((c.id, 'last used', 'No answer'))
                    else:
                        logger.info("{} has no good match".format(c.id))
                        cross_trace.append((c.id, 'last used', 'No good match: {}'.format(_response.get('trace') or 'No trace')))
                        #cached_responses['bad'].append((response, response.get('text'), c))
                    break

    # Check the loop
    if not answer:
        for c, weight in weighted_characters:
            if weight == 0:
                logger.info("Ignore zero weighted character {}".format(c.id))
                continue

            if c.id in used_charaters:
                logger.info("Ignore used tiers {}".format(c.id))
                continue

            response = c.respond(_question, lang, sess, query)
            used_charaters.append(c.id)
            assert isinstance(response, dict), "Response must be a dict"

            _answer = response.get('text', '').strip()
            if not _answer:
                if response.get('repeat'):
                    cross_trace.append((c.id, 'loop', 'Repetitive answer'))
                    cached_responses['repeat'].append((response, response.get('repeat'), c))
                else:
                    cross_trace.append((c.id, 'loop', 'No answer'))
                continue

            if response.get('bad'):
                cross_trace.append((c.id, 'loop', 'Bad answer'))
                continue

            if DISABLE_QUIBBLE and response.get('quibble'):
                logger.info("Ignore quibbled answer by {}".format(c.id))
                cross_trace.append((c.id, 'loop', 'Quibble answer'))
                cached_responses['quibble'].append((response, _answer, c))
                continue

            if response.get('gambit'):
                if random.random() > 0.3:
                    cached_responses['gambit'].append((response, _answer, c))
                    cross_trace.append((c.id, 'loop', 'Ignore gambit answer'))
                    logger.info("Ignore gambit response")
                    continue

            if 'pickup' in c.id:
                cached_responses['pickup'].append((response, _answer, c))

            # Each tier has weight*100% chance to be selected.
            # If the chance goes to the last tier, it will be selected anyway.
            if random.random() < weight:
                answer = _answer
                hit_character = c
                cross_trace.append((c.id, 'loop', response.get('trace') or 'No trace'))
                break
            else:
                cross_trace.append((c.id, 'loop', 'Pass through'))
                if 'pickup' not in c.id and 'markov' not in c.id:
                    cached_responses['pass'].append((response, _answer, c))

    if not answer:
        for response_type in ['pass', 'quibble', 'repeat', 'gambit', 'pickup']:
            if cached_responses.get(response_type):
                response, answer, hit_character = cached_responses.get(response_type)[0]
                response['text'] = answer
                cross_trace.append(
                    (hit_character.id, response_type,
                    response.get('trace') or 'No trace'))
                break

    dummy_character = get_character('dummy', lang)
    if not answer and dummy_character:
        if response.get('repeat'):
            response = dummy_character.respond("REPEAT_ANSWER", lang, sid, query)
        else:
            response = dummy_character.respond("NO_ANSWER", lang, sid, query)
        hit_character = dummy_character
        answer = response.get('text', '').strip()

    if not query and hit_character is not None:
        sess.add(question, answer, AnsweredBy=hit_character.id,
                    User=user, BotName=botname, Trace=cross_trace,
                    Revision=REVISION, Lang=lang, ModQuestion=_question)

        sess.last_used_character = hit_character

        if answer.lower().strip().endswith('?'):
            if hit_character.dynamic_level:
                sess.open_character = hit_character
                logger.info("Set open dialog character {}".format(
                            hit_character.id))
        else:
            sess.open_character = None

    response['trace'] = cross_trace
    return response

def get_responding_characters(lang, sid):
    sess = session_manager.get_session(sid)
    if sess is None:
        return []
    if not hasattr(sess.sdata, 'botname'):
        return []

    botname = sess.sdata.botname
    user = sess.sdata.user

    # current character > local character with the same name > solr > generic
    responding_characters = get_characters_by_name(
        botname, local=False, lang=lang, user=user)
    responding_characters = sorted(responding_characters, key=lambda x: x.level)

    character = None
    if responding_characters:
        character = responding_characters[0]
    else:
        return []

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


def rate_answer(sid, idx, rate):
    sess = session_manager.get_session(sid)
    if sess is None:
        logger.error("Session doesn't exist")
        return False
    try:
        return sess.rate(rate, idx)
    except Exception as ex:
        logger.error("Rate error: {}".format(ex))
        return False
    return True


def ask(question, lang, sid, query=False):
    """
    return (response dict, return code)
    """
    response = {'text': '', 'emotion': '', 'botid': '', 'botname': ''}

    sess = session_manager.get_session(sid)
    if sess is None:
        return response, INVALID_SESSION

    if not question or not question.strip():
        return response, INVALID_QUESTION

    responding_characters = get_responding_characters(lang, sid)
    if not responding_characters:
        logger.error("Wrong characer name")
        return response, WRONG_CHARACTER_NAME

    for c in responding_characters:
        if c.is_command(question):
            response.update(c.respond(question, lang, sess, query))
            return response, SUCCESS

    sess.set_characters(responding_characters)
    if question and question.lower().strip() in ['hi', 'hello']:
        session_manager.dump(sid)
        session_manager.reset_session(sid)
        logger.info("Session is cleaned by hi")
    if question and question.lower().strip() in ["what's new"]:
        sess.last_used_character = None
        sess.open_character = None
        logger.info("Triggered new topic")

    logger.info("Responding characters {}".format(responding_characters))
    _response = _ask_characters(
        responding_characters, question, lang, sid, query)

    if not query:
        # Sync session data
        if sess.last_used_character is not None:
            context = sess.last_used_character.get_context(sess)
            for c in responding_characters:
                if c.id == sess.last_used_character.id:
                    continue
                try:
                    c.set_context(sess, context)
                except NotImplementedError:
                    pass

            for c in responding_characters:
                if c.type != TYPE_AIML:
                    continue
                try:
                    c.check_reset_topic(sid)
                except Exception:
                    continue

        if 'goodbye' in question.lower().split() or \
            'see you' in question.lower().split() or \
            'bye' in question.lower().split():
            session_manager.remove_session(sid)
            logger.info("Session {} is removed by goodbye".format(sid))

    if _response is not None and _response.get('text'):
        response.update(_response)
        logger.info("Ask {}, response {}".format(question, response))
        return response, SUCCESS
    else:
        logger.error("No pattern match")
        return response, NO_PATTERN_MATCH


def dump_history():
    return session_manager.dump_all()


def dump_session(sid):
    return session_manager.dump(sid)


def reload_characters(**kwargs):
    global CHARACTERS, REVISION
    with sync:
        characters = None
        logger.info("Reloading")
        try:
            characters = load_characters(CHARACTER_PATH)
            del CHARACTERS[:]
            CHARACTERS = characters
            revision = kwargs.get('revision')
            if revision:
                REVISION = revision
                logger.info("Revision {}".format(revision))
        except Exception as ex:
            logger.error("Reloading characters error {}".format(ex))

def rebuild_cs_character(**kwargs):
    with sync:
        try:
            for c in CHARACTERS:
                if c.id == 'cs' and hasattr(c, 'rebuild'):
                    log = c.rebuild()
                    if 'ERROR SUMMARY' in log:
                        logger.error(log[log.index('ERROR SUMMARY'):])
        except Exception as ex:
            logger.error("Rebuilding chatscript characters error {}".format(ex))

atexit.register(dump_history)
