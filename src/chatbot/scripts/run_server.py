#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import os
import logging
import datetime as dt
import json
import shutil
import argparse

try:
    import colorlog
except ImportError:
    pass

import sys
import re
CWD = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(CWD, '../src'))

if 'HR_CHARACTER_PATH' not in os.environ:
    os.environ['HR_CHARACTER_PATH'] = os.path.join(CWD, 'characters')

from chatbot.server.config import SERVER_LOG_DIR, HISTORY_DIR

def init_logging():
    if not os.path.isdir(SERVER_LOG_DIR):
        os.makedirs(SERVER_LOG_DIR)
    log_config_file = '{}/{}.log'.format(
        SERVER_LOG_DIR,
        dt.datetime.strftime(dt.datetime.now(), '%Y%m%d%H%M%S'))
    link_log_fname = os.path.join(SERVER_LOG_DIR, 'latest.log')
    if os.path.islink(link_log_fname):
        os.unlink(link_log_fname)
    os.symlink(log_config_file, link_log_fname)
    formatter = logging.Formatter(
        '[%(name)s][%(levelname)s] %(asctime)s: %(message)s')
    fh = logging.FileHandler(log_config_file)
    fh.setFormatter(formatter)
    sh = logging.StreamHandler()
    if 'colorlog' in sys.modules and os.isatty(2):
        cformat = '%(log_color)s' + formatter._fmt
        formatter = colorlog.ColoredFormatter(
            cformat,
            log_colors={
                'DEBUG':'reset',
                'INFO': 'reset',
                'WARNING': 'yellow',
                'ERROR': 'red',
                'CRITICAL': 'bold_red',
            }
        )
    sh.setFormatter(formatter)
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    root_logger.addHandler(fh)
    root_logger.addHandler(sh)
    return sh, fh

sh, fh = init_logging()
from chatbot.server.auth import requires_auth
from chatbot.server.auth import check_auth, authenticate

from flask import Flask, request, Response, send_from_directory

from chatbot.server.chatbot_agent import (
    ask, list_character, session_manager, set_weights, set_context,
    dump_history, dump_session, add_character, list_character_names,
    rate_answer, get_context, said, remove_context, update_config)
from chatbot.stats import history_stats

json_encode = json.JSONEncoder().encode
app = Flask(__name__)
VERSION = 'v1.1'
ROOT = '/{}'.format(VERSION)
INCOMING_DIR = os.path.expanduser('~/.hr/aiml/incoming')

logger = logging.getLogger('hr.chatbot.server')
app.config['UPLOAD_FOLDER'] = os.path.expanduser('~/.hr/aiml')


@app.route(ROOT + '/<path:path>')
def send_client(path):
    return send_from_directory('public', path)


@app.route(ROOT + '/client', methods=['GET'])
def client():
    return send_from_directory('public', 'client.html')


@app.route(ROOT + '/chat', methods=['GET'])
@requires_auth
def _chat():
    data = request.args
    question = data.get('question')
    session = data.get('session')
    lang = data.get('lang', 'en')
    query = data.get('query', 'false')
    query = query.lower() == 'true'
    request_id = request.headers.get('X-Request-Id')
    marker = data.get('marker', 'default')
    response, ret = ask(
        question, lang, session, query, request_id=request_id, marker=marker)
    return Response(json_encode({'ret': ret, 'response': response}),
                    mimetype="application/json")


@app.route(ROOT + '/batch_chat', methods=['POST'])
def _batch_chat():
    auth = request.form.get('Auth')
    if not auth or not check_auth(auth):
        return authenticate()

    questions = request.form.get('questions')
    questions = json.loads(questions)
    session = request.form.get('session')
    lang = request.form.get('lang', 'en')
    responses = []
    for idx, question in questions:
        response, ret = ask(str(question), lang, session)
        responses.append((idx, response, ret))
    return Response(json_encode({'ret': 0, 'response': responses}),
                    mimetype="application/json")

@app.route(ROOT + '/said', methods=['GET'])
@requires_auth
def _said():
    data = request.args
    session = data.get('session')
    message = data.get('message')
    ret, response = said(session, message)
    return Response(json_encode({'ret': ret, 'response': response}),
                    mimetype="application/json")

@app.route(ROOT + '/rate', methods=['GET'])
@requires_auth
def _rate():
    data = request.args
    response = ''
    try:
        ret = rate_answer(data.get('session'), int(
            data.get('index')), data.get('rate'))
    except Exception as ex:
        response = ex.message
    return Response(json_encode({'ret': ret, 'response': response}),
                    mimetype="application/json")


@app.route(ROOT + '/chatbots', methods=['GET'])
@requires_auth
def _chatbots():
    data = request.args
    lang = data.get('lang', None)
    session = data.get('session')
    characters = list_character(lang, session)
    return Response(json_encode({'ret': 0, 'response': characters}),
                    mimetype="application/json")


@app.route(ROOT + '/bot_names', methods=['GET'])
@requires_auth
def _bot_names():
    names = list_character_names()
    return Response(json_encode({'ret': 0, 'response': names}),
                    mimetype="application/json")


@app.route(ROOT + '/start_session', methods=['GET'])
@requires_auth
def _start_session():
    botname = request.args.get('botname')
    user = request.args.get('user')
    test = request.args.get('test', 'false')
    refresh = request.args.get('refresh', 'false')
    test = test.lower() == 'true'
    refresh = refresh.lower() == 'true'
    sid = session_manager.start_session(
        user=user, key=botname, test=test, refresh=refresh)
    sess = session_manager.get_session(sid)
    sess.sdata.botname = botname
    sess.sdata.user = user
    return Response(json_encode({'ret': 0, 'sid': str(sid)}),
                    mimetype="application/json")


@app.route(ROOT + '/sessions', methods=['GET'])
@requires_auth
def _sessions():
    sessions = session_manager.list_sessions()
    return Response(json_encode({'ret': 0, 'response': sessions}),
                    mimetype="application/json")

@app.route(ROOT + '/set_weights', methods=['GET'])
@requires_auth
def _set_weights():
    data = request.args
    lang = data.get('lang', None)
    param = data.get('param')
    sid = data.get('session')
    ret, response = set_weights(param, lang, sid)
    if ret:
        sess = session_manager.get_session(sid)
        if sess and hasattr(sess.sdata, 'weights'):
            logger.info("Set weights {} successfully".format(sess.sdata.weights))
    else:
        logger.info("Set weights failed.")
    return Response(json_encode({'ret': ret, 'response': response}),
                    mimetype="application/json")

@app.route(ROOT + '/set_context', methods=['GET'])
@requires_auth
def _set_context():
    data = request.args
    context_str = data.get('context')
    context = {}
    for tok in context_str.split(','):
        k, v = tok.split('=')
        context[k.strip()] = v.strip()
    sid = data.get('session')
    ret, response = set_context(context, sid)
    return Response(json_encode({'ret': ret, 'response': response}),
                    mimetype="application/json")


@app.route(ROOT + '/remove_context', methods=['GET'])
@requires_auth
def _remove_context():
    data = request.args
    keys = data.get('keys')
    keys = keys.split(',')
    sid = data.get('session')
    ret, response = remove_context(keys, sid)
    return Response(json_encode({'ret': ret, 'response': response}),
                    mimetype="application/json")

@app.route(ROOT + '/get_context', methods=['GET'])
@requires_auth
def _get_context():
    data = request.args
    sid = data.get('session')
    lang = data.get('lang', 'en')
    ret, response = get_context(sid, lang)
    return Response(json_encode({'ret': ret, 'response': response}),
                    mimetype="application/json")

@app.route(ROOT + '/update_config', methods=['GET'])
@requires_auth
def _update_config():
    data = request.args.to_dict()
    for k, v in data.iteritems():
        if v.lower() == 'true':
            data[k]=True
        elif v.lower() == 'false':
            data[k]=False
        elif re.match(r'[0-9]+', v):
            data[k]=int(v)
        elif re.match(r'[0-9]+\.[0-9]+', v):
            data[k]=float(v)
        else:
            data[k]=str(v)
    ret, response = update_config(**data)
    return Response(json_encode({'ret': ret, 'response': response}),
                    mimetype="application/json")


@app.route(ROOT + '/upload_character', methods=['POST'])
def _upload_character():
    auth = request.form.get('Auth')
    if not auth or not check_auth(auth):
        return authenticate()
    try:
        user = request.form.get('user')
        zipfile = request.files.get('zipfile')
        lang = request.files.get('lang')

        saved_dir = os.path.join(INCOMING_DIR, user)
        if not os.path.isdir(saved_dir):
            os.makedirs(saved_dir)

        # Clean the incoming directory
        for f in os.listdir(saved_dir):
            f = os.path.join(saved_dir, f)
            if os.path.isfile(f):
                os.unlink(f)
            else:
                shutil.rmtree(f, True)

        saved_zipfile = os.path.join(saved_dir, zipfile.filename)
        zipfile.save(saved_zipfile)
        logger.info("Get zip file {}".format(zipfile.filename))

        from loader import AIMLCharacterZipLoader
        characters = AIMLCharacterZipLoader.load(
            saved_zipfile, saved_dir, 'upload')
        ret, response = True, "Done"
        for character in characters:
            character.local = False
            character.id = '{}/{}'.format(user, character.id)
            _ret, _response = add_character(character)
            if not _ret:
                ret = _ret
                response = _response
        os.remove(saved_zipfile)

        return Response(json_encode({
            'ret': ret,
            'response': response
        }),
            mimetype="application/json")

    except Exception as ex:
        return Response(json_encode({
            'ret': False,
            'response': str(ex)
        }),
            mimetype="application/json")


@app.route('/log')
def _log():
    def generate():
        with open(LOG_CONFIG_FILE) as f:
            for row in f:
                yield row
    return Response(generate(), mimetype='text/plain')


@app.route(ROOT + '/reset_session', methods=['GET'])
@requires_auth
def _reset_session():
    data = request.args
    sid = data.get('session')
    if session_manager.has_session(sid):
        session_manager.reset_session(sid)
        ret, response = True, "Session reset"
    else:
        ret, response = False, "No such session"
    return Response(json_encode({
        'ret': ret,
        'response': response
    }),
        mimetype="application/json")


@app.route(ROOT + '/dump_history', methods=['GET'])
def _dump_history():
    try:
        dump_history()
        ret, response = True, "Success"
    except Exception:
        ret, response = False, "Failure"
    return Response(json_encode({
        'ret': ret,
        'response': response
    }),
        mimetype="application/json")


@app.route(ROOT + '/dump_session', methods=['GET'])
@requires_auth
def _dump_session():
    try:
        data = request.args
        sid = data.get('session')
        fname = dump_session(sid)
        session_manager.remove_session(sid)
        if fname is not None and os.path.isfile(fname):
            return send_from_directory(os.path.dirname(fname), os.path.basename(fname))
        else:
            return '', 404
    except Exception as ex:
        logger.error("Dump error {}".format(ex))
        return '', 500


@app.route(ROOT + '/chat_history', methods=['GET'])
@requires_auth
def _chat_history():
    history_stats(HISTORY_DIR, 7)
    history_file = os.path.join(HISTORY_DIR, 'last_7_days.csv')
    if os.path.isfile(history_file):
        return send_from_directory(HISTORY_DIR, os.path.basename(history_file))
    else:
        return '', 404

@app.route(ROOT + '/session_history', methods=['GET'])
@requires_auth
def _session_history():
    try:
        data = request.args
        sid = data.get('session')
        sess = session_manager.get_session(sid)
        fname = sess.dump_file
        if fname is not None and os.path.isfile(fname):
            return send_from_directory(
                os.path.dirname(fname),
                os.path.basename(fname),
                mimetype='text/plain')
        else:
            return '', 404
    except Exception as ex:
        logger.error("Internal error {}".format(ex))
        return '', 500


@app.route(ROOT + '/ping', methods=['GET'])
def _ping():
    return Response(json_encode({'ret': 0, 'response': 'pong'}),
                    mimetype="application/json")


@app.route(ROOT + '/stats', methods=['GET'])
@requires_auth
def _stats():
    try:
        data = request.args
        days = int(data.get('lookback', 7))
        dump_history()
        response = history_stats(HISTORY_DIR, days)
        ret = True
    except Exception as ex:
        ret, response = False, {'err_msg': str(ex)}
        logger.error(ex)
    return Response(json_encode({'ret': ret, 'response': response}),
                    mimetype="application/json")

def main():
    parser = argparse.ArgumentParser('Chatbot Server')

    parser.add_argument(
        '-p, --port',
        dest='port', default=8001, help='Server port')
    parser.add_argument(
        '-v, --verbose',
        dest='verbose', action='store_true', help='Verbose')

    option = parser.parse_args()

    if option.verbose:
        fh.setLevel(logging.INFO)
        sh.setLevel(logging.INFO)
    else:
        fh.setLevel(logging.INFO)
        sh.setLevel(logging.WARN)

    if 'HR_CHATBOT_SERVER_EXT_PATH' in os.environ:
        sys.path.insert(0, os.path.expanduser(
            os.environ['HR_CHATBOT_SERVER_EXT_PATH']))
        import ext
        ext.load(app, ROOT)
    app.run(host='0.0.0.0', debug=False, use_reloader=False, port=option.port)


if __name__ == '__main__':
    main()
