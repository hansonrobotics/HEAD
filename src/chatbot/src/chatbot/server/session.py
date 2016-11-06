import threading
import time
import os
import datetime as dt
import logging
import uuid
from config import HISTORY_DIR, TEST_HISTORY_DIR, SESSION_REMOVE_TIMEOUT, SESSION_RESET_TIMEOUT
from response_cache import ResponseCache
from collections import defaultdict
from chatbot.server.character import TYPE_AIML

logger = logging.getLogger('hr.chatbot.server.session')


class SessionData(object):
    pass


class Session(object):

    def __init__(self, sid):
        self.sid = sid
        self.sdata = SessionData()
        self.cache = ResponseCache()
        self.created = dt.datetime.now()
        self.init = self.created
        self.characters = []
        dirname = os.path.join(HISTORY_DIR, self.created.strftime('%Y%m%d'))
        test_dirname = os.path.join(
            TEST_HISTORY_DIR, self.created.strftime('%Y%m%d'))
        self.fname = os.path.join(dirname, '{}.csv'.format(self.sid))
        self.test_fname = os.path.join(test_dirname, '{}.csv'.format(self.sid))
        self.dump_file = None
        self.removed = False
        self.active = False
        self.last_active_time = None
        self.test = False
        self.last_used_character = None
        self.open_character = None

    def set_test(self, test):
        if test:
            logger.info("Set test session")
        self.test = test

    def add(self, question, answer, **kwargs):
        if not self.removed:
            self.cache.add(question, answer, **kwargs)
            self.last_active_time = self.cache.last_time
            self.active = True
            return True
        return False

    def rate(self, rate, idx):
        return self.cache.rate(rate, idx)

    def set_characters(self, characters):
        self.characters = characters
        for c in self.characters:
            if c.type != TYPE_AIML:
                continue
            prop = c.get_properties()
            context = {}
            for key in ['weather', 'location', 'temperature']:
                if key in prop:
                    context[key] = prop.get(key)
            now = dt.datetime.now()
            context['time'] = dt.datetime.strftime(now, '%I:%M %p')
            context['date'] = dt.datetime.strftime(now, '%B %d %Y')
            try:
                c.set_context(self.sid, context)
            except Exception as ex:
                pass

    def reset(self):
        self.active = False
        self.dump()
        self.cache.clean()
        self.init = dt.datetime.now()
        self.last_used_character = None
        self.open_character = None
        for c in self.characters:
            try:
                c.refresh(self.sid)
            except NotImplementedError:
                pass

    def check(self, question, answer):
        return self.cache.check(question, answer)

    def dump(self):
        if self.test:
            self.dump_file = self.test_fname
        else:
            self.dump_file = self.fname
        return self.cache.dump(self.dump_file)

    def get_session_data(self):
        return self.sdata

    def since_idle(self, since):
        if self.last_active_time is not None:
            return (since - self.last_active_time).total_seconds()
        else:
            return (since - self.created).total_seconds()

    def __repr__(self):
        return "<Session {} init {} active {}>".format(
            self.sid, self.init, self.cache.last_time)


class Locker(object):

    def __init__(self):
        self._lock = threading.RLock()

    def lock(self):
        self._lock.acquire()

    def unlock(self):
        self._lock.release()


class SessionManager(object):

    def __init__(self, auto_clean=True):
        self._sessions = dict()
        self._users = defaultdict(dict)
        self._locker = Locker()
        self._session_cleaner = threading.Thread(
            target=self._clean_sessions, name="SessionCleaner")
        self._session_cleaner.daemon = True
        if auto_clean:
            self._session_cleaner.start()

    def _threadsafe(f):
        def wrap(self, *args, **kwargs):
            self._locker.lock()
            try:
                return f(self, *args, **kwargs)
            finally:
                self._locker.unlock()
        return wrap

    @_threadsafe
    def remove_session(self, sid):
        if sid in self._sessions:
            session = self._sessions.pop(sid)
            session.dump()
            session.removed = True
            del session
            logger.info("Removed session {}".format(sid))

    def reset_session(self, sid):
        if sid in self._sessions:
            session = self._sessions.get(sid)
            if session.active:
                session.reset()
                logger.info("Reset session {}".format(sid))

    def get_session(self, sid):
        if sid is not None:
            return self._sessions.get(sid, None)

    def get_sid(self, user, key):
        if user in self._users:
            sessions = self._users.get(user)
            if sessions:
                sid = sessions.get(key)
                session = self._sessions.get(sid)
                if session:
                    return sid

    def gen_sid(self):
        return str(uuid.uuid1())

    @_threadsafe
    def add_session(self, user, key, sid):
        if sid in self._sessions:
            return False
        self._sessions[sid] = Session(sid)
        self._users[user][key] = sid
        return True

    def start_session(self, user, key, test=False, refresh=False):
        """
        user: username
        key: a string to identify session in user scope
        test: if it's a session for test
        refresh: if true, it will generate new session id
        """
        _sid = self.get_sid(user, key)
        if _sid and refresh:
            self.remove_session(_sid)
            _sid = None
        if not _sid:
            _sid = self.gen_sid()
            self.add_session(user, key, _sid)
        session = self.get_session(_sid)
        assert(session is not None)
        session.set_test(test)
        return _sid

    def has_session(self, sid):
        return sid in self._sessions

    def _clean_sessions(self):
        while True:
            reset_sessions, remove_sessions = [], []
            since = dt.datetime.now()
            for sid, s in self._sessions.iteritems():
                if SESSION_RESET_TIMEOUT < s.since_idle(since) < SESSION_REMOVE_TIMEOUT:
                    reset_sessions.append(sid)
                if s.since_idle(since) > SESSION_REMOVE_TIMEOUT:
                    remove_sessions.append(sid)
            for sid in reset_sessions:
                self.reset_session(sid)
            for sid in remove_sessions:
                self.remove_session(sid)
            time.sleep(0.1)

    def list_sessions(self):
        return self._sessions.keys()


class ChatSessionManager(SessionManager):

    def __init__(self, auto_clean=True):
        super(ChatSessionManager, self).__init__(auto_clean)

    def dump_all(self):
        fnames = []
        for sid, sess in self._sessions.iteritems():
            if sess and sess.dump():
                fnames.append(sess.dump_file)
        return fnames

    def dump(self, sid):
        fname = None
        sess = self._sessions.get(sid)
        if sess and sess.dump():
            fname = sess.dump_file
        return fname
