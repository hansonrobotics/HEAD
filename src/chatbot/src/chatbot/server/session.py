import threading
import time
import os
import datetime as dt
import logging
import uuid
from config import HISTORY_DIR, SESSION_REMOVE_TIMEOUT, SESSION_RESET_TIMEOUT
from response_cache import ResponseCache

logger = logging.getLogger('hr.chatbot.server.session')

class SessionData(object): pass

class Session(object):
    def __init__(self, sid):
        self.sid = sid
        self.sdata = SessionData()
        self.cache = ResponseCache()
        self.created = dt.datetime.now()
        self.init = self.created
        self.characters = []
        dirname = os.path.join(HISTORY_DIR, self.created.strftime('%Y%m%d'))
        self.fname = os.path.join(dirname, '{}.csv'.format(self.sid))
        self.removed = False
        self.active = False
        self.last_active_time = None

    def add(self, question, answer, **kwargs):
        if not self.removed:
            self.cache.add(question, answer, **kwargs)
            self.last_active_time = self.cache.last_time
            self.active = True
            return True
        return False

    def rate(self, rate, idx):
        return self.cache.rate(rate, idx)

    def reset(self):
        self.active = False
        self.dump()
        self.cache.clean()
        self.init = dt.datetime.now()
        for c in self.characters:
            try:
                c.refresh(self.sid)
            except NotImplementedError:
                pass

    def check(self, question, answer, lang):
        return self.cache.check(question, answer, lang)

    def dump(self):
        return self.cache.dump(self.fname)

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
        self._users = dict()
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
                logger.info("Resetted session {}".format(sid))

    def get_session(self, sid):
        if sid is not None:
            return self._sessions.get(sid, None)

    def get_sid(self, user):
        if user in self._users:
            sid = self._users.get(user)
            session = self._sessions.get(sid)
            if session:
                return sid

    def gen_sid(self):
        return str(uuid.uuid1())

    @_threadsafe
    def add_session(self, user, sid):
        if sid in self._sessions:
            return False
        self._sessions[sid] = Session(sid)
        self._users[user] = sid
        return True

    def start_session(self, user):
        _sid = self.get_sid(user)
        if _sid:
            return _sid
        sid = self.gen_sid()
        self.add_session(user, sid)
        return sid

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

class ChatSessionManager(SessionManager):
    def __init__(self, auto_clean=True):
        super(ChatSessionManager, self).__init__(auto_clean)

    def dump_all(self):
        fnames = []
        for sid, sess in self._sessions.iteritems():
            if sess and sess.dump():
                fnames.append(sess.fname)
        return fnames

    def dump(self, sid):
        fname = None
        sess = self._sessions.get(sid)
        if sess and sess.dump():
            fname = sess.fname
        return fname
