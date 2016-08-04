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

    def add(self, question, answer, **kwargs):
        if not self.removed:
            self.cache.add(question, answer, **kwargs)
            return True
        return False

    def rate(self, rate, idx):
        self.cache.rate(rate, idx)

    def reset(self):
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
        if self.cache.last_time is not None:
            return (since - self.cache.last_time).total_seconds()
        else:
            return (since - self.created).total_seconds()

    def since_reset(self, since):
        return (since - self.init).total_seconds()

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
    def _add_session(self, sid):
        if sid in self._sessions: return
        self._sessions[sid] = Session(sid)

    @_threadsafe
    def remove_session(self, sid):
        if sid in self._sessions:
            session = self._sessions.pop(sid)
            session.dump()
            session.removed = True
            del session

    def reset_session(self, sid):
        if sid in self._sessions:
            session = self._sessions.get(sid)
            session.reset()

    def get_session(self, sid):
        if sid is not None:
            return self._sessions.get(sid, None)

    @_threadsafe
    def start_session(self, user):
        if user in self._users:
            sid = self._users.get(user)
            session = self._sessions.get(sid)
            if session:
                return sid
        sid = str(uuid.uuid1())
        self._add_session(sid)
        self._users[user] = sid
        return sid

    def has_session(self, sid):
        return sid in self._sessions

    def _clean_sessions(self):
        while True:
            reset_sessions, remove_sessions = [], []
            since = dt.datetime.now()
            for sid, s in self._sessions.iteritems():
                if s.since_reset(since) > SESSION_RESET_TIMEOUT:
                    reset_sessions.append(sid)
                if s.since_idle(since) > SESSION_REMOVE_TIMEOUT:
                    remove_sessions.append(sid)
            for sid in reset_sessions:
                self.reset_session(sid)
                logger.info("Reset session {}".format(sid))
            for sid in remove_sessions:
                self.remove_session(sid)
                logger.info("Removed session {}".format(sid))
            time.sleep(0.1)

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

if __name__ == '__main__':
    session_manager = SessionManager()
    sid = session_manager.start_session('test')
    session = session_manager.get_session(sid)
    print session
    sid = session_manager.start_session('test')
    session = session_manager.get_session(sid)
    session.add("hello", "hello, how are you")
    print session
    time.sleep(4)
    session = session_manager.get_session(sid)
    print session
    time.sleep(4)
    session = session_manager.get_session(sid)
    print session
