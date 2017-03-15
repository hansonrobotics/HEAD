import logging
import re
import socket

from chatbot.utils import Locker

logger = logging.getLogger('hr.chatbot.server.csclient')

class CSClient(object):

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self._locker = Locker()
        self.variable_pattern = re.compile(""".* variable: .*\$(?P<name>\S+) = (?P<value>.*)""")

    def set_host(self, host):
        self.host = host

    def set_port(self, port):
        self.port = port

    def _threadsafe(f):
        def wrap(self, *args, **kwargs):
            self._locker.lock()
            try:
                return f(self, *args, **kwargs)
            finally:
                self._locker.unlock()
        return wrap

    @_threadsafe
    def send(self, sid, msg):
        request = "{sid}\0\0{msg}\0".format(sid=sid, msg=msg)
        response =  ''
        connection = None
        try:
            connection = socket.create_connection((self.host, self.port), timeout=2)
            ret = connection.sendall(request)
            try:
                while True:
                    chunk = connection.recv(4096)
                    if chunk:
                        response += chunk
                    else:
                        break
            except socket.timeout as e:
                logger.error("Timeout {}".format(e))
                self.kill_server()
                raise e
        except Exception as ex:
            logger.error("Connection error {}".format(ex))
        finally:
            if connection is not None:
                connection.close()

        if 'No such bot' in response:
            logger.error(response)
            response = ''

        return response

    def kill_server(self):
        out = subprocess.check_output(['ps', '-ef'])
        for l in out.splitlines():
            if 'ChatScript' in l:
                pid = int(l.split()[1])
                os.kill(pid, 9)
                logger.info("Killed ChatScript server pid {}".format(pid))

    def get_variables(self, sid='.'):
        response = self.send(sid, ':variables')
        context = {}
        for line in response.splitlines():
            matchobj = self.variable_pattern.match(line)
            if matchobj:
                name, value =  matchobj.groups()
                if name.startswith('cs_'):
                    continue
                context[name] = value.replace('_', ' ')
        return context

    def set_variables(self, sid, context):
        for k, v in context.iteritems():
            if k.startswith('cs_'):
                continue
            v = ' '.join(v.split()).replace(' ', '_') # variables in CS use _ instead of space
            response = self.send(sid, ':do ${}={}'.format(k, v))


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)

    sid = 'xxx'
    client = CSClient(host='localhost', port='1024')
    print '-----------------'
    client.send(sid, ':reset')
    print client.send(sid, 'do you like humans')
    print client.send(sid, ':redo do you like humans')
    print client.get_variables(sid)
    print client.set_variables(sid, {'xxx': 'yyy'})
    print client.get_variables(sid)
