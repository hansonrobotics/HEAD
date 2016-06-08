import logging
import os
import json
from flask import request

logger = logging.getLogger('hr.webui.monitor')
json_encode = json.JSONEncoder().encode
LOG_TO_SHOW = 100

def get_logs(loglevel):
    """
    Collect the logs from the ros log files.
    If there is no roscore process running, then it displays the logs
    from the last run.
    """
    from roslaunch.roslaunch_logs import get_run_id
    import rospkg
    import glob
    import re

    logger.debug('get logs: log level {}'.format(loglevel))
    log_cursors = request.form.copy()
    logger.debug('cursors: {}'.format(log_cursors))

    log_root = rospkg.get_log_dir()
    run_id = get_run_id()
    roscore_running = True
    if not run_id:
        roscore_running = False
        subdirs = [os.path.join(log_root, d) for d in os.listdir(log_root)
                    if os.path.isdir(os.path.join(log_root, d))]
        if subdirs:
            run_id = max(subdirs, key=os.path.getmtime)
        else:
            run_id = ''

    # some extra log files that not created by roslaunch
    extra_log_files = [os.path.join(log_root, name) for name in [
        'ros_motors_webui.log', 'sophia_Eva_Behavior.log', 'blender_api.log']]
    extra_log_files = [f for f in extra_log_files if os.path.isfile(f)]

    log_dir = os.path.join(log_root, run_id)
    log_files = glob.glob(os.path.join(log_dir, '*.log'))
    log_files += extra_log_files

    # ignore stdout log files
    log_files = [log_file for log_file in log_files if
        ('stdout' not in log_file) and
        (not os.path.basename(log_file).startswith('roslaunch'))]
    log_files = sorted(log_files, key=lambda f: os.path.basename(f))
    logger.debug('get log files: {}'.format('\n'.join(log_files)))
    logs = []

    # log format [%(name)s][%(levelname)s] %(asctime)s: %(message)s
    pattern = r'\[(?P<name>\S+)\]\[(?P<levelname>\S+)\] (?P<asctime>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}): (?P<message>.*)'
    p = re.compile(pattern)
    loglevels = {
        'debug': 0,
        'info': 1,
        'warn': 2,
        'warning': 2,
        'error': 3,
        'fatal': 4,
    }

    def escape(s):
        if isinstance(s, list):
            return [escape(i) for i in s]
        s = s.replace("&", "&amp;") # Must be done first!
        s = s.replace("<", "&lt;")
        s = s.replace(">", "&gt;")
        s = s.replace('"', "&quot;")
        return s

    def get_log(log_file, loglevel, cursor, message_length=120):
        """parse log files from the start cursor"""
        _log = []
        loglevel = loglevels[loglevel.lower()]
        with open(log_file) as f:
            for _ in xrange(cursor):
                try:
                    next(f)
                except StopIteration:
                    pass
            logrecord = None
            consume = 0
            for line in f:
                m = p.match(line)
                if m:
                    if logrecord:
                        if loglevels[logrecord['levelname'].lower()] >= loglevel:
                            _log.append(logrecord)
                        cursor += consume
                        consume = 0
                        logrecord = None
                    name, levelname, asctime, message = map(
                        m.group, ['name', 'levelname', 'asctime', 'message'])
                    consume += 1
                    extra = []
                    if len(message) > message_length:
                        extra.append(message[message_length:])
                        message = message[:message_length]
                    logrecord = {
                        'name': name,
                        'levelname': levelname,
                        'asctime': asctime,
                        'message': escape(message),
                        'extra': escape(extra),
                    }
                # Append message that doesn't match the log format to the
                # previous matched log record
                elif logrecord:
                    consume += 1
                    logrecord['extra'].append(line)
            # Append the remaining log record
            if logrecord:
                if loglevels[logrecord['levelname'].lower()] >= loglevel:
                    _log.append(logrecord)
                cursor += consume
                consume = 0
                logrecord = None

        _log.reverse()
        return cursor, _log[:LOG_TO_SHOW]

    def isint(i):
        try:
            int(i)
        except:
            return False
        return True

    def parse_node_name(log_file):
        base = os.path.splitext(os.path.basename(log_file))[0]
        tokens = base.split('-')
        if 'roslaunch' in base:
            return base
        try:
            idx = map(isint, tokens).index(True)
            node = '/'.join(tokens[:idx])
        except:
            node = '/'.join(tokens)
        return node

    for log_file in log_files:
        node = parse_node_name(log_file)
        cursor = int(log_cursors.get(log_file, 0))
        cursor, log = get_log(log_file, loglevel, cursor)
        log_cursors[log_file] = cursor
        if log:
            logs.append({
                'node': node,
                'log': log,
                'log_file': log_file,
                'log_to_show': LOG_TO_SHOW,
                })

    result = {
        'logs': logs,
        'cursors': log_cursors
    }
    return json_encode(result)

