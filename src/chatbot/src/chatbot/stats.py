import os
import logging
import pandas as pd
import glob
import re
import datetime as dt
from collections import Counter

logger = logging.getLogger('hr.chatbot.stats')
trace_pattern = re.compile(
    r'../(?P<fname>.*), (?P<tloc>\(.*\)), (?P<pname>.*), (?P<ploc>\(.*\))')


def collect_history_data(history_dir, days):
    today = dt.datetime.now()
    dfs = []
    for d in glob.glob('{}/*'.format(history_dir)):
        if os.path.isdir(d):
            dirname = os.path.basename(d)
            dirdate = None
            try:
                dirdate = dt.datetime.strptime(dirname, '%Y%m%d')
            except Exception as ex:
                logger.error(ex)
            if dirdate and (days == -1 or (today - dirdate).days < days):
                for fname in glob.glob('{}/{}/*.csv'.format(history_dir, dirname)):
                    try:
                        dfs.append(pd.read_csv(fname))
                    except Exception as ex:
                        logger.warn("Reading {} error: {}".format(fname, ex))
    if not dfs:
        return None
    df = pd.concat(dfs, ignore_index=True)
    df = df[df.Datetime != 'Datetime'].sort(
        ['User', 'Datetime']).drop_duplicates()
    return df


def history_stats(history_dir, days):
    df = collect_history_data(history_dir, days)
    if df is None:
        return {}
    if days == -1:
        stats_csv = '{}/full_history.csv'.format(history_dir)
    else:
        stats_csv = '{}/last_{}_days.csv'.format(history_dir, days)
    columns = [u'Datetime', u'Revision', u'User', u'BotName',
               u'AnsweredBy', u'Question', u'Answer', u'Rate', u'Trace']
    df.to_csv(stats_csv, index=False, columns=columns)
    logger.info("Write statistic records to {}".format(stats_csv))
    records = len(df)
    rates = len(df[df.Rate.notnull()])
    good_rates = len(df[df.Rate.isin(['good'])])
    bad_rates = len(df[df.Rate.isin(['bad'])])
    if records > 0:
        csd = float(records - bad_rates) / records
    response = {
        'customers_satisfaction_degree': csd,
        'number_of_records': records,
        'number_of_rates': rates,
        'number_of_good_rates': good_rates,
        'number_of_bad_rates': bad_rates,
    }
    return response


def playback_history(df):
    from client import Client
    client = Client(os.environ.get('HR_CHATBOT_AUTHKEY', 'AAAAB3NzaC'), test=True)
    pattern_column = []
    for question in df.Question:
        answer = client.ask(question, True)
        traces = answer.get('trace')
        patterns = []
        if traces:
            for trace in traces:
                match_obj = trace_pattern.match(trace)
                if match_obj:
                    patterns.append(match_obj.group('pname'))
        pattern_column.append(patterns)
    df.loc[:,'Pattern'] = pd.Series(pattern_column, index=df.index)
    return df

def pattern_stats(history_dir, days):
    df = collect_history_data(history_dir, days)
    if df is None:
        return {}
    df = playback_history(df)
    patterns = sum(df.Pattern, [])
    counter = Counter(patterns)
    pattern_freq = pd.Series(counter)
    pattern_freq.sort(ascending=False)
    stats_csv = '{}/pattern_frequency.csv'.format(history_dir)
    pattern_freq.to_csv(stats_csv)
    logger.info("Write pattern statistic to {}".format(stats_csv))

if __name__ == '__main__':
    logging.basicConfig()
    logging.getLogger().setLevel(logging.INFO)
    history_stats(os.path.expanduser('~/.hr/chatbot/history'), -1)
    history_stats(os.path.expanduser('~/.hr/chatbot/history'), 7)
    pattern_stats(os.path.expanduser('~/.hr/chatbot/history'), -1)
