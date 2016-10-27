import datetime as dt
from collections import defaultdict
import logging
import os
from chatbot.utils import norm

logger = logging.getLogger('hr.chatbot.server.response_cache')

class ResponseCache(object):

    def __init__(self):
        self.record = []
        self.index = defaultdict(list)
        self.last_question = None
        self.last_answer = None
        self.that_question = None
        self.last_time = None
        self.staged_record = []
        self.cursor = 0

    def clean(self):
        self.staged_record.extend(self.record)
        del self.record[:]
        del self.index
        self.record = []
        self.index = defaultdict(list)
        self.last_question = None
        self.last_answer = None
        self.last_time = None

    def check(self, question, answer):
        # each additional character over the 10 characters, adds 30 seconds
        # delay before that AIML string is allowed to repeat.
        same_answers = [r for r in self.record if norm(r['Answer']) == norm(answer)]
        time_elapsed = (dt.datetime.now() - same_answers[-1]['Datetime']
                        ).seconds if same_answers else 0
        if max(0, len(norm(answer)) - 10) * 30 <= time_elapsed:
            logger.info("Allow short repeat answer")
            logger.debug("Answer length {}, time elapsed {}".format(
                len(norm(answer)), time_elapsed))
            return True

        if norm(answer) == norm(self.last_answer):
            logger.info("Last answer repeat")
            return False
        if not self.is_unique(answer):
            logger.info("Non unique answer")
            return False
        if self.contain(question, answer):
            logger.info("Repeat answer")
            return False
        return True

    def add(self, question, answer, datetime=None, **kwargs):
        time = datetime or dt.datetime.now()
        record = {
            'Datetime': time,
            'Question': question,
            'Answer': answer
        }
        if kwargs:
            record.update(kwargs)
        self.record.append(record)
        self.index[norm(question)].append(len(self.record) - 1)
        self.last_question = question
        self.last_answer = answer
        self.last_time = time

    def rate(self, rate, idx):
        all_records = self.staged_record + self.record
        if idx < 0:
            idx = len(all_records) + idx
        if idx >= self.cursor:
            all_records[idx]['Rate'] = rate
            return True
        return False

    def contain(self, question, answer):
        question = norm(question)
        answer = norm(answer)
        records = self._get_records(question)
        answers = [norm(r['Answer']) for r in records]
        return answer in answers

    def is_unique(self, answer):
        answers = [norm(r['Answer']) for r in self.record]
        return not norm(answer) in answers

    def _get_records(self, question):
        records = [self.record[i] for i in self.index[norm(question)]]
        return records

    def dump(self, fname):
        import csv
        all_records = self.staged_record + self.record
        records_to_dump = all_records[self.cursor:]
        if not records_to_dump:
            logger.debug("Nothing to dump")
            return False
        header = ['Datetime', 'Question', 'Answer', 'Rate']
        for k in records_to_dump[0].keys():
            if k not in header:
                header.append(k)

        dirname = os.path.dirname(fname)
        if not os.path.isdir(dirname):
            os.makedirs(dirname)
        with open(fname, 'a') as f:
            writer = csv.DictWriter(f, header, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(records_to_dump)
            logger.info("Dumpped chat history to {}".format(fname))
            self.cursor = len(all_records)
            return True
        return False

if __name__ == '__main__':
    cache = ResponseCache()
    cache.add('a', 'hi', dt.datetime(2016, 4, 22, 12, 0, 0),
              Answeredby='bot', User='user')
    cache.add('a', 'Hi there', dt.datetime(2016, 4, 22, 12, 30, 0))
    cache.add('a', 'how are you', dt.datetime(2016, 4, 22, 12, 30, 0))
    cache.add('a', 'hi there', dt.datetime(2016, 4, 22, 12, 30, 0))
    cache.add('a', 'how are you', dt.datetime(2016, 4, 22, 12, 32, 0))
    cache.add('a', 'how are you', dt.datetime(2016, 4, 22, 12, 32, 0))
    print cache.is_unique('Hi')
    print cache.is_unique('hello')
    cache.dump('./tmp')
    cache.dump('./tmp')
