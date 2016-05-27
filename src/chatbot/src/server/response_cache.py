import datetime as dt
from collections import defaultdict
import logging

logger = logging.getLogger('hr.chatbot.server.response_cache')

class ResponseCache(object):
    def __init__(self):
        self.record = []
        self.index = defaultdict(list)
        self.last_question = None
        self.last_answer = None
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

    def _norm(self, s):
        s = s.lower().strip()
        s = s.replace(',', ' ')
        s = s.replace('.', ' ')
        s = ' '.join(s.split()) # remove consecutive spaces
        return s

    def check(self, question, answer, lang):
        if lang == 'zh':
            return True
        if self._norm(answer) == self.last_answer:
            return False
        if not self.is_unique(answer):
            return False
        if self.contain(question, answer):
            return False
        return True

    def add(self, question, answer, datetime=None):
        question = self._norm(question)
        answer = self._norm(answer)
        time = datetime or dt.datetime.now()
        self.record.append({
            'Datetime': time,
            'Question': question,
            'Answer': answer
        })
        self.index[question].append(len(self.record)-1)
        self.last_question = question
        self.last_answer = answer
        self.last_time = time

    def contain(self, question, answer):
        question=self._norm(question)
        answer=self._norm(answer)
        records = self._get_records(question)
        answers = [r['Answer'] for r in records]
        return answer in answers

    def is_unique(self, answer):
        answer = self._norm(answer)
        answers = [r['Answer'] for r in self.record]
        return not answer in answers

    def _get_records(self, question):
        question=self._norm(question)
        records = [self.record[i] for i in self.index[question]]
        return records

    def _weight_records(self, question):
        records = self._get_records(question)
        answers = [r['Answer'] for r in records]
        now = dt.datetime.now()
        weights = [sum([(now-r['Datetime']).seconds>600, len(r['Answer'])<20, answers.count(r['Answer'])<2]) for r in records]
        return weights

    def get_best_response(self, question):
        weights = self._weight_records(question)
        if weights:
            idx = weights.index(max(weights))
            return self.record[idx]['Answer']

    def dump(self, fname):
        import csv
        all_records = self.staged_record + self.record
        records_to_dump = all_records[self.cursor:]
        if not records_to_dump:
            logger.debug("Nothing to dump")
            return
        with open(fname, 'a') as f:
            writer = csv.DictWriter(
                f, ['Datetime', 'Question', 'Answer'], extrasaction='ignore')
            writer.writeheader()
            writer.writerows(records_to_dump)
            logger.info("Dumpped chat history to {}".format(fname))
            self.cursor = len(all_records)

if __name__ == '__main__':
    cache = ResponseCache()
    cache.add('a', 'hi', dt.datetime(2016, 4, 22, 12, 0, 0))
    cache.add('a', 'hi there', dt.datetime(2016, 4, 22, 12, 30, 0))
    cache.add('a', 'how are you', dt.datetime(2016, 4, 22, 12, 30, 0))
    cache.add('a', 'hi there', dt.datetime(2016, 4, 22, 12, 30, 0))
    cache.add('a', 'how are you', dt.datetime(2016, 4, 22, 12, 32, 0))
    cache.add('a', 'how are you', dt.datetime(2016, 4, 22, 12, 32, 0))
    print cache.is_unique('Hi')
    print cache.is_unique('hello')
    print cache.get_best_response('a')
    cache.dump('tmp')
    print cache.get_best_response('a')
    cache.dump('tmp')
