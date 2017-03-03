import logging
import csv
from itertools import izip

logger = logging.getLogger('hr.chatbot.polority')


class Polarity(object):

    def __init__(self):
        # a small dictionary of terms which negate polarity
        self._negates = {"not": 1, "don't": 1, "can't": 1,
                         "won't": 1, "isn't": 1, 'never': 1}
        self._polarity = {}

    def load_sentiment_csv(self, sent_csv_file):
        logger.info('Loading sentiment')
        with open(sent_csv_file) as f:
            reader = csv.reader(f)
            sent3 = list(reader)

        # delete header keep phrases over threshold
        del sent3[0]

        # keep phrases over threshold, now is 0.1
        for phrase in sent3:
            # 1 is phrase string with spaces, 6 is polarity
            if abs(float(phrase[6])) > 0.1:
                self._polarity[phrase[1]] = float(phrase[6])
            else:
                logger.debug("{} is ignored".format(phrase[1]))
        logger.info("Loaded {} items".format(len(self._polarity)))

    def get_polarity(self, text):
        not_found = 0
        average = 0.0
        extreme = 0.0
        polarity_list = []
        negate = 1

        # strip punctuation prior to word search
        text = text.rstrip('.?!')
        words = text.split()
        for word in words:
            # check if word in negates
            if self._negates.has_key(word):
                negate = -1
            # check if words in sentic
            if word in self._polarity:
                polarity_list.append(self._polarity[word])
                logger.info(word + ' ' + str(self._polarity[word]))
            else:
                not_found += 1

        # check adjacent pairs as well
        pairs = [' '.join(pair) for pair in izip(words[:-1], words[1:])]
        for pair in pairs:
            if pair in self._polarity:
                polarity_list.append(self._polarity[pair])
                logger.info(pair + ' ' + str(self._polarity[pair]))
            else:
                not_found += 1

        # return average and extrema of words in response
        # This is a very simple function that should be improved
        # probably weighting first half of responses better
        if len(polarity_list) > 0:
            average = sum(polarity_list) / float(len(polarity_list))
            if abs(max(polarity_list)) > abs(min(polarity_list)):
                extreme = max(polarity_list)
            else:
                extreme = min(polarity_list)
            return negate * (average + extreme) / 2.0
        else:
            return 0.0

if __name__ == '__main__':
    p = Polarity()
    try:
        p.load_sentiment_csv('../../scripts/aiml/senticnet3.props.csv')
    except Exception as ex:
        logger.error("Load sentiment file error {}".format(ex))
        print ex
    print p.get_polarity("The dish is yucky")
