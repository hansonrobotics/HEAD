import logging
import json
import requests
import random
from bs4 import BeautifulSoup, NavigableString, Tag
from config import SOLR_URL

logger = logging.getLogger('hr.chatbot.server.solr_match')


def solr3col(text):
    # No match, try improving with SOLR

    logger.info('SOLR start')
    params = {
        "fl": "title,body,score",
        "indent": "true",
        "wt": "json",
        "rows": "20"
    }

    params['q'] = 'title:{}'.format(text)
    lucText = requests.get(
        SOLR_URL + '/solr/3colpattern/select', params=params).text

    if len(lucText) > 0:
        logger.debug('RESPONSE: ' + lucText)
        jResp = json.loads(lucText)
        if jResp['response']['numFound'] > 0:
            for resp in jResp['response']['docs']:
                logger.debug(' SOLR pattern: {} {}'.format(
                    resp['body'], resp['title']))

            doc = jResp['response']['docs'][0]
            query = doc['body'][0]

            params['q'] = 'body:{}'.format(query)
            templText = requests.get(
                SOLR_URL + '/solr/3coltemplate/select', params=params).text

            if len(templText) > 0:
                templResp = json.loads(templText)
                if templResp['response']['numFound'] > 0:
                    templ = templResp['response']['docs'][0]
                    meaning = templ['body']
                    candidates = [resp for resp in templResp[
                        'response']['docs'] if resp['body'] == meaning]
                    assert len(meaning) > 0
                    assert len(candidates) > 0
                    candidate = random.sample(candidates, 1)[0]
                    return meaning[0], parse_aiml_text(candidate['title'][0]), candidate['score']


def solr_aiml(text):
    # No match, try improving with SOLR
    params = {
        "fl": "*,score",
        "indent": "true",
        "q": text,
        "qf": "title",
        "wt": "json",
        "rows": "20"
    }
    lucText = requests.get(SOLR_URL + '/solr/aiml/select', params=params).text

    if len(lucText) > 0:
        logger.debug('RESPONSE: ' + lucText)
        jResp = json.loads(lucText)
        if jResp['response']['numFound'] > 0:
            lucResult = jResp['response']['docs'][0]
            return lucResult


def parse_aiml_text(text):
    text = '<p>' + text + '</p>'
    soup = BeautifulSoup(text, 'lxml')
    tokens = []
    try:
        for c in soup.p.children:
            if isinstance(c, NavigableString):
                token = c.string.strip()
                if token:
                    tokens.append(token)
    except Exception as ex:
        logger.warn(ex)
        return text
    return ' '.join(tokens)

if __name__ == '__main__':
    print solr3col('do you have animal friends?')
    print solr3col('hi')
    print solr3col('what is your favior color')
    print solr3col('tell me a story')
    print solr3col('would you like to star in movies')
    print parse_aiml_text('hi')
    print parse_aiml_text('<think>hi</think>how are you')
    print solr_aiml('what are your name')  # correct grammar
