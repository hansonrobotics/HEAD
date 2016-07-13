#
# Google image-by-image search scraper
#
import sys
import os
# command line parsing
import argparse
# random selection of user agents
import random
# XML/HTML parsing
import xml.etree
from xml.etree import ElementTree
import libxml2
from HTMLParser import HTMLParser
# naturual language tools for sanitizing results
import nltk
# regular expressions for sanitizing results
import re
# check for ssl availability in general
try:
    import ssl
except ImportError:
    print "error: no ssl support"
# library for performing HTTP(S) requests
import urllib2
from imgurpython import ImgurClient
import json

""" Parser model to extract the content only and not the tags """
class MyHTMLParser(HTMLParser):
    datafields = []

    def handle_data(self, data):
        self.datafields.append(data)

    def get_data(self):
        return ' '.join(self.datafields)

    def clean(self):
        self.datafields = []

""" HTTP request with libcurl """
class image_scaper:
    def __init__(self, useragent):
        with open(useragent, 'r') as uafile:
            self.user_agents = uafile.readlines()
        client_id = '0dae3160fb6cd6a'
        client_secret = '1a5b722c05f657dc1c97b82dc7d4e812360209a9'
        self.client = ImgurClient(client_id, client_secret)

        self.xpath = {}

        # xpaths for different fields on the result page of Google image-by-image search
        # if the interface is changed, this is the part that needs modification
        # there are web browser plugins that provide you with proper xpaths
        self.xpath['bestguess'] = "/html/body[@id='gsr']/div[@id='main']/div[@id='cnt']/div[@class='mw']/div[@id='rcnt']/div[@class='col'][1]/div[@id='center_col']/div[@id='res']/div[@id='topstuff']/div[@class='card-section']/div[@class='_hUb']/a[@class='_gUb']"

        self.xpath['desc'] = "/html/body[@id='gsr']/div[@id='main']/div[@id='cnt']/div[@class='mw']/div[@id='rcnt']/div[@id='rhscol']/div[@id='rhs']/div[@id='rhs_block']/ol/li[@class='g mnr-c rhsvw kno-kp g-blk']/div[@class='kp-blk _Jw _Rqb _RJe']/div[@class='xpdopen']/div[@class='_OKe']/ol/li[@class='_DJe mod'][1]/div[@class='_cgc kno-fb-ctx']/div[@class='kno-rdesc']/span[1]"

        # this xpath was built by removing explicit element access (just get several xpaths and try to see a pattern)
        self.xpath['summaries'] = "/html/body[@id='gsr']/div[@id='main']/div[@id='cnt']/div[@class='mw']/div[@id='rcnt']/div[@class='col'][1]/div[@id='center_col']/div[@id='res']/div[@id='search']/div/div[@id='ires']/ol[@id='rso']/div[@class='srg'][1]/li[@class='g']/div[@class='rc']/div[@class='s']/div/span[@class='st']"

        self.xpath['summaries_alternative'] = "/html/body[@id='gsr']/div[@id='main']/div[@id='cnt']/div[@id='rcnt']/div[@class='col'][2]/div[@id='center_col']/div[@id='res']/div[@id='search']/div[@id='ires']/ol[@id='rso']/li[@class='g'][1]/div[@class='rc']/div[@class='s']/div/span[@class='st']"

        self.xpath['titles'] = "/html/body[@id='gsr']/div[@id='main']/div[@id='cnt']/div[@class='mw']/div[@id='rcnt']/div[@class='col'][1]/div[@id='center_col']/div[@id='res']/div[@id='search']/div/div[@id='ires']/ol[@id='rso']/div[@class='srg']/li[@class='g']/div[@class='rc']/h3[@class='r']/a"

        self.xpath['titles_alternative'] = "/html/body[@id='gsr']/div[@id='main']/div[@id='cnt']/div[@id='rcnt']/div[@class='col'][2]/div[@id='center_col']/div[@id='res']/div[@id='search']/div[@id='ires']/ol[@id='rso']/li[@class='g']/div[@class='rc']/h3[@class='r']/a"


    def get_raw_html_libcurl(self,request_url, user_agent):
        import pycurl
        import StringIO
        curl = pycurl.Curl()
        curl.setopt(pycurl.URL, request_url)
        curl.setopt(pycurl.HEADER, 0)
        curl.setopt(pycurl.REFERER, 'http://localhost')
        curl.setopt(pycurl.SSL_VERIFYPEER, False)
        curl.setopt(pycurl.user_agent, user_agent)
        curl.setopt(pycurl.FOLLOWLOCATION, True)

        result_stringio = StringIO.StringIO()
        curl.setopt(pycurl.WRITEFUNCTION, result_stringio.write)

        try:
            curl.perform()
        except:
            import traceback
            traceback.print_exc(file=sys.stderr)
            sys.stderr.flush()

        return result_stringio.getvalue()

    """ HTTP request with urllib (default) """
    def get_raw_html_urllib(self,request_url, user_agent):
        #req = urllib2.urlopen( request_url )
        opener = urllib2.build_opener()
        opener.addheaders = [('User-agent', user_agent)]
        response = opener.open(request_url)
        return response.read()

    """ for manual sanitizing """
    def remove_containing_word(s, words):
        for w in words:
            s = re.sub('[^ ]*%s[^ ]*' % w, '', s)
        return s

    """ sanitizing results with natural language tools """
    def sanitize_result(self,s):
        # manual sanitizing
        #s = remove_containing_word(s, ['http://', 'www\.']    )
        #s = remove_containing_word(s, ['\.com', '\.org', '\.net'])
        #s = remove_containing_word(s, ['\.jpg', '\.jpeg', '\.png'] )
        #s = re.sub("[^a-zA-Z0-9 -]+", "", s)

        # remove ), (, and %
        s = re.sub('[\(\)\%]+', '', s)
        if len(s)==0:
            return s

        # split into tokens
        tokens = nltk.word_tokenize(s)
        # tag the tokens
        tagged_tokens = nltk.pos_tag( tokens )
        # we only want to extract proper nouns, etc.
        grammar = "NP: {(<NN>|<NNP>|<NNS>)+}"
        # parse the tagged tokens
        cp = nltk.RegexpParser(grammar)
        parsed_sentence = cp.parse(tagged_tokens)
        # go through the parsing result and put everything into a string
        terms = []
        for e in parsed_sentence:
            if not isinstance(e,tuple):
                for term in e:
                    if len(term[0])>1:
                        terms.append( term[0] )
        # join results and make everything lowercase
        s = ' '.join(terms)
        s = s.lower()

        return s

    """ obtain all xpath results in a string """
    def get_simple_xpath( self,doc, xpath ):
        ctxt = doc.xpathNewContext()
        # get xpath result
        xp_results  = ctxt.xpathEval(xpath)
        results = []
        i = 0
        # simply remove all tags
        parser = MyHTMLParser()
        parser.clean()
        for xp in xp_results:
            s = str(xp)
            if len(s)>0:
                parser.feed( s )

        ctxt.xpathFreeContext()
        # sanitize the results
        return self.sanitize_result(parser.get_data())

    #######################################

    def image_scraper(self, image_path):
        print "To Upload Image"
        result = self.client.upload_from_path(image_path)
        print(result['link'])
        print "Uploaded Image"
        request_url = "https://www.google.com/searchbyimage?&image_url=" + result['link']

        # select user agent
        user_agent = random.choice(self.user_agents).rstrip()


        gis_raw_result = self.get_raw_html_urllib( request_url, user_agent )

        parse_options = libxml2.HTML_PARSE_RECOVER + libxml2.HTML_PARSE_NOERROR + libxml2.HTML_PARSE_NOWARNING

        doc = libxml2.htmlReadDoc( gis_raw_result, '', None, parse_options)

        scrapeResult = {}
        for key in self.xpath:
            r = self.get_simple_xpath(doc, self.xpath[key])
            scrapeResult[key] = r

        scrapeResults = scrapeResult

        doc.freeDoc()


        # output of the results

        print json.dumps( scrapeResults, indent=4, sort_keys=False)

if __name__ == '__main__':
    image_scraper = image_scaper('/home/icog-labs/hansonrobotics/HEAD/src/vision/cmt_tracker/resources/useragents.txt')
    image_scraper.image_scraper('/home/icog-labs/Pictures/ben7.png')

