# -*- coding: utf-8 -*-
import re

def shorten(text, cutoff):
    """
    cutoff: The number of words that we want to keep, the exceeded words
            will be replaced with 'blahblah'. This is not a strict limit,
            because we want to cut at nearest the punctuation next to
            the N-th (N is cutoff) word.
    """
    text_unicode = text.decode('utf-8')
    if len(text_unicode) > cutoff:
        tail = text_unicode[cutoff:]
        match = re.search(u'\uff0c|\u3002', tail) # find，or。
        if match:
            if tail[match.start()] == u'\uff0c':
                text_unicode = text_unicode[:cutoff]+tail[:match.end()]+u"巴拉巴拉"
            else:
                text_unicode = text_unicode[:cutoff]+tail[:match.end()]
        else:
            text_unicode = text_unicode[:cutoff]+u"巴拉巴拉"
    text = text_unicode.encode('utf-8')
    return text


