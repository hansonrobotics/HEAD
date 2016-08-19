__author__ = 'zelalem'

from src.auditory.ems import em
from textblob import TextBlob


def analyze_sentiment(sent):
    if(sent.sentiment.polarity > 0):
        val='positive'
    elif(sent.sentiment.polarity<0):
       val= 'negative'
    else:
       val='neutral'

    return val

if __name__ == "__main__":
    f = open('C:/Users/rediet/Documents/Vocie-samples/readTTS.txt')

    for line in iter(f):
        line = line.strip('\n').lower()
        line = line.strip('.').lower()
        sent_splited = line.split(' ')
        # print sent_splited
        for x in em:
            # print str(sent_splited) + " " + str(sent_splited.__contains__(x)) + " " + str(x)
            if sent_splited.__contains__(x):
                sent_sentiment =analyze_sentiment(TextBlob(line))
                print line+'.'+' '+'<'+em[x]+'>'+' '+'<'+sent_sentiment+'>'
                # print line+'.'+' '+'<'+em[x]+'>'
#

# import numpy as np
# x = np.array([[1, 2, 3],
#               [1, 2, 3]])
# y = np.array([[1, 1, 1],
#               [1, 2, 3]])
# z = x - y
# print z