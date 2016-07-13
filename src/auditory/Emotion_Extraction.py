__author__ = 'zelalem'

from ems import em
from textblob import TextBlob

name = input("What's your name? ")
print("Nice to meet you " + name + "!")
age = input("Your age? ")
print("So, you are are already " + str(age) + " years old, " + name + "!")




f = open('/home/zelalem/sample_emotion.txt')

def analyze_sentiment(sent):
    if(sent.sentiment.polarity > 0):
        val='positive'
    elif(sent.sentiment.polarity<0):
       val= 'negative'
    else:
       val='neutral'

    return val

for line in iter(f):
    line = line.strip('\n').lower()
    line = line.strip('.').lower()
    sent_splited = line.split(' ')
    # print sent_splited
    for x in em:
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