 #!/usr/bin/python
#
"""
This module translates simple markup into SSSML expressive markup which can be passed to TTS.
esay_expand is the function which does the translation
* key word or modifier  (may want to use on adjective of key word
/ key word with pitch raise, emphasis, slow
() key phrase
Modify the dictionary expressive-subs to adjust the parameters
If optional argument check=True, the proportion of key words is printed but not subsitition is performed

"""
import re

expressive_subs={
        '(': '<prosody rate=\"0.85\" range=\"x-high\"> ',
        ')': '</prosody>',
        '*': ' <prosody rate=\"0.85\"> <emphasis> ',
        '#': ' <prosody pitch=\"high\" rate=\"0.9\"> <emphasis> ',
    }


def key_words(marked,mark_char):
    key_word_end=' </emphasis> </prosody> '
    start=0
    print 'matching ', mark_char
    while (marked.find(mark_char,start))!=-1:
        pos=marked.find(mark_char,start)
        # find end of word marked by space, (, ), comma or semicolon.  Take min of returned
        end=[]

        end.append(marked.find(' ',pos))
        end.append(marked.find('(',pos))
        end.append(marked.find('<',pos))
        end.append(marked.find(')',pos))
        end.append(marked.find(',',pos))
        end.append(marked.find('.',pos))
        end.append(marked.find(';',pos))
        # remove any -1 if we didn't find a possible word termination
        end=[x for x in end if x!=-1]
        end=min(end)
        #print 'key word ', marked[pos:end]
        marked=marked[0:end]+key_word_end+marked[end:]
        marked=marked.replace(mark_char,expressive_subs[mark_char],1)
        # new end after append
        start=end+len(expressive_subs[mark_char])+len(key_word_end)
        #print "next sub from",marked[start:]
    return marked

def esay_expand(marked,check=False):
    phrase_target_low=.35
    phrase_target_high=.45
    word_target_low=.10
    word_target_high=.14
    # check matching parens
    if marked.count('(')!=marked.count(')'):
        return -1
    # copy, strip, count words

    cleaned=marked.translate(None,'()*.,#')

    word_total=len(cleaned.split(' '))
    # compare key words and key phrases to targets, report
    #print word_total," words found"
    print (marked.count('*')+marked.count('#'))/float(word_total), "key words, target ",word_target_low,"-",word_target_high
    # get words in parens with regex

    in_parens=re.findall(r"\((.*?)\)",marked)
    #print 'in_parens ',in_parens
    phr_total=0
    for str in in_parens:
        phr_total+=len(str.split(' '))

    print phr_total / float(word_total), "key phrases, target ",phrase_target_low,"-",phrase_target_high


    if check:
        return


    # substitute key words

    marked=key_words(marked,'#')
    marked=key_words(marked,'*')

    # subsitute key phrases

    marked=marked.replace('(',expressive_subs['('])
    marked=marked.replace(')',expressive_subs[')'])
    # speed up text where not marked
    speed='<prosody rate=\"x-fast\"> '
    end_speed=' </prosody>'
    marked=speed+marked+end_speed
    return marked

#tstr='Now (*witches and *wizards), as you perhaps know, are *people who are (born for the first time).'
tstr='Now (*witches and *wizards), as you perhaps know, are *people who are #born for the (first time). \
I suppose we have *all passed through this fair experience, we must *all have had our (chance of making magic). '

print esay_expand(tstr)
