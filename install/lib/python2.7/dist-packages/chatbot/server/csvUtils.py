__author__ = 'daviddemaris'
import os
import sys
CWD = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(CWD, '..'))
import csv

escapes = ''.join([chr(char) for char in range(16, 17)])


def generateAimlFromSimpleCSV(csvData):

    aimlFile = '<?xml version="1.0" encoding="ISO-8859-1"?>\n'
    aimlFile += '<aiml>\n'

    state = 'init'
    temp_random = []
    srai = []
    currentMeaning = ''
    currentPattern = ''

    def processInPatternState(category, aimlFile):
         # build template from random, think
        if len(temp_random) > 1:
            randomString = ' <random> \n'
            for li in temp_random:
                li = li.strip()
                # spurious star pairs are put in during aiml2csv by bs4, strip
                # back to singleton
                randomString = randomString.replace('<star></star>', '<star/>')
                randomString = randomString + '  <li> ' + li + ' </li>\n'
            randomString = randomString + ' </random>\n'
        else:
            # single random element found so no random, li tags
            randomString = str(temp_random[0]).strip()

            # spurious star pairs are put in during aiml2csv by bs4, strip back
            # to singleton
            randomString = randomString.replace('<star></star>', '<star/>')
        # # spurious star pairs are put in during aiml2csv by bs4, strip back to singleton
        slots['Think'] = slots['Think'].replace('<star></star>', '<star/>')
        slots['Template'] = randomString + slots['Think']
        category = category.replace("XPATTERN", slots['Pattern'])
        # in case that has no *
        if slots['That'] != '*':
            category = category.replace("XTHAT", slots['That'])
        else:
            category = category.replace('<that>', '')
            category = category.replace('</that>\n', '')
            # slots['That'].replace('*','')
            category = category.replace("XTHAT", '')
        category = category.replace("XTEMPLATE", slots['Template'])
        category = category.replace("XTOPIC", slots['Topic'])

        aimlFile += category
        # write out any accumulated srai
        for red in srai:
            category = " <category>\n  <pattern>XPATTERN</pattern>\n <template>XTEMPLATE</template>\n </category>\n"
            # need to get rid of random and li tags from template

            #category = category.replace("XPATTERN",'<srai>'+red+'</srai>')
            category = category.replace("XPATTERN", red)

            category = category.replace(
                "XTEMPLATE", '<srai>' + slots['Pattern'] + '</srai>')
            # print 'reduce', red
            # print 'category', category
            aimlFile += category
        return aimlFile

    # in pattern
    for row in csvData:
        if row['Meaning'] != currentMeaning:
            if ('Meaning' in row) and row['Meaning'] != "":
                currentMeaning = row['Meaning']
                # put first template
                # temp_random.append(row['Meaning'])
            category = " <category>\n  <pattern>XPATTERN</pattern>\n  <that>XTHAT</that>\n  <template>XTEMPLATE</template>\n </category>\n"
            if state == 'init':
                state = 'inPattern'

            elif state == 'inPattern':
                # hit next meaning so create category of single  and srais
                # pointing to it
                aimlFile = processInPatternState(category, aimlFile)
                temp_random = []
                srai = []
                # put first template
                # temp_random.append(row['Meaning'])
        # regardless of state, having written out last pattern or not,
        # initialize and read current row
        slots = {}
        slots['Pattern'] = "*"
        slots['That'] = "*"
        slots['Template'] = ""
        slots['Topic'] = "*"
        slots['Think'] = ""
        slots['Pattern'] = currentMeaning

        # temp_random=[]
        # srai=[]
        if (row['Human_says'] != ""):
            srai.append(row['Human_says'])
            # keep these around in case we put syntactic sugar in columns to recover this aiml expression
            #if (('That' in row ) and (row['That']!="")): slots['That']=row['That']
            # assume template may be first of random
            #if (('Topic' in row ) and (row['Topic']!="")): slots['Topic']=row['Topic']
            # f (('Think' in row ) and (row['Think']!="")): slots['Think']=row['Think']
        # if ('Meaning' in row) and (row['Meaning'])!="":
        #    slots['Pattern']==row['Meaning']

        if (('Robot_says' in row) and (row['Robot_says'] != "")):
            temp_random.append(row['Robot_says'].replace('#Comma', ","))

    # write out accumulated data for last category
    aimlFile = processInPatternState(category, aimlFile)

    aimlFile += "</aiml>"
    aimlFile = aimlFile.translate(None, escapes)
    return aimlFile


def generateAimlFromLongCSV(csvData):

    aimlFile = '<?xml version="1.0" encoding="ISO-8859-1"?>\n'
    aimlFile += '<aiml>\n'

    state = 'init'
    temp_random = []
    srai = []

    def processInPatternState(category, aimlFile):
         # build template from random, think
        if len(temp_random) > 1:
            randomString = ' <random> \n'
            for li in temp_random:
                li = li.strip()
                # spurious star pairs are put in during aiml2csv by bs4, strip
                # back to singleton
                randomString = randomString.replace('<star></star>', '<star/>')
                randomString = randomString + '  <li> ' + li + ' </li>\n'
            randomString = randomString + ' </random>\n'
        else:
            # single random element found so no random, li tags
            randomString = str(temp_random[0]).strip()

            # spurious star pairs are put in during aiml2csv by bs4, strip back
            # to singleton
            randomString = randomString.replace('<star></star>', '<star/>')
        # # spurious star pairs are put in during aiml2csv by bs4, strip back to singleton
        slots['Think'] = slots['Think'].replace('<star></star>', '<star/>')
        slots['Template'] = randomString + slots['Think']
        category = category.replace("XPATTERN", slots['Pattern'])
        # in case that has no *
        if slots['That'] != '*':
            category = category.replace("XTHAT", slots['That'])
        else:
            category = category.replace('<that>', '')
            category = category.replace('</that>\n', '')
            # slots['That'].replace('*','')
            category = category.replace("XTHAT", '')
        category = category.replace("XTEMPLATE", slots['Template'])
        category = category.replace("XTOPIC", slots['Topic'])

        aimlFile += category
        # write out any accumulated srai
        for red in srai:
            category = " <category>\n  <pattern>XPATTERN</pattern>\n <template>XTEMPLATE</template>\n </category>\n"
            # need to get rid of random and li tags from template

            #category = category.replace("XPATTERN",'<srai>'+red+'</srai>')
            category = category.replace("XPATTERN", red)

            category = category.replace(
                "XTEMPLATE", '<srai>' + slots['Pattern'] + '</srai>')
            # print 'reduce', red
            # print 'category', category
            aimlFile += category
        return aimlFile

    # in pattern
    for row in csvData:

        # three row types pattern, alt (element of random list tag in template), srai
        # pattern triggers start of new category
        # append templates to list, append srais to a list,
        if row['Type'] == 'pattern':
            category = " <category>\n  <pattern>XPATTERN</pattern>\n  <that>XTHAT</that>\n  <template>XTEMPLATE</template>\n </category>\n"
            if state == 'init':
                state = 'inPattern'

            elif state == 'inPattern':
                aimlFile = processInPatternState(category, aimlFile)
            # regardless of state, having written out last pattern or not,
            # initialize and read current row

            slots = {}
            slots['Type'] = "*"
            slots['Pattern'] = "*"
            slots['That'] = "*"
            slots['Template'] = ""
            slots['Topic'] = "*"
            slots['Think'] = ""
            temp_random = []
            srai = []
            if (row['Pattern'] != ""):
                slots['Pattern'] = row['Pattern']
                if (('That' in row) and (row['That'] != "")):
                    slots['That'] = row['That']
                # assume template may be first of random
                if (('Template' in row) and (row['Template'] != "")):
                    temp_random.append(row['Template'].replace("#Comma", ","))
                if (('Topic' in row) and (row['Topic'] != "")):
                    slots['Topic'] = row['Topic']
                if (('Think' in row) and (row['Think'] != "")):
                    slots['Think'] = row['Think']

        if row['Type'] == 'alt' or row['Type'] == "":
            if (('Template' in row) and (row['Template'] != "")):
                temp_random.append(row['Template'])
            # use
        if row['Type'] == 'srai':
            if state == 'inPattern':
                # maybe should store tuple or pattern.template string
                if (('Template' in row) and (row['Template'] != "")):
                    srai.append(row['Pattern'])
    # write out accumulated data for last category
    aimlFile = processInPatternState(category, aimlFile)

    aimlFile += "</aiml>"
    aimlFile = aimlFile.translate(None, escapes)
    return aimlFile

# test
if __name__ == '__main__':
    # logging.basicConfig()
    # logger=logging.getLogger().setLevel(logging.DEBUG)
    # open long csv file
    """
    longtest=csv.DictReader(open('../character_aiml/sophia-personality.invert.csv','r'))
    aiml=generateAimlFromLongCSV(longtest)
    ftest=open('../character_aiml/sophia-personality.invert.xml','w')
    print >>ftest,aiml
    ftest.close()
    """
    simpletest = csv.DictReader(open('../character_aiml/convoid419.csv', 'r'))
    aiml = generateAimlFromSimpleCSV(simpletest)
    ftest = open('../character_aiml/convoid419.invert.xml', 'w')
    print >>ftest, aiml
    ftest.close()
