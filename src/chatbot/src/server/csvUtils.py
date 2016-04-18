__author__ = 'daviddemaris'
import os
import sys
CWD = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(CWD, '..'))
sys.path.insert(0, os.path.join(CWD, '../scripts'))
import csv
from bs4 import BeautifulSoup
import logging

def generateAimlFromLongCSV(csvData):
   #lines = csvData.splitlines()
   #if (len(lines) == 0) : return "";
   #header = lines[0];
   aimlFile='<?xml version="1.0" encoding="ISO-8859-1"?>\n'
   aimlFile+='<aiml>\n'
   #reader = csv.DictReader(lines, delimiter='\t')
   state='init'
   for row in csvData:
      #print row
      #logger.debug(row)
      randomString=''
      # three row types pattern, alt (element of random list tag in template), srai
      # pattern triggers start of new category
      # append templates to list, append srais to a list,
      if row['Type']=='pattern':
            if state=='init':
                state='inPattern'

            elif state=='inPattern':
                # build template from random, think
                if len(temp_random)>0:
                      randomString=' <random> \n'
                      for li in temp_random:
                          randomString=randomString+'  <li> '+li+' </li>\n'
                      randomString=randomString+' </random>\n'
                else:
                      # single random element found so no random, li tags
                      randomString=str(temp_random[0])

                slots['Template']=randomString+slots['Think']
                category=category.replace("XPATTERN",slots['Pattern'])
                # in case that has no *
                if slots['That']!='*':
                    category=category.replace("XTHAT",slots['That'])
                else:
                    category=category.replace('<that>','')
                    category=category.replace('</that>\n','')
                    #slots['That'].replace('*','')
                    category=category.replace("XTHAT",'')
                category=category.replace("XTEMPLATE",slots['Template'])
                category=category.replace("XTOPIC",slots['Topic'])

                aimlFile += category
                  # write out any accumulated srai
                for red in srai:
                    category = " <category>\n  <pattern>XPATTERN</pattern>\n <template>XTEMPLATE</template>\n </category>\n"
                    # need to get rid of random and li tags from template

                    #category = category.replace("XPATTERN",'<srai>'+red+'</srai>')
                    category = category.replace("XPATTERN",red)

                    category = category.replace("XTEMPLATE",'<srai>'+slots['Pattern']+'</srai>')
                    #print 'reduce', red
                    #print 'category', category
                    aimlFile += category
            # regardless of state, having written out last pattern or not, initialize and read current row
            category = " <category>\n  <pattern>XPATTERN</pattern>\n  <that>XTHAT</that>\n  <template>XTEMPLATE</template>\n </category>\n"
            slots = {}
            slots['Type']="*"
            slots['Pattern']="*"
            slots['That']="*"
            slots['Template']=""
            slots['Topic']="*"
            slots['Think']=""
            temp_random=[]
            srai=[]
            if (row['Pattern']!=""):
                      slots['Pattern']=row['Pattern']
                      if (('That' in row ) and (row['That']!="")): slots['That']=row['That']
                      # assume template may be first of random
                      if (('Template' in row ) and (row['Template']!="")): temp_random.append(row['Template'].replace("#Comma",","))
                      if (('Topic' in row ) and (row['Topic']!="")): slots['Topic']=row['Topic']
                      if (('Think' in row ) and (row['Think']!="")): slots['Think']=row['Think']
            else:
                print 'pattern expected but blank'
                #logger.error("Pattern expected but blank",row)
            # in pattern
      if row['Type']=='alt' or row['Type']=="":
                if (('Template' in row) and (row['Template']!="")): temp_random.append(row['Template'])
                # use
      if row['Type']=='srai':
         if state=='inPattern':
               # maybe should store tuple or pattern.template string
               if (('Template' in row) and (row['Template']!="")):
                   srai.append(row['Pattern'])


   aimlFile+="</aiml>"
   return aimlFile

# test
if __name__ == '__main__':
    #logging.basicConfig()
    #logger=logging.getLogger().setLevel(logging.DEBUG)
    # open long csv file
    longtest=csv.DictReader(open('../character_aiml/clkSheet.csv','r'))
    aiml=generateAimlFromLongCSV(longtest)
    ftest=open('../character_aiml/sophia.clkSheet.invert.xml','w')
    #simpltest=csv.DictReader(open('../character_aiml/sophia.stories.csv','r'))
    print >>ftest,aiml
