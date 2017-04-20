import os
import sys
CWD = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(CWD, '..'))
import aiml
import urllib
import csv
import logging
import glob
from csvUtils import generateAimlFromLongCSV, generateAimlFromSimpleCSV

import xml.etree.ElementTree as ET

logger = logging.getLogger('hr.chatbot.gsheet_chatter')

xmlns = '{http://www.w3.org/2005/Atom}'


def getWorkSheets(skey, dirname='.'):
    urlwks = "https://spreadsheets.google.com/feeds/worksheets/<KEY>/public/full"
    urlwks = urlwks.replace("<KEY>", skey)
    wksData = urllib.urlopen(urlwks).read()
    tree = ET.fromstring(wksData)
    author = tree.find(xmlns + 'author')
    name = author.find(xmlns + 'name').text
    email = author.find(xmlns + 'email').text
    aiml_files, csv_files = [], []

    if not os.path.isdir(dirname):
        os.makedirs(dirname)

    for entry in tree.findall('{}entry'.format(xmlns)):
        aimlFileData = None
        csvData = None
        title = None
        for item in entry.iter():
            if item.tag == xmlns + 'link' and item.attrib.get('type') == 'text/csv':
                pagelink = item.attrib.get('href')
                #pagelink = pagelink.replace('format=csv', 'format=tsv', )
                csvData = loadSheetViaURL(pagelink)
                aimlFileData = generateAimlFromCSV(csvData)
            if item.tag == xmlns + 'title':
                title = item.text
                filename = os.path.join(
                    dirname, '{}_{}.aiml'.format(skey, title))
                csv_fname = os.path.join(
                    dirname, '{}_{}.csv'.format(skey, title))

        if title == 'question':  # skip "question" sheet
            continue

        if csvData is not None:
            with open(csv_fname, 'w') as f:
                f.write(csvData)
            csv_files.append(csv_fname)

        if aimlFileData is not None:
            with open(filename, 'w') as f:
                f.write(aimlFileData)
            aiml_files.append(filename)

    return aiml_files, csv_files

# http://stackoverflow.com/questions/11290337/how-to-convert-google-spreadsheets-worksheet-string-id-to-integer-index-gid


def to_gid(worksheet_id):
    return int(worksheet_id, 36) ^ 31578


def loadSheet(skey, page):
    #// REPLACE THIS WITH YOUR URL
    logger.debug("PAGE:" + str(page))
    logger.debug("GID :" + str(to_gid(str(page))))
    urlcsv = "https://docs.google.com/spreadsheets/d/<KEY>/export?format=csv&id=<KEY>&gid=" + \
        str(page)  # +str(to_gid(str(page)))
    urlcsv = urlcsv.replace("<KEY>", skey)
    csvData = urllib.urlopen(urlcsv).read()
    if ("DOCTYPE html" in csvData):
        return ""
    logger.debug("URL : " + urlcsv)
    return csvData


def loadSheetViaURL(urlcsv):
    csvData = urllib.urlopen(urlcsv).read()
    if ("DOCTYPE html" in csvData):
        return ""
    logger.debug("URL : " + urlcsv)
    return csvData


def generateAimlFromCSV(csvData, delimiter=','):
    lines = csvData.splitlines()
    if (len(lines) == 0):
        return ""
    header = lines[0]
    aimlFile = '<?xml version="1.0" encoding="ISO-8859-1"?>\n'
    aimlFile += '<aiml>\n'
    reader = csv.DictReader(lines, delimiter=delimiter)
    for row in reader:
        logger.debug(row)
        slots = {}
        slots['PATTERN'] = "*"
        slots['THAT'] = "*"
        slots['TEMPLATE'] = ""
        slots['TOPIC'] = "*"
        slots['REDUCE_TO'] = ""
        category = " <category>\n  <pattern>XPATTERN</pattern>\n  <that>XTHAT</that>\n  <template>XTEMPLATEXREDUCE</template>\n </category>\n"
        if (('PATTERN' in row) and (row['PATTERN'] != "")):
            slots['PATTERN'] = row['PATTERN'].upper()
        if (('THAT' in row) and (row['THAT'] != "")):
            slots['THAT'] = row['THAT']
        if (('TEMPLATE' in row) and (row['TEMPLATE'] != "")):
            slots['TEMPLATE'] = row['TEMPLATE'].replace("#Comma", ",")
        if (('TOPIC' in row) and (row['TOPIC'] != "")):
            slots['TOPIC'] = row['TOPIC']
        if (('REDUCE_TO' in row) and (row['REDUCE_TO'] != "")):
            slots['REDUCE_TO'] = "<srai>" + row['REDUCE_TO'] + "</srai>"

        category = category.replace("XPATTERN", slots['PATTERN'])
        category = category.replace("XTHAT", slots['THAT'])
        category = category.replace("XTEMPLATE", slots['TEMPLATE'])
        category = category.replace("XTOPIC", slots['TOPIC'])
        category = category.replace("XREDUCE", slots['REDUCE_TO'])
        aimlFile += category
    aimlFile += "</aiml>"
    return aimlFile


def readAndLoadSheets(sheetList, engine):
    for sheetKey in sheetList:
        aiml_files, _ = getWorkSheets(sheetKey)
        for aiml_file in aiml_files:
            engine.learn(aiml_file)

#      for page in range(0,3):
#       csvDat = loadSheet(sheetKey,int(page))
#       aimlFileData = generateAimlFromCSV(csvDat)
#   if (len(aimlFileData)==0): continue
#       filename = sheetKey+"_"+str(page) +".aiml"
#       target = open(filename, 'w')
#       target.truncate()
#       target.write(aimlFileData)
#       target.close()
#       engine.learn(filename)

# The Kernel object is the public interface to
# the AIML interpreter.


def get_csv_version(csv_file):
    # Guessing
    with open(csv_file) as f:
        header = f.readline().strip()
        if sorted(header.split(',')) == sorted(
                ['Human_says', 'Meaning', 'Robot_says']):
            return "3"
        elif sorted(header.split(',')) == sorted(
                ['Type', 'Pattern', 'That', 'Template', 'Source', 'Think', 'Topic']):
            return "2"
        else:
            return "1"


def batch_csv2aiml(csv_dir, aiml_dir, csv_version=None):
    """Convert all the csv files in the csv_dir to aiml files.
    csv_version:
        1:  PATTERN,THAT,TOPIC,TEMPLATE,REDUCE_TO
        2:  Type,Pattern,That,Template,Source,Think
        3:  Human_says,Meaning,Robot_says
    """
    if not os.path.isdir(aiml_dir):
        os.makedirs(aiml_dir)
    aiml_files = []
    csv_files = []
    for csv_file in glob.glob('{}/*.csv'.format(csv_dir)):
        filename = os.path.basename(csv_file)
        filename = os.path.splitext(filename)[0] + '.aiml'
        filename = os.path.join(aiml_dir, filename)
        aimlFileData = None
        with open(csv_file) as f:
            if csv_version is None:
                csv_version = get_csv_version(csv_file)
            if csv_version == '1':
                csvData = f.read()
                aimlFileData = generateAimlFromCSV(csvData, ',')
            elif csv_version == '2':
                csvData = csv.DictReader(f)
                try:
                    aimlFileData = generateAimlFromLongCSV(csvData)
                except Exception as ex:
                    raise Exception('Generate aiml from csv {} error {}'.format(
                        os.path.basename(csv_file), ex))
            elif csv_version == '3':
                csvData = csv.DictReader(f)
                try:
                    aimlFileData = generateAimlFromSimpleCSV(csvData)
                except Exception as ex:
                    raise Exception('Generate aiml from csv {} error {}'.format(
                        os.path.basename(csv_file), ex))
        if aimlFileData is not None:
            with open(filename, 'w') as f:
                f.write(aimlFileData)
                logger.info("Convert {} to {}".format(csv_file, filename))
                aiml_files.append(filename)
                csv_files.append(csv_file)
    return aiml_files, csv_files

if __name__ == '__main__':
    logging.basicConfig()
    logging.getLogger().setLevel(logging.DEBUG)
    k = aiml.Kernel()

    # **************** CHANGE TO GOOGLE SHEET KEY HERE ***********************
    sheetList = {"1Tbro_Kjbby162Rms0GpQswoqhavXOoRe85HVRyEB1NU"}
    readAndLoadSheets(sheetList, k)

    #csvDat = loadSheet(sheetKey)
    # print "CSVDAT"
    # print csvDat
    #aimlFile = generateAimlFromCSV(csvDat)
    # print aimlFile

    # Use the 'learn' method to load the contents
    # of an AIML file into the Kernel.
    # k.learn("std-startup.xml")

    # Use the 'respond' method to compute the response
    # to a user's input string.  respond() returns
    # the interpreter's response, which in this case
    # we ignore.
    # k.respond("load aiml b")

    # Loop forever, reading user input from the command
    # line and printing responses.
    while True:
        userin = raw_input("> ")
        print "raw response:" + k.respond(userin)
