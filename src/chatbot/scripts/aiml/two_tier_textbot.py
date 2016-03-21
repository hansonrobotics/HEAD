import aiml
import sys

# get bot name argument and check
validbots=['pkd','sophia']
botname="sophia"
if len(sys.argv)>1:

    if sys.argv[1] in validbots:
        botname=sys.argv[1]
    else:
        botname="sophia"

def load(kern, botname):
# add more non generic stuff here, you could say sophia*.xml
    print "running character",botname
    if botname=='sophia':
        kern.learn("../character_aiml/sophia.*.xml")
    elif botname=='pkd':
        kern.learn("../character_aiml/pkd.*.xml")
        kern.learn("../character_aiml/pkd.short.xml")


# read properties
# The Kernel object is the public interface to
# the AIML interpreter.
generic = aiml.Kernel()

# Use the 'learn' method to load the contents
# of an AIML file into the Kernel.

generic.learn("aiml/standard/hidden/std*.aiml")
#generic.learn("aiml/standard/*.aiml")
# this is from current hanson chat set

generic.learn("../generic_aiml/*.xml")


# create a character specific kernel
character=aiml.Kernel()
# load files based on botname and return filename

load(character, botname)
# load the profile, a set of key value predicates
propname="../character_aiml/"+botname+".properties"

try:
    f=open(propname)
    for line in f:
          parts = line.split('=')
          key = parts[0].strip()
          value = parts[1].strip()
          character.setBotPredicate(key, value)
          generic.setBotPredicate(key, value)
    f.close()
except:
    print "couldn't open", propname


# turn off verbose so we don't print warning on matches of early pattern matchers
character.verbose(False)
generic.verbose(False)
# Use the 'respond' method to compute the response
# to a user's input string.  respond() returns
print botname," says: Type something."
while True:
    # Loop forever, reading user input from the command
    # line and printing responses.
    search=raw_input("> ")
    if search=='reload':
        load(character,botname)
        print 'OK reloaded ',botname,'!'
    else:
        character_match=character.respond(search)
        if len(character_match)>0:
            print character_match
        else:
            print generic.respond(search)

