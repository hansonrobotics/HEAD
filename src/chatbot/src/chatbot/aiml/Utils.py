"""
Copyright 2003-2010 Cort Stratton. All rights reserved.
Copyright 2015, 2016 Hanson Robotics

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
 1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the
    distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE FREEBSD PROJECT OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

"""This file contains assorted general utility functions used by other
modules in the PyAIML package.
"""

def sentences(s):
    """Split the string s into a list of sentences."""
    try:
        s + ""
    except:
        raise TypeError, "s must be a string"
    pos = 0
    sentenceList = []
    l = len(s)
    while pos < l:
        try:
            p = s.index('.', pos)
        except:
            p = l + 1
        try:
            q = s.index('?', pos)
        except:
            q = l + 1
        try:
            e = s.index('!', pos)
        except:
            e = l + 1
        end = min(p, q, e)
        sentenceList.append(s[pos:end].strip())
        pos = end + 1
    # If no sentences were found, return a one-item list containing
    # the entire input string.
    if len(sentenceList) == 0:
        sentenceList.append(s)
    return sentenceList

# Self test
if __name__ == "__main__":
    # sentences
    sents = sentences(
        "First.  Second, still?  Third and Final!  Well, not really")
    assert(len(sents) == 4)
