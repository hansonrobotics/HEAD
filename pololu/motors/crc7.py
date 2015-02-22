#
# pololu/motors/crc7.py
#
# Basic algorithm thanks to Jean-Loup Le Roux
# https://code.google.com/p/robotter/source/browse/code/crc7.py?repo=charon
#
# Modifications by Carl J. Nobile
#

CRC7_POLY = 0x91

def byte_crc7(v):
    """
    Compute CRC of a single byte.
    """
    for i in range(8):
        if v & 1:
            v ^= CRC7_POLY

        v >>= 1

    return v

CRC7_TABLE = tuple(byte_crc7(i) for i in range(256))

def crc7(data):
    """
    Compute CRC of a whole message.
    """
    crc = 0

    for c in data:
        crc = CRC7_TABLE[crc ^ c]

    return crc


if __name__ == '__main__':
    import sys

    if len(sys.argv) > 1:
        data = sys.argv[1]
    else:
        data = sys.stdin.read()

    data = [int(b) for b in data.replace('\n', ',').split(',') if b != '']
    print '\n{0:#x}, {0}'.format(crc7(data))
