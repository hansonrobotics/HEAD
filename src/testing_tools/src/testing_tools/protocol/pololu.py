
class CompactProtocal(object):
    CMD_DICT = {
        0x84: 'position',
        0x87: 'speed',
        0x89: 'accelaration'
        }
    def parse(self, data):
        if not data:
            return
        assert len(data) % 4 == 0, "Wrong data length."
        instructions = []
        idx = 0
        while idx < len(data)-1:
            cmd = self.CMD_DICT[ord(data[idx])]
            id = ord(data[idx+1])
            value = (ord(data[idx+3])<<7) + ord(data[idx+2])
            idx += 4
            instructions.append((cmd, id, value))
        return instructions

