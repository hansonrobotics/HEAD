#!/usr/bin/env python

import unittest
import time
import os
import subprocess

from testing_tools.misc import PololuSerialReader
from pololu.motors import Maestro

CWD = os.path.abspath(os.path.dirname(__file__))

class TestROSPololu(unittest.TestCase):
    def setUp(self):
        self.port0 = '%s/%s' % (CWD, 'port0')
        self.port1 = '%s/%s' % (CWD, 'port1')
        cmd = 'socat -d -d pty,link=%s,raw,echo=0 pty,link=%s,raw,echo=0' % (
            self.port0, self.port1)
        self.proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
        while not os.path.islink(self.port0):
            time.sleep(0.02)

    def tearDown(self):
        os.killpg(self.proc.pid, 2)

    def test_port_io(self):
        controller = Maestro(self.port0)
        reader = PololuSerialReader(self.port1)
        for i in range(10):
            pulses = [6143, 6122, 5899, 5085, 5352, 5810, 6075, 6143]*100
            for pulse in pulses:
                controller.setAcceleration(12, 255)
                controller.setSpeed(12, 256)
                controller.setTarget(12, pulse)

            pulses_got = []
            for i in range(3*len(pulses)):
                id, cmd, value = reader.read()
                if cmd == 'speed':
                    self.assertEqual(value, 256)
                if cmd == 'accelaration':
                    self.assertEqual(value, 255)
                if cmd == 'position':
                    pulses_got.append(value)
            self.assertListEqual(pulses, pulses_got)

if __name__ == '__main__':
    unittest.main()

