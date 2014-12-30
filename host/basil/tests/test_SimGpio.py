#
# ------------------------------------------------------------
# Copyright (c) All rights reserved
# SiLab, Institute of Physics, University of Bonn
# ------------------------------------------------------------
#

import unittest
import subprocess
import os
import time
import yaml
from basil.dut import Dut
from basil.utils.sim.utils import cocotb_compile_and_run

class TestSimGpio(unittest.TestCase):
    def setUp(self):

        cocotb_compile_and_run(['test_SimGpio.v'])
        
        cnfg = yaml.load(open("test_SimGpio.yaml", 'r'))
        self.chip = Dut(cnfg)
        self.chip.init()
        
    def test_one(self):
        
        self.chip['PWR']['EN_VD1'] = 1
        self.chip['PWR']['EN_VD2'] = 1
        self.chip['PWR']['EN_VA1'] = 0
        self.chip['PWR']['EN_VA2'] = 1
        self.chip['PWR'].write()
        
        ret = self.chip['gpio'].get_data()
        
        self.assertEqual(0, 1)
    
    def tearDown(self):
        self.chip.close() # let it close connection and stop simulator
        time.sleep(1)
        subprocess.call('make clean', shell=True)
        subprocess.call('rm -f Makefile', shell=True)

if __name__ == '__main__':
    unittest.main()
