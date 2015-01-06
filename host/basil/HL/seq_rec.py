from basil.HL.RegisterHardwareLayer import RegisterHardwareLayer
from struct import pack, unpack_from
from array import array


class seq_rec(RegisterHardwareLayer):
    '''Sequencer generator controller interface for seq_gen FPGA module.
    '''

    _registers = {'RESET': {'descr': {'addr': 0, 'size': 8, 'properties': ['writeonly']}},
                  'READY': {'descr': {'addr': 1, 'size': 1, 'properties': ['ro']}},
                  'START': {'descr': {'addr': 1, 'size': 8, 'properties': ['writeonly']}},
                  'CLK_DIV': {'descr': {'addr': 2, 'size': 8}},
                  'SIZE': {'descr': {'addr': 3, 'size': 16}},
                  'WAIT': {'descr': {'addr': 5, 'size': 16}},
                  'REPEAT': {'descr': {'addr': 7, 'size': 8}},
                  'REPEAT_START': {'descr': {'addr': 8, 'size': 16}},
    }

    def __init__(self, intf, conf):
        super(seq_rec, self).__init__(intf, conf)
        self._seq_mem_offset = 16  # in bytes
        try:
            self._seq_mem_size = conf['mem_size']  # in bytes
        except KeyError:
            self._seq_mem_size = 2048  # default is 2048 bytes, user should be aware of address ranges in FPGA

#    def init(self):
#        self.reset()

    def reset(self):
        self._intf.write(self._conf['base_addr'], 0)

    def start(self):
        self._intf.write(self._conf['base_addr'] + 1, 0)

    def set_size(self, value):
        self._intf.write(self._conf['base_addr'] + 3, value)

    def get_size(self):
        ret = self._intf.read(self._conf['base_addr'] + 3, size=2)
        return unpack_from('H', ret)[0]

    def set_count(self, value):
        self._intf.write(self._conf['base_addr'] + 3, (value & 0x00ff) >> 8)
        self._intf.write(self._conf['base_addr'] + 4, value & 0xFF00)

    def is_done(self):
        return self.is_ready

    @property
    def is_ready(self):
        return (self._intf.read(self._conf['base_addr'] + 1, 1)[0] & 0x01) == 1

    def get_done(self):
        return self.is_ready

    def get_data(self, size=None, addr=0):
        if size and self._seq_mem_size < size:
            raise ValueError('Size is too big')
        if not size:
            return self._intf.read(self._conf['base_addr'] + self._seq_mem_offset + addr, self._seq_mem_size)
        else:
            return self._intf.read(self._conf['base_addr'] + self._seq_mem_offset + addr, size)