from pymodbus.constants import Endian

from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder

from struct import pack

import functools


class IECPayloadBuilder(BinaryPayloadBuilder):
    def __swap_sstring_bytes(self, bytestring):
        return b''.join([bytes(list(bytestring)[s:s+2][::-1]) for s in range(0,len(bytestring),2)])

    def add_string(self, value, **kwargs):
        # encode
        value = value.encode(kwargs.get('encoding', 'utf-8'))
        # pad with zero if necessary
        if(len(value)%2 and not kwargs.get('ignore_byteorder', False)):
            value += b'\x00'

        # respect byteorder
        if(not kwargs.get('ignore_byteorder', False)):
            value = self.__swap_sstring_bytes(value)

        fstring = self._byteorder + str(len(value)) + 's'
        self._payload.append(pack(fstring, value))

    def add_word(self, value):
        value = value + [False]*(16 - len(value))
        slice_direction = 1 if self._byteorder == Endian.Little else -1
        self.add_bits(value[::slice_direction][0:8][::slice_direction])
        self.add_bits(value[::slice_direction][8:16][::slice_direction])

    def __add_multi_word(self, value, wordcount):
        value = value + [False]*(16*wordcount - len(value))

        slice_direction = 1 if self._wordorder == Endian.Little else -1
        for w in range(wordcount):
            self.add_word(value[::slice_direction][w*16:(w+1)*16][::slice_direction])

    def add_dword(self, value):
        __wordcount = 2
        self.__add_multi_word(value, __wordcount)

    def add_lword(self, value):
        __wordcount = 4
        self.__add_multi_word(value, __wordcount)


class IECPayloadDecoder(BinaryPayloadDecoder):
    def decode_byte(self):
        return self.decode_bits()
    
    def decode_word(self):
        # order bits depending on byteorder
        slice_direction = 1 if self._byteorder == Endian.Little else -1
        # when little endian, bytes are in right order and the list can be concatenated directly
        # when byteoder is big, both lists from the decode_bits operation are reversed, concatenated
        # and the resulting list reversed again
        return ((self.decode_byte()[::slice_direction]) + (self.decode_byte()[::slice_direction]))[::slice_direction]
    
    def __decode_multi_word(self, wordcount):
        # order words/registers depending on wordorder
        slice_direction = 1 if self._wordorder == Endian.Little else -1
        words = [(self.decode_word()[::slice_direction]) for _ in range(wordcount)][::slice_direction]
        return functools.reduce(lambda acc, w: acc + w, words, [])
    
    def decode_dword(self):
        __wordcount = 2
        return self.__decode_multi_word(__wordcount)
    
    def decode_lword(self):
        __wordcount = 4
        return self.__decode_multi_word(__wordcount)
    
    # expects string to be of even length (%2 == 0)!
    def __swap_sstring_bytes(self, bytestring):
        return b''.join([bytes(list(bytestring)[s:s+2][::-1]) for s in range(0,len(bytestring),2)])

    def __trim_decode_bytestring(self, bytestring, encoding):
        term_idx = bytestring.find(b'\x00')
        end = len(bytestring) if term_idx == -1 else term_idx
        return bytestring[0:end].decode(encoding)

    def decode_string(self, size=1, **kwargs):
        self._pointer += size
        s = self._payload[self._pointer - size:self._pointer]

        # swap string element pairs
        if(not kwargs.get('ignore_byteorder', False) and size > 1):
            s = self.__swap_sstring_bytes(s)
        
        return self.__trim_decode_bytestring(s, kwargs.get('encoding', 'utf-8'))

    # def decode_wstring(self, size=1, **kwargs):
    #     size *= 2 # read in "words" not in bytes
    #     self._pointer += size
    #     s = self._payload[self._pointer - size:self._pointer]

    #     # swap string element pairs
    #     s = self.__swap_sstring_bytes(s)
        
    #     return self.__trim_decode_bytestring(s, kwargs.get('encoding', 'utf-16'))

