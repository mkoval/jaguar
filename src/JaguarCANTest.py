import unittest, struct
from JaguarCAN import JaguarUART, Packetizer

class PacketizerTests(unittest.TestCase):
    def setUp(self):
        self.packetizer = Packetizer(0xff, 0xfe, 0xfe, 0xfd)

    def test_RecvByte_ReturnsNone_WhenNoPacket(self):
        packet = self.packetizer.recv_byte('\xff')
        self.assertIsNone(packet)
    
    def test_RecvByte_WhenPacketIsEmpty(self):
        packet  = self.build_packet('', 0)
        payloads = self.recv_bytes(packet)
        self.assertEqual(payloads, [ None, '' ])

    def test_RecvByte_WhenPacketIsNonEmpty(self):
        packet   = self.build_packet('ab', 2)
        payloads = self.recv_bytes(packet)
        self.assertEqual(payloads, [ None, None, None, 'ab' ])

    def test_RecvByte_WhenSOFIsEscaped(self):
        packet   = self.build_packet('\xFE\xFE', 1)
        payloads = self.recv_bytes(packet)
        self.assertEqual(payloads, [ None, None, None, '\xFF' ])

    def build_packet(self, payload_raw, count):
        payload = map(ord, payload_raw)
        return struct.pack('BB{0}B'.format(len(payload)), 0xff, count, *payload)

    def recv_bytes(self, data):
        return map(self.packetizer.recv_byte, data)

if __name__ == '__main__':
    unittest.main()
