"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class controller_states_t(object):
    __slots__ = ["mode", "timestamp", "control_torques", "error_flag", "armed"]

    __typenames__ = ["int16_t", "int64_t", "double", "int16_t", "boolean"]

    __dimensions__ = [None, None, [4], None, None]

    def __init__(self):
        self.mode = 0
        self.timestamp = 0
        self.control_torques = [ 0.0 for dim0 in range(4) ]
        self.error_flag = 0
        self.armed = False

    def encode(self):
        buf = BytesIO()
        buf.write(controller_states_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">hq", self.mode, self.timestamp))
        buf.write(struct.pack('>4d', *self.control_torques[:4]))
        buf.write(struct.pack(">hb", self.error_flag, self.armed))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != controller_states_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return controller_states_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = controller_states_t()
        self.mode, self.timestamp = struct.unpack(">hq", buf.read(10))
        self.control_torques = struct.unpack('>4d', buf.read(32))
        self.error_flag = struct.unpack(">h", buf.read(2))[0]
        self.armed = bool(struct.unpack('b', buf.read(1))[0])
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if controller_states_t in parents: return 0
        tmphash = (0xf337302ac3ec6f70) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if controller_states_t._packed_fingerprint is None:
            controller_states_t._packed_fingerprint = struct.pack(">Q", controller_states_t._get_hash_recursive([]))
        return controller_states_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", controller_states_t._get_packed_fingerprint())[0]

