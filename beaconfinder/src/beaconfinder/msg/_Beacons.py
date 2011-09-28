"""autogenerated by genmsg_py from Beacons.msg. Do not edit."""
import roslib.message
import struct

import beaconfinder.msg
import std_msgs.msg

class Beacons(roslib.message.Message):
  _md5sum = "873d03fb58b312326f8bc5e4bda6e56b"
  _type = "beaconfinder/Beacons"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """#Beacons message for publishing a list of beacons from a laser scan to a topic
Header header
uint8 numBeacons
# A list of Beacons found at this timestamp
Beacon[] beacon

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: beaconfinder/Beacon
# Beacon message type, for a single beacon

# These IDs go from 0..n, for n beacons in increasing order of size
uint8 ID

# The x coordinate (robot relative) of the centre of the beacon
float32 x

# The y coordinate (robot relative) of the centre of the beacon
float32 y

"""
  __slots__ = ['header','numBeacons','beacon']
  _slot_types = ['Header','uint8','beaconfinder/Beacon[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       header,numBeacons,beacon
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(Beacons, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      if self.numBeacons is None:
        self.numBeacons = 0
      if self.beacon is None:
        self.beacon = []
    else:
      self.header = std_msgs.msg._Header.Header()
      self.numBeacons = 0
      self.beacon = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.numBeacons))
      length = len(self.beacon)
      buff.write(_struct_I.pack(length))
      for val1 in self.beacon:
        _x = val1
        buff.write(_struct_B2f.pack(_x.ID, _x.x, _x.y))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      start = end
      end += 1
      (self.numBeacons,) = _struct_B.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.beacon = []
      for i in xrange(0, length):
        val1 = beaconfinder.msg.Beacon()
        _x = val1
        start = end
        end += 9
        (_x.ID, _x.x, _x.y,) = _struct_B2f.unpack(str[start:end])
        self.beacon.append(val1)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.numBeacons))
      length = len(self.beacon)
      buff.write(_struct_I.pack(length))
      for val1 in self.beacon:
        _x = val1
        buff.write(_struct_B2f.pack(_x.ID, _x.x, _x.y))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      start = end
      end += 1
      (self.numBeacons,) = _struct_B.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.beacon = []
      for i in xrange(0, length):
        val1 = beaconfinder.msg.Beacon()
        _x = val1
        start = end
        end += 9
        (_x.ID, _x.x, _x.y,) = _struct_B2f.unpack(str[start:end])
        self.beacon.append(val1)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_B2f = struct.Struct("<B2f")
_struct_3I = struct.Struct("<3I")
_struct_B = struct.Struct("<B")
