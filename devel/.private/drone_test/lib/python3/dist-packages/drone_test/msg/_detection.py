# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from drone_test/detection.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class detection(genpy.Message):
  _md5sum = "156cdb81e2b7ac989e48faaa7c1e6124"
  _type = "drone_test/detection"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float64 erro_x
float64 erro_y
float64 erro_z
uint8 flag
uint8 start_opencv"""
  __slots__ = ['erro_x','erro_y','erro_z','flag','start_opencv']
  _slot_types = ['float64','float64','float64','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       erro_x,erro_y,erro_z,flag,start_opencv

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(detection, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.erro_x is None:
        self.erro_x = 0.
      if self.erro_y is None:
        self.erro_y = 0.
      if self.erro_z is None:
        self.erro_z = 0.
      if self.flag is None:
        self.flag = 0
      if self.start_opencv is None:
        self.start_opencv = 0
    else:
      self.erro_x = 0.
      self.erro_y = 0.
      self.erro_z = 0.
      self.flag = 0
      self.start_opencv = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3d2B().pack(_x.erro_x, _x.erro_y, _x.erro_z, _x.flag, _x.start_opencv))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 26
      (_x.erro_x, _x.erro_y, _x.erro_z, _x.flag, _x.start_opencv,) = _get_struct_3d2B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3d2B().pack(_x.erro_x, _x.erro_y, _x.erro_z, _x.flag, _x.start_opencv))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 26
      (_x.erro_x, _x.erro_y, _x.erro_z, _x.flag, _x.start_opencv,) = _get_struct_3d2B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3d2B = None
def _get_struct_3d2B():
    global _struct_3d2B
    if _struct_3d2B is None:
        _struct_3d2B = struct.Struct("<3d2B")
    return _struct_3d2B
