# generated from rosidl_generator_py/resource/_idl.py.em
# with input from radar_msg:msg/RadarCartesian.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'x'
# Member 'y'
# Member 'z'
# Member 'gps_e'
# Member 'gps_n'
# Member 'gps_alt'
# Member 'gps_qx'
# Member 'gps_qy'
# Member 'gps_qz'
# Member 'gps_qw'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RadarCartesian(type):
    """Metaclass of message 'RadarCartesian'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('radar_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'radar_msg.msg.RadarCartesian')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__radar_cartesian
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__radar_cartesian
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__radar_cartesian
            cls._TYPE_SUPPORT = module.type_support_msg__msg__radar_cartesian
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__radar_cartesian

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RadarCartesian(metaclass=Metaclass_RadarCartesian):
    """Message class 'RadarCartesian'."""

    __slots__ = [
        '_header',
        '_stamps',
        '_x',
        '_y',
        '_z',
        '_gps_e',
        '_gps_n',
        '_gps_alt',
        '_gps_qx',
        '_gps_qy',
        '_gps_qz',
        '_gps_qw',
        '_gps_frame',
        '_robot_pose_id',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'stamps': 'sequence<builtin_interfaces/Time>',
        'x': 'sequence<float>',
        'y': 'sequence<float>',
        'z': 'sequence<float>',
        'gps_e': 'sequence<float>',
        'gps_n': 'sequence<float>',
        'gps_alt': 'sequence<float>',
        'gps_qx': 'sequence<float>',
        'gps_qy': 'sequence<float>',
        'gps_qz': 'sequence<float>',
        'gps_qw': 'sequence<float>',
        'gps_frame': 'string',
        'robot_pose_id': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.stamps = kwargs.get('stamps', [])
        self.x = array.array('f', kwargs.get('x', []))
        self.y = array.array('f', kwargs.get('y', []))
        self.z = array.array('f', kwargs.get('z', []))
        self.gps_e = array.array('f', kwargs.get('gps_e', []))
        self.gps_n = array.array('f', kwargs.get('gps_n', []))
        self.gps_alt = array.array('f', kwargs.get('gps_alt', []))
        self.gps_qx = array.array('f', kwargs.get('gps_qx', []))
        self.gps_qy = array.array('f', kwargs.get('gps_qy', []))
        self.gps_qz = array.array('f', kwargs.get('gps_qz', []))
        self.gps_qw = array.array('f', kwargs.get('gps_qw', []))
        self.gps_frame = kwargs.get('gps_frame', str())
        self.robot_pose_id = kwargs.get('robot_pose_id', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.stamps != other.stamps:
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        if self.gps_e != other.gps_e:
            return False
        if self.gps_n != other.gps_n:
            return False
        if self.gps_alt != other.gps_alt:
            return False
        if self.gps_qx != other.gps_qx:
            return False
        if self.gps_qy != other.gps_qy:
            return False
        if self.gps_qz != other.gps_qz:
            return False
        if self.gps_qw != other.gps_qw:
            return False
        if self.gps_frame != other.gps_frame:
            return False
        if self.robot_pose_id != other.robot_pose_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def stamps(self):
        """Message field 'stamps'."""
        return self._stamps

    @stamps.setter
    def stamps(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Time) for v in value) and
                 True), \
                "The 'stamps' field must be a set or sequence and each value of type 'Time'"
        self._stamps = value

    @builtins.property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'x' array.array() must have the type code of 'f'"
            self._x = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'x' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._x = array.array('f', value)

    @builtins.property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'y' array.array() must have the type code of 'f'"
            self._y = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'y' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._y = array.array('f', value)

    @builtins.property
    def z(self):
        """Message field 'z'."""
        return self._z

    @z.setter
    def z(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'z' array.array() must have the type code of 'f'"
            self._z = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'z' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._z = array.array('f', value)

    @builtins.property
    def gps_e(self):
        """Message field 'gps_e'."""
        return self._gps_e

    @gps_e.setter
    def gps_e(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'gps_e' array.array() must have the type code of 'f'"
            self._gps_e = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'gps_e' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._gps_e = array.array('f', value)

    @builtins.property
    def gps_n(self):
        """Message field 'gps_n'."""
        return self._gps_n

    @gps_n.setter
    def gps_n(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'gps_n' array.array() must have the type code of 'f'"
            self._gps_n = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'gps_n' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._gps_n = array.array('f', value)

    @builtins.property
    def gps_alt(self):
        """Message field 'gps_alt'."""
        return self._gps_alt

    @gps_alt.setter
    def gps_alt(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'gps_alt' array.array() must have the type code of 'f'"
            self._gps_alt = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'gps_alt' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._gps_alt = array.array('f', value)

    @builtins.property
    def gps_qx(self):
        """Message field 'gps_qx'."""
        return self._gps_qx

    @gps_qx.setter
    def gps_qx(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'gps_qx' array.array() must have the type code of 'f'"
            self._gps_qx = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'gps_qx' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._gps_qx = array.array('f', value)

    @builtins.property
    def gps_qy(self):
        """Message field 'gps_qy'."""
        return self._gps_qy

    @gps_qy.setter
    def gps_qy(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'gps_qy' array.array() must have the type code of 'f'"
            self._gps_qy = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'gps_qy' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._gps_qy = array.array('f', value)

    @builtins.property
    def gps_qz(self):
        """Message field 'gps_qz'."""
        return self._gps_qz

    @gps_qz.setter
    def gps_qz(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'gps_qz' array.array() must have the type code of 'f'"
            self._gps_qz = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'gps_qz' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._gps_qz = array.array('f', value)

    @builtins.property
    def gps_qw(self):
        """Message field 'gps_qw'."""
        return self._gps_qw

    @gps_qw.setter
    def gps_qw(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'gps_qw' array.array() must have the type code of 'f'"
            self._gps_qw = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'gps_qw' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._gps_qw = array.array('f', value)

    @builtins.property
    def gps_frame(self):
        """Message field 'gps_frame'."""
        return self._gps_frame

    @gps_frame.setter
    def gps_frame(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'gps_frame' field must be of type 'str'"
        self._gps_frame = value

    @builtins.property
    def robot_pose_id(self):
        """Message field 'robot_pose_id'."""
        return self._robot_pose_id

    @robot_pose_id.setter
    def robot_pose_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robot_pose_id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'robot_pose_id' field must be an unsigned integer in [0, 4294967295]"
        self._robot_pose_id = value
