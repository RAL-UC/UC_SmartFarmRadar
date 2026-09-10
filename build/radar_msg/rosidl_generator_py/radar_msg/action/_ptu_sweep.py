# generated from rosidl_generator_py/resource/_idl.py.em
# with input from radar_msg:action/PtuSweep.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PtuSweep_Goal(type):
    """Metaclass of message 'PtuSweep_Goal'."""

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
                'radar_msg.action.PtuSweep_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__ptu_sweep__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__ptu_sweep__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__ptu_sweep__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__ptu_sweep__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__ptu_sweep__goal

            from radar_msg.msg import Ptu
            if Ptu.__class__._TYPE_SUPPORT is None:
                Ptu.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PtuSweep_Goal(metaclass=Metaclass_PtuSweep_Goal):
    """Message class 'PtuSweep_Goal'."""

    __slots__ = [
        '_target_ptu',
    ]

    _fields_and_field_types = {
        'target_ptu': 'radar_msg/Ptu',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['radar_msg', 'msg'], 'Ptu'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from radar_msg.msg import Ptu
        self.target_ptu = kwargs.get('target_ptu', Ptu())

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
        if self.target_ptu != other.target_ptu:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def target_ptu(self):
        """Message field 'target_ptu'."""
        return self._target_ptu

    @target_ptu.setter
    def target_ptu(self, value):
        if __debug__:
            from radar_msg.msg import Ptu
            assert \
                isinstance(value, Ptu), \
                "The 'target_ptu' field must be a sub message of type 'Ptu'"
        self._target_ptu = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PtuSweep_Result(type):
    """Metaclass of message 'PtuSweep_Result'."""

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
                'radar_msg.action.PtuSweep_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__ptu_sweep__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__ptu_sweep__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__ptu_sweep__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__ptu_sweep__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__ptu_sweep__result

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PtuSweep_Result(metaclass=Metaclass_PtuSweep_Result):
    """Message class 'PtuSweep_Result'."""

    __slots__ = [
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

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
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PtuSweep_Feedback(type):
    """Metaclass of message 'PtuSweep_Feedback'."""

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
                'radar_msg.action.PtuSweep_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__ptu_sweep__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__ptu_sweep__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__ptu_sweep__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__ptu_sweep__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__ptu_sweep__feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PtuSweep_Feedback(metaclass=Metaclass_PtuSweep_Feedback):
    """Message class 'PtuSweep_Feedback'."""

    __slots__ = [
        '_current_pan_deg',
        '_current_tilt_deg',
        '_status',
    ]

    _fields_and_field_types = {
        'current_pan_deg': 'int32',
        'current_tilt_deg': 'int32',
        'status': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.current_pan_deg = kwargs.get('current_pan_deg', int())
        self.current_tilt_deg = kwargs.get('current_tilt_deg', int())
        self.status = kwargs.get('status', str())

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
        if self.current_pan_deg != other.current_pan_deg:
            return False
        if self.current_tilt_deg != other.current_tilt_deg:
            return False
        if self.status != other.status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def current_pan_deg(self):
        """Message field 'current_pan_deg'."""
        return self._current_pan_deg

    @current_pan_deg.setter
    def current_pan_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_pan_deg' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'current_pan_deg' field must be an integer in [-2147483648, 2147483647]"
        self._current_pan_deg = value

    @builtins.property
    def current_tilt_deg(self):
        """Message field 'current_tilt_deg'."""
        return self._current_tilt_deg

    @current_tilt_deg.setter
    def current_tilt_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_tilt_deg' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'current_tilt_deg' field must be an integer in [-2147483648, 2147483647]"
        self._current_tilt_deg = value

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'status' field must be of type 'str'"
        self._status = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PtuSweep_SendGoal_Request(type):
    """Metaclass of message 'PtuSweep_SendGoal_Request'."""

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
                'radar_msg.action.PtuSweep_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__ptu_sweep__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__ptu_sweep__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__ptu_sweep__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__ptu_sweep__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__ptu_sweep__send_goal__request

            from radar_msg.action import PtuSweep
            if PtuSweep.Goal.__class__._TYPE_SUPPORT is None:
                PtuSweep.Goal.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PtuSweep_SendGoal_Request(metaclass=Metaclass_PtuSweep_SendGoal_Request):
    """Message class 'PtuSweep_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'radar_msg/PtuSweep_Goal',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['radar_msg', 'action'], 'PtuSweep_Goal'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from radar_msg.action._ptu_sweep import PtuSweep_Goal
        self.goal = kwargs.get('goal', PtuSweep_Goal())

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
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            from radar_msg.action._ptu_sweep import PtuSweep_Goal
            assert \
                isinstance(value, PtuSweep_Goal), \
                "The 'goal' field must be a sub message of type 'PtuSweep_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PtuSweep_SendGoal_Response(type):
    """Metaclass of message 'PtuSweep_SendGoal_Response'."""

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
                'radar_msg.action.PtuSweep_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__ptu_sweep__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__ptu_sweep__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__ptu_sweep__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__ptu_sweep__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__ptu_sweep__send_goal__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PtuSweep_SendGoal_Response(metaclass=Metaclass_PtuSweep_SendGoal_Response):
    """Message class 'PtuSweep_SendGoal_Response'."""

    __slots__ = [
        '_accepted',
        '_stamp',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'stamp': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

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
        if self.accepted != other.accepted:
            return False
        if self.stamp != other.stamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value


class Metaclass_PtuSweep_SendGoal(type):
    """Metaclass of service 'PtuSweep_SendGoal'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('radar_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'radar_msg.action.PtuSweep_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__ptu_sweep__send_goal

            from radar_msg.action import _ptu_sweep
            if _ptu_sweep.Metaclass_PtuSweep_SendGoal_Request._TYPE_SUPPORT is None:
                _ptu_sweep.Metaclass_PtuSweep_SendGoal_Request.__import_type_support__()
            if _ptu_sweep.Metaclass_PtuSweep_SendGoal_Response._TYPE_SUPPORT is None:
                _ptu_sweep.Metaclass_PtuSweep_SendGoal_Response.__import_type_support__()


class PtuSweep_SendGoal(metaclass=Metaclass_PtuSweep_SendGoal):
    from radar_msg.action._ptu_sweep import PtuSweep_SendGoal_Request as Request
    from radar_msg.action._ptu_sweep import PtuSweep_SendGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PtuSweep_GetResult_Request(type):
    """Metaclass of message 'PtuSweep_GetResult_Request'."""

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
                'radar_msg.action.PtuSweep_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__ptu_sweep__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__ptu_sweep__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__ptu_sweep__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__ptu_sweep__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__ptu_sweep__get_result__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PtuSweep_GetResult_Request(metaclass=Metaclass_PtuSweep_GetResult_Request):
    """Message class 'PtuSweep_GetResult_Request'."""

    __slots__ = [
        '_goal_id',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())

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
        if self.goal_id != other.goal_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PtuSweep_GetResult_Response(type):
    """Metaclass of message 'PtuSweep_GetResult_Response'."""

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
                'radar_msg.action.PtuSweep_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__ptu_sweep__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__ptu_sweep__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__ptu_sweep__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__ptu_sweep__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__ptu_sweep__get_result__response

            from radar_msg.action import PtuSweep
            if PtuSweep.Result.__class__._TYPE_SUPPORT is None:
                PtuSweep.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PtuSweep_GetResult_Response(metaclass=Metaclass_PtuSweep_GetResult_Response):
    """Message class 'PtuSweep_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'radar_msg/PtuSweep_Result',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['radar_msg', 'action'], 'PtuSweep_Result'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        from radar_msg.action._ptu_sweep import PtuSweep_Result
        self.result = kwargs.get('result', PtuSweep_Result())

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
        if self.status != other.status:
            return False
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            from radar_msg.action._ptu_sweep import PtuSweep_Result
            assert \
                isinstance(value, PtuSweep_Result), \
                "The 'result' field must be a sub message of type 'PtuSweep_Result'"
        self._result = value


class Metaclass_PtuSweep_GetResult(type):
    """Metaclass of service 'PtuSweep_GetResult'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('radar_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'radar_msg.action.PtuSweep_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__ptu_sweep__get_result

            from radar_msg.action import _ptu_sweep
            if _ptu_sweep.Metaclass_PtuSweep_GetResult_Request._TYPE_SUPPORT is None:
                _ptu_sweep.Metaclass_PtuSweep_GetResult_Request.__import_type_support__()
            if _ptu_sweep.Metaclass_PtuSweep_GetResult_Response._TYPE_SUPPORT is None:
                _ptu_sweep.Metaclass_PtuSweep_GetResult_Response.__import_type_support__()


class PtuSweep_GetResult(metaclass=Metaclass_PtuSweep_GetResult):
    from radar_msg.action._ptu_sweep import PtuSweep_GetResult_Request as Request
    from radar_msg.action._ptu_sweep import PtuSweep_GetResult_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PtuSweep_FeedbackMessage(type):
    """Metaclass of message 'PtuSweep_FeedbackMessage'."""

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
                'radar_msg.action.PtuSweep_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__ptu_sweep__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__ptu_sweep__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__ptu_sweep__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__ptu_sweep__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__ptu_sweep__feedback_message

            from radar_msg.action import PtuSweep
            if PtuSweep.Feedback.__class__._TYPE_SUPPORT is None:
                PtuSweep.Feedback.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PtuSweep_FeedbackMessage(metaclass=Metaclass_PtuSweep_FeedbackMessage):
    """Message class 'PtuSweep_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'radar_msg/PtuSweep_Feedback',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['radar_msg', 'action'], 'PtuSweep_Feedback'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from radar_msg.action._ptu_sweep import PtuSweep_Feedback
        self.feedback = kwargs.get('feedback', PtuSweep_Feedback())

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
        if self.goal_id != other.goal_id:
            return False
        if self.feedback != other.feedback:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if __debug__:
            from radar_msg.action._ptu_sweep import PtuSweep_Feedback
            assert \
                isinstance(value, PtuSweep_Feedback), \
                "The 'feedback' field must be a sub message of type 'PtuSweep_Feedback'"
        self._feedback = value


class Metaclass_PtuSweep(type):
    """Metaclass of action 'PtuSweep'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('radar_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'radar_msg.action.PtuSweep')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__ptu_sweep

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from radar_msg.action import _ptu_sweep
            if _ptu_sweep.Metaclass_PtuSweep_SendGoal._TYPE_SUPPORT is None:
                _ptu_sweep.Metaclass_PtuSweep_SendGoal.__import_type_support__()
            if _ptu_sweep.Metaclass_PtuSweep_GetResult._TYPE_SUPPORT is None:
                _ptu_sweep.Metaclass_PtuSweep_GetResult.__import_type_support__()
            if _ptu_sweep.Metaclass_PtuSweep_FeedbackMessage._TYPE_SUPPORT is None:
                _ptu_sweep.Metaclass_PtuSweep_FeedbackMessage.__import_type_support__()


class PtuSweep(metaclass=Metaclass_PtuSweep):

    # The goal message defined in the action definition.
    from radar_msg.action._ptu_sweep import PtuSweep_Goal as Goal
    # The result message defined in the action definition.
    from radar_msg.action._ptu_sweep import PtuSweep_Result as Result
    # The feedback message defined in the action definition.
    from radar_msg.action._ptu_sweep import PtuSweep_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from radar_msg.action._ptu_sweep import PtuSweep_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from radar_msg.action._ptu_sweep import PtuSweep_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from radar_msg.action._ptu_sweep import PtuSweep_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')
