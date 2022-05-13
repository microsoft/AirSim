class RPCError(Exception):
    CODE = ".RPCError"

    def __init__(self, message):
        Exception.__init__(self, message)

    @property
    def code(self):
        return self.__class__.CODE

    def to_msgpack(self):
        return [self.message]

    @staticmethod
    def from_msgpack(message):
        return RPCError(message)

class TimeoutError(RPCError):
    CODE = ".TimeoutError"
    pass

class TransportError(RPCError):
    CODE = ".TransportError"
    pass

class CallError(RPCError):
    CODE = ".NoMethodError"
    pass

class NoMethodError(CallError):
    CODE = ".CallError.NoMethodError"
    pass

class ArgumentError(CallError):
    CODE = ".CallError.ArgumentError"
    pass
