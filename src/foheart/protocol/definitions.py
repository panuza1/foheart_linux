class ProtocolError(RuntimeError):
    pass


class MalformedPayloadError(ProtocolError):
    pass


class ProtocolNotDecodedError(ProtocolError):
    pass


class UnsupportedProtocolError(ProtocolError):
    pass

