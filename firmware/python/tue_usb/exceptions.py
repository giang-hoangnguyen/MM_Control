"""
Exceptions for TUE USB communication library.
"""

from typing import Optional
from .defs import StatusCode


class TueError(Exception):
    """Base exception for all TUE USB protocol and communication errors."""
    pass


class TueTimeoutError(TueError):
    """Raised when an operation times out waiting for a response."""
    pass


class TueCrcError(TueError):
    """Raised when an incoming frame fails CRC verification."""
    pass


class TueConnectionError(TueError):
    """Raised when the serial port cannot be opened or connection is broken."""
    pass


class TueProtocolError(TueError):
    """Raised when the MCU returns an error status code or violates protocol specification."""

    def __init__(self, status_code: int, message: Optional[str] = None):
        self.status_code = status_code
        self.status_name = (
            StatusCode(status_code).name
            if status_code in StatusCode._value2member_map_
            else f"0x{status_code:02X}"
        )
        desc = StatusCode.describe(status_code)
        full_msg = f"MCU returned {self.status_name} ({desc})"
        if message:
            full_msg += f": {message}"
        super().__init__(full_msg)
