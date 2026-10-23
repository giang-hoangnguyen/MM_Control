"""
CRC-16/CCITT-FALSE calculation and validation module.

Polynomial: 0x1021
Initial value: 0xFFFF
RefIn: False, RefOut: False, XorOut: 0x0000
Check value for b"123456789": 0x29B1
"""

from typing import Union

# Precomputed 256-entry lookup table for CRC-16/CCITT-FALSE (poly 0x1021)
_CRC16_TABLE: list[int] = []
for _i in range(256):
    _curr = (_i << 8) & 0xFFFF
    for _ in range(8):
        if _curr & 0x8000:
            _curr = ((_curr << 1) ^ 0x1021) & 0xFFFF
        else:
            _curr = (_curr << 1) & 0xFFFF
    _CRC16_TABLE.append(_curr)


def crc16_ccitt(data: Union[bytes, bytearray, memoryview], init_crc: int = 0xFFFF) -> int:
    """Calculate CRC-16/CCITT-FALSE over byte data using fast table lookup.

    Args:
        data: Buffer of bytes to hash.
        init_crc: Initial CRC value, defaults to 0xFFFF.

    Returns:
        Computed 16-bit unsigned integer CRC.
    """
    crc = init_crc & 0xFFFF
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC16_TABLE[((crc >> 8) ^ b) & 0xFF]
    return crc


def verify_crc16(data: Union[bytes, bytearray, memoryview], expected_crc: int) -> bool:
    """Verify that data produces the expected CRC-16 value.

    Args:
        data: Buffer of bytes to hash.
        expected_crc: The 16-bit CRC value to compare against.

    Returns:
        True if checksum matches, False otherwise.
    """
    return (crc16_ccitt(data) & 0xFFFF) == (expected_crc & 0xFFFF)
