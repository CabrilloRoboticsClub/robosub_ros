"""
targetpoint.py

Copyright (C) 2024-2025 Cabrillo Robotics Club

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Cabrillo Robotics Club
6500 Soquel Drive Aptos, CA 95003
cabrillorobotics@gmail.com
"""
from sys import stderr
from serial import Serial
from binascii import crc_hqx
from struct import unpack

class FailedCRC(Exception):
    pass


class MismatchedFrameID(Exception):
    pass

def get_bytes(n: int, length: int) -> bytes:
    """
    Int to bytes (big endian) wrapper.

    Args:
        n: Integer to convert into bytes.
        length: Length of bytes object to use.

    Returns:
        Bytes in big endian representing integer `n`.
    """
    return n.to_bytes(length, byteorder="big")