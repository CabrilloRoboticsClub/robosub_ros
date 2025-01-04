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

# Table 7-2: TPTCM Command Set
# Map command name to command frame ID
_cmd_frame_id = {
    "kGetModInfo":          get_bytes(0x01, 1),  # Queries the device’s type and firmware revision.
    "kSetDataComponents":   get_bytes(0x03, 1),  # Sets the data components to be output.
    "kGetData":             get_bytes(0x04, 1),  # Queries the TPTCM for data
}

# Table 7-6: Component Identifiers
# Map component name to component IDs
_data_comp = {
    "kHeading":         0x05,
    "kPitch":           0x18,
    "kRoll":            0x19,
    "kHeadingStatus":   0x4F,
    "kQuaternion":      0x4D,
    "kTemperature":     0x07,
    "kDistortion":      0x08,
    "kCalStatus":       0x09,
    "kAccelX":          0x15,
    "kAccelY":          0x16,
    "kAccelZ":          0x17,
    "kMagX":            0x1B,
    "kMagY":            0x1C,
    "kMagZ":            0x1D,
    "kGyroX":           0x4A,
    "kGyroY":           0x4B,
    "kGyroZ":           0x4C,
}