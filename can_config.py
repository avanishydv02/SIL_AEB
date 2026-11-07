"""Shared CAN bus configuration, DBC message dictionary, and payload helpers.

This module provides:
1. Virtual CAN bus creation using python-can's ``udp_multicast`` interface over the
   local network loopback (`127.0.0.1`), enabling independent ECU scripts running
   in separate OS processes/terminals to communicate cross-platform without kernel
   modules.
2. CAN Arbitration IDs and timing parameters (DBC definitions).
3. DRY ``struct`` packing and unpacking helper functions for all 8-byte CAN payloads.
4. Standardized, millisecond-timestamped console logging for all ECUs.
"""

from __future__ import annotations

import logging
import socket
import struct
import sys
from typing import Tuple

import can

# ---------------------------------------------------------------------------
# Virtual CAN Bus Configuration
# ---------------------------------------------------------------------------
# Note: While single-process simulations use can.interface.Bus('vcan0', bustype='virtual'),
# multi-process SIL across separate terminals requires 'udp_multicast' over loopback.
DEFAULT_INTERFACE: str = "udp_multicast"
MULTICAST_GROUP: str = "239.74.163.2"
MULTICAST_PORT: int = 43113
LOOPBACK_IP: str = "127.0.0.1"
VIRTUAL_CHANNEL: str = "vcan0"

# ---------------------------------------------------------------------------
# CAN Message Dictionary (DBC Definitions)
# ---------------------------------------------------------------------------
RADAR_DATA_ID: int = 0x100      # Sensor ECU -> ADAS ECU (20 Hz periodic)
VEHICLE_SPEED_ID: int = 0x200   # Powertrain ECU -> ADAS & Sensor ECUs (20 Hz periodic)
AEB_COMMAND_ID: int = 0x300     # ADAS ECU -> Powertrain ECU (Event-triggered)

CAN_PAYLOAD_SIZE: int = 8       # Standard 8-byte classic CAN frame payload
PUBLISH_PERIOD_SEC: float = 0.05  # 20 Hz transmit period (50 ms)

# Struct format strings
RADAR_DATA_FMT: str = "ff"      # Bytes 0-3: distance (float32), Bytes 4-7: rel_speed (float32)
VEHICLE_SPEED_FMT: str = "f"    # Bytes 0-3: vehicle_speed (float32)
AEB_COMMAND_FMT: str = "?"      # Byte 0: aeb_enabled (bool)

# ---------------------------------------------------------------------------
# Simulation Physics & Safety Constants
# ---------------------------------------------------------------------------
INITIAL_DISTANCE_M: float = 200.0
INITIAL_RELATIVE_SPEED_MPS: float = -15.0
INITIAL_VEHICLE_SPEED_MPS: float = 25.0

# Emergency braking deceleration rate (m/s^2) in Powertrain ECU
AEB_DECELERATION_MPS2: float = 10.0

# Safety thresholds for ADAS ECU
MIN_TTC_THRESHOLD_SEC: float = 2.0
SPEED_REGIME_THRESHOLD_MPS: float = 28.0
# If initial speed > 28 m/s -> stop between 35m to 40m
HIGH_SPEED_GAP_MIN_M: float = 35.0
HIGH_SPEED_GAP_MAX_M: float = 40.0
# If initial speed <= 28 m/s -> stop between 40m to 45m
LOW_SPEED_GAP_MIN_M: float = 40.0
LOW_SPEED_GAP_MAX_M: float = 45.0
TARGET_STOPPING_GAP_MIN_M: float = 35.0
TARGET_STOPPING_GAP_MAX_M: float = 45.0


def create_can_bus(
    interface: str = DEFAULT_INTERFACE,
    channel: str = MULTICAST_GROUP,
    port: int = MULTICAST_PORT,
) -> can.BusABC:
    """Create and return a configured virtual CAN bus instance.

    Uses ``udp_multicast`` bound to the local loopback interface (`127.0.0.1`)
    by default so separate terminal processes can exchange CAN frames reliably
    across macOS, Linux, and Windows without OS kernel drivers. Also supports
    ``interface='virtual'`` (e.g., ``can.interface.Bus('vcan0', bustype='virtual')``)
    when requested for in-process testing.

    Args:
        interface: The python-can bus interface type (default: ``'udp_multicast'``).
        channel: The multicast IPv4 group address or virtual channel name.
        port: The UDP port used for multicast communication.

    Returns:
        An active :class:`can.BusABC` instance.
    """
    if interface == "virtual":
        return can.interface.Bus(channel=VIRTUAL_CHANNEL, interface="virtual")

    bus = can.interface.Bus(
        channel=channel,
        interface=interface,
        port=port,
        hop_limit=1,
        receive_own_messages=False,
        fd=False,
    )

    # Bind multicast traffic explicitly to local loopback (127.0.0.1) so multi-process
    # communication works offline and on macOS without 'No route to host' errors.
    multicast_impl = getattr(bus, "_multicast", None)
    raw_sock = getattr(multicast_impl, "_socket", None)
    if isinstance(raw_sock, socket.socket):
        loopback_bytes = socket.inet_aton(LOOPBACK_IP)
        group_bytes = socket.inet_aton(channel)
        raw_sock.setsockopt(
            socket.IPPROTO_IP, socket.IP_MULTICAST_IF, loopback_bytes
        )
        raw_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        try:
            raw_sock.setsockopt(
                socket.IPPROTO_IP,
                socket.IP_ADD_MEMBERSHIP,
                group_bytes + loopback_bytes,
            )
        except OSError:
            # Membership on loopback interface is already registered.
            pass

    return bus


# ---------------------------------------------------------------------------
# Struct Packing / Unpacking Helpers (8-Byte CAN Payloads)
# ---------------------------------------------------------------------------
def pack_radar_data(distance: float, relative_speed: float) -> bytes:
    """Pack RADAR_DATA (0x100) into an 8-byte CAN payload using format ``'ff'``.

    Args:
        distance: Distance to the obstacle in front in meters (bytes 0-3).
        relative_speed: Relative speed in meters per second (bytes 4-7).

    Returns:
        8-byte packed payload.
    """
    return struct.pack(RADAR_DATA_FMT, float(distance), float(relative_speed))


def unpack_radar_data(data: bytes) -> Tuple[float, float]:
    """Unpack RADAR_DATA (0x100) from a CAN payload using format ``'ff'``.

    Args:
        data: Raw CAN frame payload (at least 8 bytes).

    Returns:
        A tuple of ``(distance, relative_speed)``.
    """
    distance, relative_speed = struct.unpack(RADAR_DATA_FMT, bytes(data[:8]))
    return float(distance), float(relative_speed)


def pack_vehicle_speed(vehicle_speed: float) -> bytes:
    """Pack VEHICLE_SPEED (0x200) into an 8-byte CAN payload using format ``'f'``.

    Bytes 0-3 store the 32-bit float vehicle speed; bytes 4-7 are zero-padded
    to form a standard 8-byte CAN frame.

    Args:
        vehicle_speed: Current ego vehicle speed in meters per second.

    Returns:
        8-byte packed payload.
    """
    packed = struct.pack(VEHICLE_SPEED_FMT, float(vehicle_speed))
    return packed.ljust(CAN_PAYLOAD_SIZE, b"\x00")


def unpack_vehicle_speed(data: bytes) -> float:
    """Unpack VEHICLE_SPEED (0x200) from bytes 0-3 of a CAN payload using format ``'f'``.

    Args:
        data: Raw CAN frame payload (at least 4 bytes).

    Returns:
        The ego vehicle speed in meters per second.
    """
    (vehicle_speed,) = struct.unpack(VEHICLE_SPEED_FMT, bytes(data[:4]))
    return float(vehicle_speed)


def pack_aeb_command(enable_brake: bool) -> bytes:
    """Pack AEB_COMMAND (0x300) into an 8-byte CAN payload using format ``'?'``.

    Byte 0 stores the boolean emergency brake state; bytes 1-7 are zero-padded
    to form a standard 8-byte CAN frame.

    Args:
        enable_brake: ``True`` to engage emergency braking, ``False`` to release.

    Returns:
        8-byte packed payload.
    """
    packed = struct.pack(AEB_COMMAND_FMT, bool(enable_brake))
    return packed.ljust(CAN_PAYLOAD_SIZE, b"\x00")


def unpack_aeb_command(data: bytes) -> bool:
    """Unpack AEB_COMMAND (0x300) from byte 0 of a CAN payload using format ``'?'``.

    Args:
        data: Raw CAN frame payload (at least 1 byte).

    Returns:
        ``True`` if emergency braking is enabled, ``False`` otherwise.
    """
    (enable_brake,) = struct.unpack(AEB_COMMAND_FMT, bytes(data[:1]))
    return bool(enable_brake)


# ---------------------------------------------------------------------------
# Logging Helper
# ---------------------------------------------------------------------------
def get_ecu_logger(ecu_name: str) -> logging.Logger:
    """Create a standardized, millisecond-timestamped console logger for an ECU node.

    Args:
        ecu_name: Identifier of the ECU node (e.g., ``'SENSOR_ECU'``).

    Returns:
        Configured :class:`logging.Logger` instance.
    """
    logger = logging.getLogger(ecu_name)
    if not logger.handlers:
        logger.setLevel(logging.INFO)
        handler = logging.StreamHandler(sys.stdout)
        formatter = logging.Formatter(
            fmt="%(asctime)s.%(msecs)03d | %(name)-14s | %(levelname)-5s | %(message)s",
            datefmt="%H:%M:%S",
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        logger.propagate = False
    return logger

