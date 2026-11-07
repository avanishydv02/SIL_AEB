"""Sensor ECU Node: Radar/Lidar Obstacle Tracking Simulation.

Connects to the virtual CAN bus (`udp_multicast` over loopback) and transmits
`RADAR_DATA` (CAN ID `0x100`) at 20 Hz (every 0.05 seconds).

Simulation Behavior:
- Default initial distance to obstacle: 200.0 meters (configurable via
  `INITIAL_DISTANCE` below or `--distance`).
- Default initial relative speed: -15.0 m/s at 25.0 m/s ego speed.
- Closed-loop feedback: Listens to `VEHICLE_SPEED` (`0x200`) from the Powertrain
  ECU so that closing speed scales with ego speed and reaches 0.0 m/s when the
  ego vehicle stops.
- Clamps distance to 0.0 m if it drops below zero (simulated impact/collision).
"""

from __future__ import annotations

import argparse
import signal
import threading
import time
from types import FrameType
from typing import Optional

import can

from can_config import (
    INITIAL_DISTANCE_M,
    INITIAL_RELATIVE_SPEED_MPS,
    INITIAL_VEHICLE_SPEED_MPS,
    PUBLISH_PERIOD_SEC,
    RADAR_DATA_ID,
    VEHICLE_SPEED_ID,
    create_can_bus,
    get_ecu_logger,
    pack_radar_data,
    unpack_vehicle_speed,
)

# ===========================================================================
# STARTING VARIABLES (Editable for Scenario Testing)
# ===========================================================================
INITIAL_DISTANCE: float = 200.0  # Initial distance to obstacle in meters (X)
INITIAL_RELATIVE_SPEED: float = -15.0  # Baseline relative speed in m/s at 25 m/s

logger = get_ecu_logger("SENSOR_ECU")


class SensorECU:
    """Radar/Lidar Sensor Electronic Control Unit simulation node."""

    def __init__(
        self,
        bus: can.BusABC,
        initial_distance: float = INITIAL_DISTANCE,
        initial_relative_speed: float = INITIAL_RELATIVE_SPEED,
    ) -> None:
        """Initialize the Sensor ECU state.

        Args:
            bus: Configured virtual CAN bus instance.
            initial_distance: Starting distance to the obstacle in meters.
            initial_relative_speed: Baseline relative speed in m/s (at 25.0 m/s ego speed).
        """
        self.bus = bus
        self.distance: float = max(0.0, float(initial_distance))
        self.base_relative_speed: float = float(initial_relative_speed)
        self.relative_speed: float = float(initial_relative_speed)
        self.ego_speed: float = INITIAL_VEHICLE_SPEED_MPS
        self._speed_received = threading.Event()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._impact_logged = False
        self._standstill_logged = False

    def stop(self) -> None:
        """Signal the ECU simulation loops to stop gracefully."""
        self._stop_event.set()

    def _listen_vehicle_speed(self) -> None:
        """Continuously listen for VEHICLE_SPEED (0x200) to close the physics loop."""
        while not self._stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.05)
            except can.CanError as exc:
                if not self._stop_event.is_set():
                    logger.error("CAN receive error on Sensor ECU: %s", exc)
                continue

            if msg is None or msg.arbitration_id != VEHICLE_SPEED_ID:
                continue

            try:
                measured_ego_speed = max(0.0, unpack_vehicle_speed(msg.data))
            except Exception as exc:
                logger.warning("Failed to unpack VEHICLE_SPEED frame: %s", exc)
                continue

            with self._lock:
                self.ego_speed = measured_ego_speed
                if INITIAL_VEHICLE_SPEED_MPS > 0.0:
                    speed_ratio = self.ego_speed / INITIAL_VEHICLE_SPEED_MPS
                else:
                    speed_ratio = 0.0
                self.relative_speed = self.base_relative_speed * speed_ratio
            self._speed_received.set()

    def run(self) -> None:
        """Execute the 20 Hz radar simulation and CAN publishing loop."""
        listener_thread = threading.Thread(
            target=self._listen_vehicle_speed,
            name="SensorSpeedListener",
            daemon=True,
        )
        listener_thread.start()

        # Wait briefly (up to 0.15s) to sync initial ego speed from Powertrain ECU
        self._speed_received.wait(timeout=0.15)

        with self._lock:
            init_dist = self.distance
            init_rel = self.relative_speed

        logger.info(
            "Sensor ECU started | Initial Distance: %.2f m | Relative Speed: %.2f m/s | Rate: 20 Hz",
            init_dist,
            init_rel,
        )

        next_tick = time.monotonic()
        first_frame = True
        while not self._stop_event.is_set():
            with self._lock:
                if not first_frame and self.distance > 0.0:
                    self.distance += self.relative_speed * PUBLISH_PERIOD_SEC
                    if self.distance <= 0.0:
                        self.distance = 0.0
                        self.relative_speed = 0.0
                first_frame = False

                current_distance = self.distance
                current_rel_speed = self.relative_speed
                current_ego_speed = self.ego_speed

            payload = pack_radar_data(current_distance, current_rel_speed)
            msg = can.Message(
                arbitration_id=RADAR_DATA_ID,
                data=payload,
                is_extended_id=False,
            )

            try:
                self.bus.send(msg)
                logger.info(
                    "TX [0x%03X] RADAR_DATA    | Payload: %s | Distance: %6.2f m | Rel Speed: %6.2f m/s",
                    RADAR_DATA_ID,
                    payload.hex().upper(),
                    current_distance,
                    current_rel_speed,
                )
            except can.CanError as exc:
                logger.error("Failed to transmit RADAR_DATA (0x100): %s", exc)

            if current_distance <= 0.0 and not self._impact_logged:
                logger.critical("SIMULATED IMPACT: Obstacle distance reached 0.00 m!")
                self._impact_logged = True
            elif (
                current_ego_speed == 0.0
                and abs(current_rel_speed) < 1e-6
                and current_distance > 0.0
                and not self._standstill_logged
            ):
                logger.info(
                    "Ego vehicle halted safely | Maintained final gap: %.2f m",
                    current_distance,
                )
                self._standstill_logged = True

            next_tick += PUBLISH_PERIOD_SEC
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                self._stop_event.wait(sleep_time)
            else:
                next_tick = time.monotonic()

        listener_thread.join(timeout=0.5)


def parse_args() -> argparse.Namespace:
    """Parse optional CLI arguments for the Sensor ECU."""
    parser = argparse.ArgumentParser(description="Sensor ECU Node (RADAR_DATA 0x100).")
    parser.add_argument(
        "--distance",
        type=float,
        default=INITIAL_DISTANCE,
        help=f"Initial distance to obstacle in meters (default: {INITIAL_DISTANCE}).",
    )
    parser.add_argument(
        "--relative-speed",
        type=float,
        default=INITIAL_RELATIVE_SPEED,
        help=f"Baseline relative speed in m/s (default: {INITIAL_RELATIVE_SPEED}).",
    )
    return parser.parse_args()


def main() -> None:
    """Entry point for running the Sensor ECU node."""
    args = parse_args()
    bus: Optional[can.BusABC] = None
    ecu: Optional[SensorECU] = None

    def _signal_handler(signum: int, _frame: Optional[FrameType]) -> None:
        logger.info("Received signal %d; stopping Sensor ECU...", signum)
        if ecu is not None:
            ecu.stop()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        bus = create_can_bus()
        ecu = SensorECU(
            bus,
            initial_distance=args.distance,
            initial_relative_speed=args.relative_speed,
        )
        ecu.run()
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received; shutting down Sensor ECU...")
    except Exception as exc:
        logger.error("Unexpected error in Sensor ECU: %s", exc)
    finally:
        if ecu is not None:
            ecu.stop()
        if bus is not None:
            bus.shutdown()
            logger.info("Sensor ECU CAN bus connection closed cleanly.")


if __name__ == "__main__":
    main()
