"""Powertrain ECU Node: Engine, Speed Sensor, and Emergency Brake Actuator Simulation.

Connects to the virtual CAN bus (`udp_multicast` over loopback) and executes two
concurrent tasks via Python `threading`:
1. Publisher: Transmits `VEHICLE_SPEED` (CAN ID `0x200`) at 20 Hz (every 0.05 s).
2. Listener: Listens for `AEB_COMMAND` (CAN ID `0x300`). When `True` is received,
   engages full emergency braking to smoothly decelerate the ego vehicle to 0.0 m/s
   and logs `"EMERGENCY BRAKE ENGAGED - SPEED REDUCED TO 0"`.
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
    AEB_COMMAND_ID,
    AEB_DECELERATION_MPS2,
    PUBLISH_PERIOD_SEC,
    VEHICLE_SPEED_ID,
    create_can_bus,
    get_ecu_logger,
    pack_vehicle_speed,
    unpack_aeb_command,
)

# ===========================================================================
# STARTING VARIABLES (Editable for Scenario Testing)
# ===========================================================================
INITIAL_VEHICLE_SPEED: float = 25.0  # Initial ego vehicle speed in m/s (Y)

logger = get_ecu_logger("POWERTRAIN_ECU")


class PowertrainECU:
    """Powertrain and Brake Actuator Electronic Control Unit simulation node."""

    def __init__(
        self,
        bus: can.BusABC,
        initial_speed: float = INITIAL_VEHICLE_SPEED,
    ) -> None:
        """Initialize the Powertrain ECU state.

        Args:
            bus: Configured virtual CAN bus instance.
            initial_speed: Initial ego vehicle speed in m/s (default: 25.0 m/s).
        """
        self.bus = bus
        self.vehicle_speed: float = max(0.0, float(initial_speed))
        self.aeb_engaged: bool = False
        self._zero_speed_logged: bool = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

    def stop(self) -> None:
        """Signal both concurrent tasks to stop gracefully."""
        self._stop_event.set()

    def _publisher_task(self) -> None:
        """Publish `VEHICLE_SPEED` (0x200) every 0.05 seconds (20 Hz)."""
        next_tick = time.monotonic()
        while not self._stop_event.is_set():
            reached_zero_now = False
            with self._lock:
                if self.aeb_engaged and self.vehicle_speed > 0.0:
                    decel_step = AEB_DECELERATION_MPS2 * PUBLISH_PERIOD_SEC
                    self.vehicle_speed = max(0.0, self.vehicle_speed - decel_step)
                    if self.vehicle_speed == 0.0 and not self._zero_speed_logged:
                        self._zero_speed_logged = True
                        reached_zero_now = True

                current_speed = self.vehicle_speed
                braking_active = self.aeb_engaged

            payload = pack_vehicle_speed(current_speed)
            msg = can.Message(
                arbitration_id=VEHICLE_SPEED_ID,
                data=payload,
                is_extended_id=False,
            )

            try:
                self.bus.send(msg)
                state_tag = "BRAKING" if (braking_active and current_speed > 0.0) else (
                    "STOPPED" if (braking_active or current_speed == 0.0) else "CRUISING"
                )
                logger.info(
                    "TX [0x%03X] VEHICLE_SPEED | Payload: %s | Speed: %6.2f m/s (%5.1f km/h) | State: %s",
                    VEHICLE_SPEED_ID,
                    payload.hex().upper(),
                    current_speed,
                    current_speed * 3.6,
                    state_tag,
                )
            except can.CanError as exc:
                logger.error("Failed to transmit VEHICLE_SPEED (0x200): %s", exc)

            if reached_zero_now:
                logger.warning("EMERGENCY BRAKE ENGAGED - SPEED REDUCED TO 0")

            next_tick += PUBLISH_PERIOD_SEC
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                self._stop_event.wait(sleep_time)
            else:
                next_tick = time.monotonic()

    def _listener_task(self) -> None:
        """Continuously listen for `AEB_COMMAND` (0x300) frames on the CAN bus."""
        while not self._stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.05)
            except can.CanError as exc:
                if not self._stop_event.is_set():
                    logger.error("CAN receive error on Powertrain ECU: %s", exc)
                continue

            if msg is None or msg.arbitration_id != AEB_COMMAND_ID:
                continue

            try:
                brake_command = unpack_aeb_command(msg.data)
            except Exception as exc:
                logger.warning("Failed to unpack AEB_COMMAND (0x300): %s", exc)
                continue

            if brake_command:
                with self._lock:
                    already_engaged = self.aeb_engaged
                    self.aeb_engaged = True

                if not already_engaged:
                    logger.warning(
                        "RX [0x%03X] AEB_COMMAND=True | EMERGENCY BRAKE ENGAGED - SPEED REDUCED TO 0 (Decelerating at %.1f m/s^2)",
                        AEB_COMMAND_ID,
                        AEB_DECELERATION_MPS2,
                    )
            else:
                with self._lock:
                    if self.aeb_engaged and self.vehicle_speed == 0.0:
                        logger.info("RX [0x%03X] AEB_COMMAND=False | Vehicle holding at standstill.", AEB_COMMAND_ID)

    def run(self) -> None:
        """Start concurrent Publisher and Listener threads and wait until stopped."""
        logger.info(
            "Powertrain ECU started | Initial Speed: %.2f m/s (%.1f km/h) | Publish Rate: 20 Hz",
            self.vehicle_speed,
            self.vehicle_speed * 3.6,
        )

        publisher_thread = threading.Thread(
            target=self._publisher_task,
            name="PowertrainPublisher",
            daemon=True,
        )
        listener_thread = threading.Thread(
            target=self._listener_task,
            name="PowertrainListener",
            daemon=True,
        )

        listener_thread.start()
        publisher_thread.start()

        while not self._stop_event.is_set():
            self._stop_event.wait(0.1)

        publisher_thread.join(timeout=0.5)
        listener_thread.join(timeout=0.5)


def parse_args() -> argparse.Namespace:
    """Parse optional CLI arguments for the Powertrain ECU."""
    parser = argparse.ArgumentParser(description="Powertrain ECU Node (VEHICLE_SPEED 0x200).")
    parser.add_argument(
        "--speed",
        type=float,
        default=INITIAL_VEHICLE_SPEED,
        help=f"Initial ego vehicle speed in m/s (default: {INITIAL_VEHICLE_SPEED}).",
    )
    return parser.parse_args()


def main() -> None:
    """Entry point for running the Powertrain ECU node."""
    args = parse_args()
    bus: Optional[can.BusABC] = None
    ecu: Optional[PowertrainECU] = None

    def _signal_handler(signum: int, _frame: Optional[FrameType]) -> None:
        logger.info("Received signal %d; stopping Powertrain ECU...", signum)
        if ecu is not None:
            ecu.stop()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        bus = create_can_bus()
        ecu = PowertrainECU(bus, initial_speed=args.speed)
        ecu.run()
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received; shutting down Powertrain ECU...")
    except Exception as exc:
        logger.error("Unexpected error in Powertrain ECU: %s", exc)
    finally:
        if ecu is not None:
            ecu.stop()
        if bus is not None:
            bus.shutdown()
            logger.info("Powertrain ECU CAN bus connection closed cleanly.")


if __name__ == "__main__":
    main()
