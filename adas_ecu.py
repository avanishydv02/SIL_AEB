"""ADAS ECU Node: Autonomous Emergency Braking (AEB) Controller & TTC Brain.

Connects to the virtual CAN bus (`udp_multicast` over loopback), subscribes to
`RADAR_DATA` (`0x100`) and `VEHICLE_SPEED` (`0x200`), computes Time-To-Collision
(TTC), and transmits an event-triggered `AEB_COMMAND` (`0x300`) when braking must
commence so that the ego vehicle halts with a speed-dependent inter-vehicle gap:
- Initial speed > 28 m/s  -> stops between 35m and 40m
- Initial speed <= 28 m/s -> stops between 40m and 45m
"""

from __future__ import annotations

import math
import signal
import time
from types import FrameType
from typing import Optional

import can

from can_config import (
    AEB_COMMAND_ID,
    AEB_DECELERATION_MPS2,
    HIGH_SPEED_GAP_MAX_M,
    HIGH_SPEED_GAP_MIN_M,
    INITIAL_DISTANCE_M,
    INITIAL_RELATIVE_SPEED_MPS,
    INITIAL_VEHICLE_SPEED_MPS,
    LOW_SPEED_GAP_MAX_M,
    LOW_SPEED_GAP_MIN_M,
    MIN_TTC_THRESHOLD_SEC,
    RADAR_DATA_ID,
    SPEED_REGIME_THRESHOLD_MPS,
    TARGET_STOPPING_GAP_MAX_M,
    TARGET_STOPPING_GAP_MIN_M,
    VEHICLE_SPEED_ID,
    create_can_bus,
    get_ecu_logger,
    pack_aeb_command,
    unpack_radar_data,
    unpack_vehicle_speed,
)

logger = get_ecu_logger("ADAS_ECU")


class AdasECU:
    """Central ADAS Electronic Control Unit for TTC calculation and AEB triggering."""

    def __init__(self, bus: can.BusABC) -> None:
        """Initialize the ADAS ECU state variables.

        Args:
            bus: Configured virtual CAN bus instance.
        """
        self.bus = bus
        self.distance: float = INITIAL_DISTANCE_M
        self.relative_speed: float = INITIAL_RELATIVE_SPEED_MPS
        self.vehicle_speed: float = INITIAL_VEHICLE_SPEED_MPS
        self.cruise_speed: float = INITIAL_VEHICLE_SPEED_MPS

        self.speed_before_braking: Optional[float] = None
        self.distance_before_braking: Optional[float] = None

        self._radar_received: bool = False
        self._speed_received: bool = False
        self.aeb_triggered: bool = False
        self._last_aeb_tx_time: float = 0.0
        self._terminal_reported: bool = False
        self._running: bool = True

    def stop(self) -> None:
        """Signal the ADAS processing loop to stop gracefully."""
        self._running = False

    @staticmethod
    def calculate_ttc(distance: float, relative_speed: float) -> float:
        """Calculate Time-To-Collision (TTC) in seconds, handling division by zero.

        Formula: ``ttc = abs(distance / relative_speed)``

        Args:
            distance: Current obstacle distance in meters.
            relative_speed: Current relative speed in meters per second.

        Returns:
            TTC in seconds, or ``float('inf')`` if ``relative_speed`` is zero.
        """
        if abs(relative_speed) < 1e-6:
            return float("inf")
        return abs(distance / relative_speed)

    def compute_target_stopping_gap(self) -> float:
        """Compute the speed-dependent target stopping gap.

        Rules:
        - If initial/cruise speed > 28 m/s: stop between 35m to 40m.
        - If initial/cruise speed <= 28 m/s: stop between 40m to 45m.

        Returns:
            Target stopping gap in meters.
        """
        if self.cruise_speed > SPEED_REGIME_THRESHOLD_MPS:
            # High-speed regime (> 28 m/s): map smoothly within [36.0m, 39.2m] inside (35m, 40m)
            ratio = min(1.0, max(0.0, (self.cruise_speed - SPEED_REGIME_THRESHOLD_MPS) / 32.0))
            return (HIGH_SPEED_GAP_MAX_M - 0.8) - 3.2 * ratio
        else:
            # Low/normal-speed regime (<= 28 m/s): map smoothly within [41.0m, 44.2m] inside (40m, 45m)
            ratio = min(1.0, max(0.0, self.cruise_speed / SPEED_REGIME_THRESHOLD_MPS))
            return (LOW_SPEED_GAP_MAX_M - 0.8) - 3.2 * ratio

    def compute_critical_ttc(self) -> float:
        """Compute the critical TTC threshold needed to stop at the target gap.

        Combines the 2.0s baseline TTC floor with the kinematic braking distance
        required to decelerate the vehicle to 0 m/s while preserving the speed-based
        inter-vehicle gap (35m-40m for >28 m/s, 40m-45m for <=28 m/s).

        Returns:
            Critical TTC threshold in seconds.
        """
        closing_speed = abs(self.relative_speed)
        if closing_speed < 1e-6 or self.vehicle_speed <= 0.0:
            return MIN_TTC_THRESHOLD_SEC

        target_gap = self.compute_target_stopping_gap()
        braking_duration = self.vehicle_speed / AEB_DECELERATION_MPS2
        # Average closing rate during uniform deceleration is 0.5 * closing_speed
        braking_closure_m = closing_speed * (0.5 * braking_duration)
        required_trigger_distance = target_gap + braking_closure_m

        kinematic_ttc = required_trigger_distance / closing_speed
        return max(MIN_TTC_THRESHOLD_SEC, kinematic_ttc)

    def _send_aeb_command(self, enable: bool) -> None:
        """Transmit an event-triggered `AEB_COMMAND` (0x300) frame on the CAN bus.

        Args:
            enable: ``True`` to engage emergency braking, ``False`` to release.
        """
        payload = pack_aeb_command(enable)
        msg = can.Message(
            arbitration_id=AEB_COMMAND_ID,
            data=payload,
            is_extended_id=False,
        )
        try:
            self.bus.send(msg)
            self._last_aeb_tx_time = time.monotonic()
            logger.warning(
                "TX [0x%03X] AEB_COMMAND   | Payload: %s | Brake Enabled: %s (Event-Triggered)",
                AEB_COMMAND_ID,
                payload.hex().upper(),
                enable,
            )
        except can.CanError as exc:
            logger.error("Failed to transmit AEB_COMMAND (0x300): %s", exc)

    def evaluate_safety_logic(self) -> None:
        """Evaluate TTC and trigger AEB_COMMAND (0x300) if collision is imminent."""
        if not self._radar_received or not self._speed_received:
            return

        # Check for collision (distance dropped to 0.0 m)
        if self.distance <= 0.0:
            if not self._terminal_reported:
                init_spd = (
                    self.speed_before_braking
                    if self.speed_before_braking is not None
                    else self.cruise_speed
                )
                dist_before = (
                    self.distance_before_braking
                    if self.distance_before_braking is not None
                    else 0.0
                )
                logger.critical(
                    "RESULT_SUMMARY | status=Collided | initial_speed=%.2f | distance_before_braking=%.2f | distance_maintained=0.00",
                    init_spd,
                    dist_before,
                )
                self._terminal_reported = True
            return

        ttc = self.calculate_ttc(self.distance, self.relative_speed)
        critical_ttc = self.compute_critical_ttc()
        target_gap = self.compute_target_stopping_gap()

        if math.isinf(ttc) or self.vehicle_speed == 0.0:
            if self.vehicle_speed == 0.0 and not self._terminal_reported:
                init_spd = (
                    self.speed_before_braking
                    if self.speed_before_braking is not None
                    else self.cruise_speed
                )
                dist_before = (
                    self.distance_before_braking
                    if self.distance_before_braking is not None
                    else self.distance
                )
                gap_min = (
                    HIGH_SPEED_GAP_MIN_M
                    if init_spd > SPEED_REGIME_THRESHOLD_MPS
                    else LOW_SPEED_GAP_MIN_M
                )
                gap_max = (
                    HIGH_SPEED_GAP_MAX_M
                    if init_spd > SPEED_REGIME_THRESHOLD_MPS
                    else LOW_SPEED_GAP_MAX_M
                )
                logger.info(
                    "TTC:    INF s | Vehicle STOPPED | Final Gap: %6.2f m (Target Window: %.0fm-%.0fm) | Ego Speed: 0.00 m/s",
                    self.distance,
                    gap_min,
                    gap_max,
                )
                logger.info(
                    "RESULT_SUMMARY | status=Stopped safely | initial_speed=%.2f | distance_before_braking=%.2f | distance_maintained=%.2f",
                    init_spd,
                    dist_before,
                    self.distance,
                )
                self._terminal_reported = True
            return

        logger.info(
            "TTC: %6.2f s (Crit: %.2f s) | Distance: %6.2f m | Rel Speed: %6.2f m/s | Ego Speed: %6.2f m/s | Target Gap: %.1f m",
            ttc,
            critical_ttc,
            self.distance,
            self.relative_speed,
            self.vehicle_speed,
            target_gap,
        )

        # Trigger AEB when TTC falls below the threshold and ego vehicle is moving
        if (ttc < MIN_TTC_THRESHOLD_SEC or ttc < critical_ttc) and self.vehicle_speed > 0.0:
            now = time.monotonic()
            should_transmit = (not self.aeb_triggered) or (
                self.vehicle_speed >= self.cruise_speed
                and (now - self._last_aeb_tx_time) >= 0.20
            )
            if should_transmit:
                if not self.aeb_triggered:
                    self.speed_before_braking = self.cruise_speed
                    self.distance_before_braking = self.distance
                self.aeb_triggered = True
                logger.critical(
                    "COLLISION IMMINENT! TTC=%.2fs < Threshold=%.2fs at Distance=%.2fm -> Triggering AEB (0x300)",
                    ttc,
                    critical_ttc,
                    self.distance,
                )
                self._send_aeb_command(True)

    def run(self) -> None:
        """Continuously receive Sensor and Powertrain frames and run ADAS safety logic."""
        logger.info(
            "ADAS ECU started | Listening for RADAR_DATA (0x%03X) & VEHICLE_SPEED (0x%03X) | Target Gap: %.0f-%.0f m",
            RADAR_DATA_ID,
            VEHICLE_SPEED_ID,
            TARGET_STOPPING_GAP_MIN_M,
            TARGET_STOPPING_GAP_MAX_M,
        )

        while self._running:
            try:
                msg = self.bus.recv(timeout=0.05)
            except can.CanError as exc:
                if self._running:
                    logger.error("CAN receive error on ADAS ECU: %s", exc)
                continue

            if msg is None:
                continue

            if msg.arbitration_id == RADAR_DATA_ID:
                try:
                    self.distance, self.relative_speed = unpack_radar_data(msg.data)
                    self._radar_received = True
                except Exception as exc:
                    logger.warning("Failed to unpack RADAR_DATA (0x100): %s", exc)
                    continue
                self.evaluate_safety_logic()

            elif msg.arbitration_id == VEHICLE_SPEED_ID:
                try:
                    self.vehicle_speed = unpack_vehicle_speed(msg.data)
                    self._speed_received = True
                    if not self.aeb_triggered and self.vehicle_speed > 0.0:
                        self.cruise_speed = self.vehicle_speed
                except Exception as exc:
                    logger.warning("Failed to unpack VEHICLE_SPEED (0x200): %s", exc)
                    continue
                self.evaluate_safety_logic()


def main() -> None:
    """Entry point for running the ADAS ECU node."""
    bus: Optional[can.BusABC] = None
    ecu: Optional[AdasECU] = None

    def _signal_handler(signum: int, _frame: Optional[FrameType]) -> None:
        logger.info("Received signal %d; stopping ADAS ECU...", signum)
        if ecu is not None:
            ecu.stop()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        bus = create_can_bus()
        ecu = AdasECU(bus)
        ecu.run()
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received; shutting down ADAS ECU...")
    except Exception as exc:
        logger.error("Unexpected error in ADAS ECU: %s", exc)
    finally:
        if ecu is not None:
            ecu.stop()
        if bus is not None:
            bus.shutdown()
            logger.info("ADAS ECU CAN bus connection closed cleanly.")


if __name__ == "__main__":
    main()
