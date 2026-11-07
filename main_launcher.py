"""SIL AEB Multi-Process Orchestrator (Interactive Main Launcher).

Provides an interactive menu loop to run the 3-ECU (`powertrain_ecu.py`,
`adas_ecu.py`, `sensor_ecu.py`) Software-in-the-Loop (SIL) simulation over the
`udp_multicast` virtual CAN bus:

    1. Automated test run
    2. Test different scenarios
    3. Exit

Stopping gap rules:
- Initial speed > 28 m/s  -> stops between 35m and 40m
- Initial speed <= 28 m/s -> stops between 40m and 45m

When the vehicle either stops completely ("Stopped safely") or crashes ("Collided"),
the continuous CAN log stream immediately halts and displays the final summary:
- Vehicle status
- Initial speed of vehicle (before engaging brakes)
- Distance to vehicle before engaging brakes
- Distance maintained (after the result)
"""

from __future__ import annotations

import argparse
import os
import platform
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from can_config import (
    INITIAL_DISTANCE_M,
    INITIAL_VEHICLE_SPEED_MPS,
    get_ecu_logger,
)

logger = get_ecu_logger("MAIN_LAUNCHER")

ECU_SCRIPTS: List[str] = [
    "powertrain_ecu.py",
    "adas_ecu.py",
    "sensor_ecu.py",
]

ANSI_COLORS: Dict[str, str] = {
    "powertrain_ecu.py": "\033[96m",  # Cyan
    "adas_ecu.py": "\033[93m",        # Yellow
    "sensor_ecu.py": "\033[92m",      # Green
}
ANSI_BOLD: str = "\033[1m"
ANSI_GREEN: str = "\033[92m"
ANSI_RED: str = "\033[91m"
ANSI_RESET: str = "\033[0m"


@dataclass
class SimulationResult:
    """Final metrics captured at the end of a simulation run."""

    vehicle_status: str
    initial_speed: float
    distance_before_braking: float
    distance_maintained: float


def read_starting_variables_from_files(project_dir: Path) -> Tuple[float, float]:
    """Read current `INITIAL_DISTANCE` and `INITIAL_VEHICLE_SPEED` from the ECU files.

    Args:
        project_dir: Root directory containing `sensor_ecu.py` and `powertrain_ecu.py`.

    Returns:
        Tuple of ``(initial_distance_m, initial_vehicle_speed_mps)``.
    """
    sensor_file = project_dir / "sensor_ecu.py"
    powertrain_file = project_dir / "powertrain_ecu.py"

    distance = INITIAL_DISTANCE_M
    speed = INITIAL_VEHICLE_SPEED_MPS

    if sensor_file.exists():
        content = sensor_file.read_text(encoding="utf-8")
        match = re.search(
            r"^INITIAL_DISTANCE\s*(?::\s*float)?\s*=\s*([0-9]+(?:\.[0-9]+)?)",
            content,
            re.MULTILINE,
        )
        if match:
            distance = float(match.group(1))

    if powertrain_file.exists():
        content = powertrain_file.read_text(encoding="utf-8")
        match = re.search(
            r"^INITIAL_VEHICLE_SPEED\s*(?::\s*float)?\s*=\s*([0-9]+(?:\.[0-9]+)?)",
            content,
            re.MULTILINE,
        )
        if match:
            speed = float(match.group(1))

    return distance, speed


def write_starting_variables_to_files(
    project_dir: Path,
    distance: float,
    speed: float,
) -> None:
    """Update `INITIAL_DISTANCE` in `sensor_ecu.py` and `INITIAL_VEHICLE_SPEED` in `powertrain_ecu.py`.

    Args:
        project_dir: Root directory containing the ECU scripts.
        distance: New starting distance (X) in meters.
        speed: New starting ego vehicle speed (Y) in m/s.
    """
    sensor_file = project_dir / "sensor_ecu.py"
    powertrain_file = project_dir / "powertrain_ecu.py"

    if sensor_file.exists():
        content = sensor_file.read_text(encoding="utf-8")
        updated = re.sub(
            r"^(INITIAL_DISTANCE\s*(?::\s*float)?\s*=\s*)[0-9]+(?:\.[0-9]+)?",
            rf"\g<1>{distance:.1f}",
            content,
            count=1,
            flags=re.MULTILINE,
        )
        sensor_file.write_text(updated, encoding="utf-8")

    if powertrain_file.exists():
        content = powertrain_file.read_text(encoding="utf-8")
        updated = re.sub(
            r"^(INITIAL_VEHICLE_SPEED\s*(?::\s*float)?\s*=\s*)[0-9]+(?:\.[0-9]+)?",
            rf"\g<1>{speed:.1f}",
            content,
            count=1,
            flags=re.MULTILINE,
        )
        powertrain_file.write_text(updated, encoding="utf-8")


def print_simulation_result(result: SimulationResult) -> None:
    """Display the final formatted simulation summary table.

    Args:
        result: Captured :class:`SimulationResult` metrics.
    """
    status_color = (
        ANSI_GREEN if result.vehicle_status == "Stopped safely" else ANSI_RED
    )
    print("\n" + "=" * 68)
    print(f"{ANSI_BOLD}                      FINAL SIMULATION RESULT{ANSI_RESET}")
    print("=" * 68)
    print(
        f"  Vehicle status                            : "
        f"{ANSI_BOLD}{status_color}{result.vehicle_status}{ANSI_RESET}"
    )
    print(
        f"  Initial speed of vehicle                  : "
        f"{result.initial_speed:.2f} m/s ({result.initial_speed * 3.6:.1f} km/h)"
    )
    print(
        f"  Distance to vehicle before engaging brakes: "
        f"{result.distance_before_braking:.2f} m"
    )
    print(
        f"  Distance maintained                       : "
        f"{result.distance_maintained:.2f} m"
    )
    print("=" * 68 + "\n")
    sys.stdout.flush()


def run_simulation_once(
    project_dir: Path,
    python_exe: str,
    initial_distance: float,
    initial_speed: float,
) -> Optional[SimulationResult]:
    """Run one complete simulation cycle until the vehicle stops safely or collides.

    Immediately stops streaming CAN output once the terminal state ("Stopped safely"
    or "Collided") is reached, shuts down all three ECU processes cleanly, and
    returns the final :class:`SimulationResult`.

    Args:
        project_dir: Directory containing the ECU scripts.
        python_exe: Path to the Python interpreter.
        initial_distance: Initial distance to obstacle in meters.
        initial_speed: Initial vehicle speed in m/s.

    Returns:
        :class:`SimulationResult` if completed, or ``None`` if interrupted early.
    """
    processes: List[subprocess.Popen] = []
    result_holder: List[SimulationResult] = []
    stream_gate = threading.Event()
    result_lock = threading.Lock()

    state_tracker: Dict[str, float] = {
        "initial_speed": initial_speed,
        "distance_before_braking": initial_distance,
        "latest_distance": initial_distance,
        "aeb_seen": 0.0,
    }

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"

    summary_pattern = re.compile(
        r"RESULT_SUMMARY\s*\|\s*status=([^|]+?)\s*\|\s*"
        r"initial_speed=([0-9.]+)\s*\|\s*"
        r"distance_before_braking=([0-9.]+)\s*\|\s*"
        r"distance_maintained=([0-9.]+)"
    )
    aeb_trigger_pattern = re.compile(
        r"COLLISION IMMINENT!.*Distance=([0-9.]+)m"
    )
    radar_dist_pattern = re.compile(r"Distance:\s*([0-9.]+)\s*m")

    def _stream_output(proc: subprocess.Popen, script_name: str) -> None:
        color = ANSI_COLORS.get(script_name, "")
        assert proc.stdout is not None
        for raw_line in iter(proc.stdout.readline, ""):
            if not raw_line:
                break
            line = raw_line.rstrip("\r\n")

            # Check for structured RESULT_SUMMARY from ADAS ECU
            summary_match = summary_pattern.search(line)
            if summary_match:
                with result_lock:
                    if not result_holder:
                        result_holder.append(
                            SimulationResult(
                                vehicle_status=summary_match.group(1).strip(),
                                initial_speed=float(summary_match.group(2)),
                                distance_before_braking=float(summary_match.group(3)),
                                distance_maintained=float(summary_match.group(4)),
                            )
                        )
                        stream_gate.set()
                break

            # Once terminal result is latched, suppress any further continuous stream lines
            if stream_gate.is_set():
                continue

            aeb_match = aeb_trigger_pattern.search(line)
            if aeb_match and state_tracker["aeb_seen"] == 0.0:
                state_tracker["distance_before_braking"] = float(aeb_match.group(1))
                state_tracker["aeb_seen"] = 1.0

            dist_match = radar_dist_pattern.search(line)
            if dist_match:
                state_tracker["latest_distance"] = float(dist_match.group(1))

            sys.stdout.write(f"{color}{line}{ANSI_RESET}\n")
            sys.stdout.flush()

            # Fallback detection if sensor_ecu logs impact directly before ADAS summary
            if "SIMULATED IMPACT: Obstacle distance reached 0.00 m!" in line:
                with result_lock:
                    if not result_holder:
                        result_holder.append(
                            SimulationResult(
                                vehicle_status="Collided",
                                initial_speed=state_tracker["initial_speed"],
                                distance_before_braking=state_tracker["distance_before_braking"],
                                distance_maintained=0.0,
                            )
                        )
                        stream_gate.set()
                break

    script_args: Dict[str, List[str]] = {
        "powertrain_ecu.py": ["--speed", str(initial_speed)],
        "adas_ecu.py": [],
        "sensor_ecu.py": ["--distance", str(initial_distance)],
    }

    try:
        print(
            f"\n{ANSI_BOLD}>>> Starting SIL AEB Run | "
            f"Initial Distance: {initial_distance:.2f} m | "
            f"Initial Vehicle Speed: {initial_speed:.2f} m/s ({initial_speed * 3.6:.1f} km/h){ANSI_RESET}\n"
        )
        for script_name in ECU_SCRIPTS:
            script_path = project_dir / script_name
            cmd = [python_exe, "-u", str(script_path)] + script_args.get(script_name, [])
            proc = subprocess.Popen(
                cmd,
                cwd=str(project_dir),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            processes.append(proc)

            reader_thread = threading.Thread(
                target=_stream_output,
                args=(proc, script_name),
                daemon=True,
            )
            reader_thread.start()
            time.sleep(0.08)

        while not stream_gate.is_set():
            for proc in processes:
                if proc.poll() is not None and not stream_gate.is_set():
                    stream_gate.set()
                    break
            stream_gate.wait(timeout=0.05)

    except KeyboardInterrupt:
        stream_gate.set()
        print(f"\n{ANSI_RED}Simulation run interrupted by user.{ANSI_RESET}")
    finally:
        stream_gate.set()
        for proc in processes:
            if proc.poll() is None:
                try:
                    proc.send_signal(signal.SIGINT)
                except Exception:
                    proc.terminate()

        deadline = time.monotonic() + 2.0
        for proc in processes:
            remaining = max(0.1, deadline - time.monotonic())
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                proc.kill()

    return result_holder[0] if result_holder else None


def prompt_custom_scenario(project_dir: Path) -> Tuple[float, float]:
    """Prompt for custom distance (X) and vehicle speed (Y) and automatically update the ECU files.

    Does not open external terminal or editor windows; reads input directly from
    the console, writes the updated values into `sensor_ecu.py` and `powertrain_ecu.py`,
    and returns them for immediate execution.

    Args:
        project_dir: Directory containing `sensor_ecu.py` and `powertrain_ecu.py`.

    Returns:
        Tuple of ``(initial_distance, initial_speed)``.
    """
    current_dist, current_speed = read_starting_variables_from_files(project_dir)

    print("\n" + "-" * 68)
    print(f"{ANSI_BOLD}TEST DIFFERENT SCENARIOS (Custom Parameters){ANSI_RESET}")
    print("-" * 68)

    while True:
        raw_dist = input(
            f"  Enter initial distance X (in meters) [current: {current_dist:.1f} m]: "
        ).strip()
        if not raw_dist:
            new_dist = current_dist
            break
        try:
            new_dist = float(raw_dist)
            if new_dist < 0.0:
                print("  Please enter a non-negative distance in meters.")
                continue
            break
        except ValueError:
            print("  Invalid input. Please enter a numeric distance (e.g., 150 or 20).")

    while True:
        raw_speed = input(
            f"  Enter initial vehicle speed Y (in m/s) [current: {current_speed:.1f} m/s]: "
        ).strip()
        if not raw_speed:
            new_speed = current_speed
            break
        try:
            new_speed = float(raw_speed)
            if new_speed < 0.0:
                print("  Please enter a non-negative speed in m/s.")
                continue
            break
        except ValueError:
            print("  Invalid input. Please enter a numeric speed (e.g., 25 or 35).")

    write_starting_variables_to_files(project_dir, new_dist, new_speed)
    print(
        f"  -> Automatically updated sensor_ecu.py (INITIAL_DISTANCE = {new_dist:.1f} m) "
        f"and powertrain_ecu.py (INITIAL_VEHICLE_SPEED = {new_speed:.1f} m/s)."
    )
    return new_dist, new_speed


def interactive_menu_loop(project_dir: Path, python_exe: str) -> None:
    """Run the interactive menu loop until the user selects Option 3 (Exit).

    Args:
        project_dir: Root directory containing the ECU scripts.
        python_exe: Path to the Python interpreter.
    """
    while True:
        print("\n" + "=" * 68)
        print(f"{ANSI_BOLD}         SIL AEB SIMULATION MENU{ANSI_RESET}")
        print("=" * 68)
        print("  1. Automated Test Run (Default: Distance = 200.0 m, Speed = 25.0 m/s)")
        print("  2. Test different scenarios (Configure custom 'X' distance & 'Y' speed)")
        print("  3. Exit")
        print("-" * 68)

        try:
            choice = input("Enter your choice (1 / 2 / 3): ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting SIL AEB Launcher.")
            break

        if choice == "1":
            write_starting_variables_to_files(
                project_dir,
                INITIAL_DISTANCE_M,
                INITIAL_VEHICLE_SPEED_MPS,
            )
            result = run_simulation_once(
                project_dir=project_dir,
                python_exe=python_exe,
                initial_distance=INITIAL_DISTANCE_M,
                initial_speed=INITIAL_VEHICLE_SPEED_MPS,
            )
            if result is not None:
                print_simulation_result(result)

        elif choice == "2":
            try:
                custom_dist, custom_speed = prompt_custom_scenario(project_dir)
            except (EOFError, KeyboardInterrupt):
                print("\nCancelled scenario configuration; returning to main menu.")
                continue

            result = run_simulation_once(
                project_dir=project_dir,
                python_exe=python_exe,
                initial_distance=custom_dist,
                initial_speed=custom_speed,
            )
            if result is not None:
                print_simulation_result(result)

        elif choice == "3":
            print("Exiting SIL AEB Simulation. Goodbye!")
            break

        else:
            print("Invalid option. Please enter 1, 2, or 3.")


def launch_in_separate_terminal_windows(project_dir: Path, python_exe: str) -> List[subprocess.Popen]:
    """Launch each ECU script in a separate native OS terminal window.

    Args:
        project_dir: Directory containing the ECU scripts.
        python_exe: Path to the Python interpreter.

    Returns:
        List of launched :class:`subprocess.Popen` handles.
    """
    system = platform.system()
    processes: List[subprocess.Popen] = []

    for script_name in ECU_SCRIPTS:
        script_path = project_dir / script_name
        if system == "Darwin":
            cmd_str = f"cd {shlex.quote(str(project_dir))} && {shlex.quote(python_exe)} {shlex.quote(str(script_path))}"
            apple_script = f'tell application "Terminal" to do script "{cmd_str}"'
            proc = subprocess.Popen(["osascript", "-e", apple_script])
            processes.append(proc)
        elif system == "Windows":
            creation_flags = getattr(subprocess, "CREATE_NEW_CONSOLE", 0)
            proc = subprocess.Popen(
                [python_exe, str(script_path)],
                cwd=str(project_dir),
                creationflags=creation_flags,
            )
            processes.append(proc)
        else:
            proc = subprocess.Popen(
                ["x-terminal-emulator", "-e", python_exe, str(script_path)],
                cwd=str(project_dir),
            )
            processes.append(proc)

        logger.info("Launched %s in a separate terminal window.", script_name)
        time.sleep(0.15)

    return processes


def parse_args() -> argparse.Namespace:
    """Parse command-line options for the SIL AEB launcher."""
    parser = argparse.ArgumentParser(
        description="Launch the 3-ECU SIL AEB simulation over a virtual UDP multicast CAN bus."
    )
    parser.add_argument(
        "--separate-terminals",
        action="store_true",
        help="Open each ECU script in a separate OS terminal window.",
    )
    parser.add_argument(
        "--auto-stop",
        action="store_true",
        help="Run a single automated test non-interactively and exit after displaying the result.",
    )
    return parser.parse_args()


def main() -> None:
    """Main entry point for launching the three-ECU SIL simulation."""
    args = parse_args()
    project_dir = Path(__file__).resolve().parent
    python_exe = sys.executable

    if args.separate_terminals:
        launch_in_separate_terminal_windows(project_dir, python_exe)
    elif args.auto_stop:
        result = run_simulation_once(
            project_dir=project_dir,
            python_exe=python_exe,
            initial_distance=INITIAL_DISTANCE_M,
            initial_speed=INITIAL_VEHICLE_SPEED_MPS,
        )
        if result is not None:
            print_simulation_result(result)
    else:
        interactive_menu_loop(project_dir, python_exe)


if __name__ == "__main__":
    main()
