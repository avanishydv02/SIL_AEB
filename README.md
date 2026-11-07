# SIL AEB: In-Vehicle CAN Network Simulation

A Software-in-the-Loop (SIL) simulation of a distributed in-vehicle network for an **Autonomous Emergency Braking (AEB)** system built with Python and `python-can`.

Three independent Electronic Control Units (ECUs) execute in separate OS processes and communicate over a virtual CAN bus (`udp_multicast` over the local loopback interface `127.0.0.1`), running cross-platform without kernel-level CAN drivers.

---

## 1. System Architecture

| Node | Script | Role | Transmit ID | Receive IDs | Rate |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Sensor ECU** | `sensor_ecu.py` | Simulates Radar/Lidar obstacle distance & closing speed | `0x100` (`RADAR_DATA`) | `0x200` (`VEHICLE_SPEED`) | 20 Hz (`0.05 s`) |
| **Powertrain ECU** | `powertrain_ecu.py` | Simulates vehicle speed sensor & emergency brake actuator | `0x200` (`VEHICLE_SPEED`) | `0x300` (`AEB_COMMAND`) | 20 Hz (`0.05 s`) |
| **ADAS ECU** | `adas_ecu.py` | Central safety controller; computes TTC & triggers AEB | `0x300` (`AEB_COMMAND`) | `0x100`, `0x200` | Event-triggered |

---

## 2. Interactive Main Launcher (`main_launcher.py`)

Run the interactive launcher:
```bash
.venv/bin/python main_launcher.py
```

You will be presented with an interactive menu loop:
1. **Automated test run**: Runs the default scenario (`Initial Distance = 200.0 m`, `Initial Speed = 25.0 m/s`).
2. **Test different scenarios**: Opens `sensor_ecu.py` and `powertrain_ecu.py` and allows you to configure custom starting values for initial distance `X` (meters) and vehicle speed `Y` (m/s) before running the simulation.
3. **Exit**: Cleanly exits the program.

As soon as the vehicle either **stops completely** or **collides**, the continuous CAN stream stops immediately and displays the **Final Simulation Result**:
- **Vehicle status**: `Stopped safely` or `Collided`
- **Initial speed of vehicle**: Speed before engaging brakes
- **Distance to vehicle before engaging brakes**: Obstacle distance when AEB triggered
- **Distance maintained**: Final inter-vehicle gap after the result
