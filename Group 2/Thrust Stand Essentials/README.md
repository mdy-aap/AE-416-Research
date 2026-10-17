# Thrust Stand Essentials

Hardware firmware, calibration utilities, and real-time data acquisition (DAQ) software for the propeller static test stand.

---

## Directory Overview

| File / Folder | Type | Description |
| :--- | :--- | :--- |
| `gui_logger.py` | Python Script | Real-time Tkinter/Matplotlib telemetry GUI & data logger with AeroSandbox/BEMT integration. |
| `test_com.py` | Python Script | Serial port communication and packet verification script. |
| `rig_daq_softwarepwmcontrol/` | Arduino Sketch | Primary ESP32-S3 DAQ firmware with software PWM motor throttle control, dual load cell, optical RPM, and current sensing. |
| `rig_daq.ino` | Arduino Sketch | Standalone dual-HX711 and RPM pulse counting firmware. |
| `loadcell_test/` | Arduino Sketch | Isolated test bench sketch for HX711 and load cell diagnostics. |
| `rnt_calibration/` | Arduino Sketch | Deadweight calibration sketch for 1kg and 5kg load cells (calculating counts per gram). |

---

## Hardware Configuration (ESP32-S3)

- **Thrust Load Cell (5kg):** DT &rarr; GPIO 4, SCK &rarr; GPIO 5
- **Torque Load Cell #1 (1kg):** DT &rarr; GPIO 16, SCK &rarr; GPIO 17
- **Torque Load Cell #2 (1kg):** DT &rarr; GPIO 6, SCK &rarr; GPIO 7
- **Optical RPM Sensor (TCRT5000):** DO &rarr; GPIO 19
- **Current Sensor (ACS758 50A):** OUT &rarr; GPIO 9 (ADC1_CH8)
- **ESC PWM Throttle Output:** Signal &rarr; GPIO 18
- **HX711 3.3V Power Rail:** GPIO 3 / GPIO 8
- **Software 0V Ground:** GPIO 46 / GPIO 20
- **Baud Rate:** `115200 baud`

---

## Quick Start

1. **Calibrate Load Cells:**
   - Upload `rnt_calibration/rnt_calibration.ino` via Arduino IDE to determine exact calibration factors for your load cells using known weights.
2. **Flash Main Firmware:**
   - Open `rig_daq_softwarepwmcontrol/rig_daq_softwarepwmcontrol.ino` in Arduino IDE.
   - Select your ESP32-S3 board and flash the firmware.
3. **Run Telemetry GUI:**
   - Activate the virtual environment:
     ```cmd
     uav-env\Scripts\activate
     ```
   - Launch the logger:
     ```bash
     python gui_logger.py
     ```
   - Select the active COM port, connect, tare sensors, and begin automated or manual data logging.