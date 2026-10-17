import os
import sys
import site

# Automatically link local virtual environment packages if running outside uav-env
_venv_site = os.path.join(os.path.dirname(os.path.abspath(__file__)), "uav-env", "Lib", "site-packages")
if os.path.isdir(_venv_site):
    site.addsitedir(_venv_site)

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
import serial.tools.list_ports
import csv
import time
from datetime import datetime
import pandas as pd
import threading

try:
    from run_bemt_solver import parse_pe0_file, StaticBEMTQSolver
    BEMT_SOLVER_AVAILABLE = True
except ImportError:
    BEMT_SOLVER_AVAILABLE = False

# ------------------ AEROSANDBOX INTEGRATION SETUP ------------------
try:
    import aerosandbox as asb
    import aerosandbox.numpy as np
    AEROSANDBOX_AVAILABLE = True
except ImportError:
    import numpy as np
    AEROSANDBOX_AVAILABLE = False

# Propeller Physical Geometry Configuration (Default: 6.04" APC 6x4.5E from dataset)
DEFAULT_PROP_DIAMETER_INCHES = 6.04
PROP_DIAMETER_METERS = DEFAULT_PROP_DIAMETER_INCHES * 0.0254  # 0.1534 m
RHO_AIR = 1.225               # Standard atmospheric air density (kg/m^3)
# Propeller Rigidity & Aeroelastic Washout Presets (Module D)
RIGIDITY_PRESETS = {
    "Thin Nylon (APC-E)": 2.8,       # Standard APC Electric thin blades
    "Soft / Slow Flyer": 3.8,        # Wide, thin slow-flyer blades (SF/multirotor)
    "Stiff / Carbon Fiber": 0.5,     # High-modulus carbon fiber / reinforced
    "Rigid (Pure Theory 0°)": 0.0    # Unyielding 2D geometry (pure textbook BEMT)
}

# Motor Specs (For AeroSandbox First-Order Model Validation)
MOTOR_KV = 980                # Motor KV rating (RPM/V)
MOTOR_RESISTANCE = 0.06       # Internal resistance (Ohms)
MOTOR_NO_LOAD_CURRENT = 0.5   # No-load current (Amps)
# -------------------------------------------------------------------

DEFAULT_PORT = 'COM8'
BAUD_RATE = 115200
CSV_FILE_NAME = "uav_propeller_test_data.csv"
FIXED_VOLTAGE = 16.50 

class RCBenchmarkCloneApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RCBenchmark Characterization Suite - BEMT Validation Platform")
        self.root.geometry("1420x820")
        self.root.minsize(1240, 720)
        self.root.configure(bg="#111111")
        self.root.protocol("WM_DELETE_WINDOW", self.on_window_close)
        
        self.ser = None
        self.is_connected = False
        self.active_port = None
        self.tare_offset_thrust = 0.0
        self.tare_offset_torque = 0.0
        
        # Performance & Anti-Lag Optimization States
        self.last_slider_send = 0.0
        self.last_scan_time = 0.0
        self.last_plot_time = 0.0
        self.current_live_rpm = 0.0
        self.plot_dirty = True
        self.scan_thread = None
        self.discovered_ser = None
        self.discovered_port = None
        self._resize_job = None
        self._last_resize_dims = (0, 0)
        self._last_resize_time = 0.0
        self._daq_loop_job = None
        self.csv_file = None
        self.csv_writer = None
        
        # Closed-Loop Automation Variables
        self.in_auto_sweep = False
        self.sweep_targets = []
        self.current_step_idx = 0
        self.step_start_time = 0.0
        self.step_duration = 0.0
        self.sweep_current_pwm = 1000
        
        self.timestamps, self.thrust_data, self.torque_data = [], [], []
        self.rpm_data, self.current_data, self.power_data = [], [], []
        
        # Propeller and BEMT Validation States
        self.prop_filename = "6x45E-PERF.PE0"
        self.prop_diameter_in = DEFAULT_PROP_DIAMETER_INCHES
        self.prop_diameter_m = PROP_DIAMETER_METERS
        self.ct_proxy = 0.1305 # Calibrated 6x4.5E with Module C & D (Thin Nylon)
        self.cp_proxy = 0.0813
        self.current_rigidity_name = "Thin Nylon (APC-E)"
        self.cached_pe0_geom = None
        self.view_mode = "time"  # "time" (scrolling time strip-chart) or "bemt" (RPM validation)
        self.invert_torque = False
        self.rotor_direction_mode = "AUTO"  # "AUTO", "FORCE_CCW", "FORCE_CW"
        self.detected_direction = "IDLE"     # "IDLE", "CCW", "CW"
        self.bemt_history_rpm = []
        self.bemt_history_thrust = []
        self.bemt_history_torque = []
        self.bemt_point_counter = 0
        self.last_tq1_g = 0.0
        self.last_tq2_g = 0.0

        # Research Trial & Step Aggregation States
        self.trial_name = "Baseline"
        self.trial_number = 1
        self.auto_increment_trial = tk.BooleanVar(value=True)
        self.auto_export_excel = tk.BooleanVar(value=True)
        self.current_trial_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_results")
        os.makedirs(self.current_trial_dir, exist_ok=True)
        self.current_step_buffer = []      # Raw samples during active step
        self.trial_summary_rows = []       # Steady-state summary rows (BEMT vs Measured)
        self.current_trial_raw_log = []    # Raw 10Hz log of the active trial

        # Pre-cache default APC geometry if present
        if BEMT_SOLVER_AVAILABLE and os.path.exists(self.prop_filename):
            try:
                self.cached_pe0_geom = parse_pe0_file(self.prop_filename)
                r_m, hr_m, nb, rad_m, ch_m, tw_deg = self.cached_pe0_geom
                self.prop_diameter_m = r_m * 2.0
                self.prop_diameter_in = self.prop_diameter_m / 0.0254
            except Exception as e:
                print(f"[INIT] Geometry pre-cache skipped: {e}")

        self.init_csv()
        self.create_layout()
        self.root.after(100, self.update_daq_loop)

    def init_csv(self):
        try:
            with open(CSV_FILE_NAME, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Timestamp_ms", "Thrust_g", "Torque_Nm", "RPM", "Current_A", "Power_W"])
            if hasattr(self, 'csv_file') and self.csv_file and not self.csv_file.closed:
                try: self.csv_file.close()
                except Exception: pass
            self.csv_file = open(CSV_FILE_NAME, mode='a', newline='', buffering=1)
            self.csv_writer = csv.writer(self.csv_file)
        except Exception as e:
            print(f"CSV Init Error: {e}")

    def create_layout(self):
        # ------------------ LEFT SIDEBAR (DIGITAL KPI CARDS & ALARMS) ------------------
        self.sidebar = tk.Frame(self.root, width=310, bg="#161616", padx=10, pady=10)
        self.sidebar.pack(side=tk.LEFT, fill=tk.Y)
        self.sidebar.pack_propagate(False)
        
        # Header title
        hdr_frame = tk.Frame(self.sidebar, bg="#161616")
        hdr_frame.pack(fill=tk.X, pady=(0, 6))
        tk.Label(hdr_frame, text="⚡ DAQ TELEMETRY", font=("Arial", 12, "bold"), fg="#FF6600", bg="#161616").pack(side=tk.LEFT)
        self.lbl_daq_hz = tk.Label(hdr_frame, text="10 Hz", font=("Arial", 8), fg="#666666", bg="#161616")
        self.lbl_daq_hz.pack(side=tk.RIGHT, pady=(2, 0))

        # Hardware connection status card
        self.status_frame = tk.Frame(self.sidebar, bg="#2A1005", padx=8, pady=6, highlightbackground="#FF4400", highlightthickness=1)
        self.status_frame.pack(fill=tk.X, pady=(0, 8))
        
        self.status_lbl = tk.Label(self.status_frame, text="Hardware:\nDisconnected\n(Scanning...)", 
                                   font=("Arial", 9, "bold"), fg="#FF5500", bg="#2A1005", justify=tk.LEFT)
        self.status_lbl.pack(anchor="w")

        # Emergency Motor Stop Button (Permanent prominent crimson red)
        self.btn_estop = tk.Button(self.sidebar, text="STOP MOTOR (E-STOP)", font=("Arial", 10, "bold"),
                                   bg="#B30000", fg="#FFFFFF", activebackground="#FF1111", activeforeground="#FFFFFF",
                                   relief=tk.RAISED, bd=2, padx=8, pady=6, command=self.emergency_motor_stop)
        self.btn_estop.pack(fill=tk.X, pady=(0, 8))

        # Bottom Action Buttons (Packed side=BOTTOM first so they remain visible at the bottom)
        self.btn_tare = tk.Button(self.sidebar, text="⚖ Zero / Tare Rig", font=("Arial", 9, "bold"), bg="#282828", fg="#FFFFFF",
                                  activebackground="#383838", activeforeground="#00FFCC", relief=tk.FLAT, padx=8, pady=5,
                                  command=self.trigger_tare)
        self.btn_tare.pack(fill=tk.X, side=tk.BOTTOM, pady=(4, 0))

        self.btn_cal = tk.Button(self.sidebar, text="⚙ Calibrate Load Cells", font=("Arial", 9, "bold"), bg="#D95D00", fg="#FFFFFF",
                                 activebackground="#FF6600", activeforeground="#FFFFFF", relief=tk.FLAT, padx=8, pady=5,
                                 command=self.open_calibration_window)
        self.btn_cal.pack(fill=tk.X, side=tk.BOTTOM, pady=(0, 2))

        # --- DIGITAL KPI CARDS CONTAINER ---
        self.cards_frame = tk.Frame(self.sidebar, bg="#161616")
        self.cards_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 4))

        # 1. THRUST CARD
        self.card_thrust = tk.Frame(self.cards_frame, bg="#202020", highlightbackground="#333333", highlightthickness=1, padx=8, pady=4)
        self.card_thrust.pack(fill=tk.X, pady=2)
        
        c1_top = tk.Frame(self.card_thrust, bg="#202020")
        c1_top.pack(fill=tk.X)
        tk.Label(c1_top, text="THRUST", font=("Arial", 8, "bold"), fg="#888888", bg="#202020").pack(side=tk.LEFT)
        self.lbl_th_secondary = tk.Label(c1_top, text="0.00 N", font=("Arial", 8, "bold"), fg="#00B4D8", bg="#202020")
        self.lbl_th_secondary.pack(side=tk.RIGHT)

        c1_val = tk.Frame(self.card_thrust, bg="#202020")
        c1_val.pack(fill=tk.X, pady=(1, 0))
        self.lbl_th_val = tk.Label(c1_val, text="0.0", font=("Consolas", 16, "bold"), fg="#00FFCC", bg="#202020")
        self.lbl_th_val.pack(side=tk.LEFT)
        tk.Label(c1_val, text="g", font=("Arial", 9, "bold"), fg="#666666", bg="#202020").pack(side=tk.LEFT, padx=(3, 0), pady=(3, 0))

        # 2. AERODYNAMIC TORQUE CARD
        self.card_torque = tk.Frame(self.cards_frame, bg="#202020", highlightbackground="#333333", highlightthickness=1, padx=8, pady=4)
        self.card_torque.pack(fill=tk.X, pady=2)
        
        c2_top = tk.Frame(self.card_torque, bg="#202020")
        c2_top.pack(fill=tk.X)
        tk.Label(c2_top, text="AERO TORQUE", font=("Arial", 8, "bold"), fg="#888888", bg="#202020").pack(side=tk.LEFT)
        self.lbl_tq_secondary = tk.Label(c2_top, text="0.0 mN·m", font=("Arial", 8, "bold"), fg="#FF99CC", bg="#202020")
        self.lbl_tq_secondary.pack(side=tk.RIGHT)

        c2_val = tk.Frame(self.card_torque, bg="#202020")
        c2_val.pack(fill=tk.X, pady=(1, 0))
        self.lbl_tq_val = tk.Label(c2_val, text="0.0000", font=("Consolas", 14, "bold"), fg="#FF66CC", bg="#202020")
        self.lbl_tq_val.pack(side=tk.LEFT)
        tk.Label(c2_val, text="N·m", font=("Arial", 8, "bold"), fg="#666666", bg="#202020").pack(side=tk.LEFT, padx=(3, 0), pady=(3, 0))

        self.lbl_tq_cells = tk.Label(self.card_torque, text="T1: 0.0g  |  T2: 0.0g", font=("Consolas", 8), fg="#666666", bg="#202020")
        self.lbl_tq_cells.pack(anchor="w")

        # 3. MOTOR SPEED (RPM) CARD
        self.card_rpm = tk.Frame(self.cards_frame, bg="#202020", highlightbackground="#333333", highlightthickness=1, padx=8, pady=4)
        self.card_rpm.pack(fill=tk.X, pady=2)

        c3_top = tk.Frame(self.card_rpm, bg="#202020")
        c3_top.pack(fill=tk.X)
        tk.Label(c3_top, text="MOTOR SPEED", font=("Arial", 8, "bold"), fg="#888888", bg="#202020").pack(side=tk.LEFT)
        self.lbl_rpm_target = tk.Label(c3_top, text="Target: ---", font=("Arial", 8), fg="#777777", bg="#202020")
        self.lbl_rpm_target.pack(side=tk.RIGHT)

        c3_val = tk.Frame(self.card_rpm, bg="#202020")
        c3_val.pack(fill=tk.X, pady=(1, 0))
        self.lbl_rpm_val = tk.Label(c3_val, text="0", font=("Consolas", 16, "bold"), fg="#00FF66", bg="#202020")
        self.lbl_rpm_val.pack(side=tk.LEFT)
        tk.Label(c3_val, text="RPM", font=("Arial", 9, "bold"), fg="#666666", bg="#202020").pack(side=tk.LEFT, padx=(3, 0), pady=(3, 0))

        # 4. ELECTRICAL POWER CARD
        self.card_elec = tk.Frame(self.cards_frame, bg="#202020", highlightbackground="#333333", highlightthickness=1, padx=8, pady=4)
        self.card_elec.pack(fill=tk.X, pady=2)

        c4_top = tk.Frame(self.card_elec, bg="#202020")
        c4_top.pack(fill=tk.X)
        tk.Label(c4_top, text="ELECTRICAL POWER", font=("Arial", 8, "bold"), fg="#888888", bg="#202020").pack(side=tk.LEFT)
        self.lbl_elec_alert = tk.Label(c4_top, text="", font=("Arial", 8, "bold"), fg="#FF3300", bg="#202020")
        self.lbl_elec_alert.pack(side=tk.RIGHT)

        c4_val = tk.Frame(self.card_elec, bg="#202020")
        c4_val.pack(fill=tk.X, pady=(1, 0))
        self.lbl_power_val = tk.Label(c4_val, text="0.0", font=("Consolas", 15, "bold"), fg="#FFCC00", bg="#202020")
        self.lbl_power_val.pack(side=tk.LEFT)
        tk.Label(c4_val, text="W", font=("Arial", 9, "bold"), fg="#666666", bg="#202020").pack(side=tk.LEFT, padx=(3, 0), pady=(3, 0))

        self.lbl_vi_val = tk.Label(self.card_elec, text=f"{FIXED_VOLTAGE:.1f} V  •  0.00 A", font=("Consolas", 8), fg="#AAAAAA", bg="#202020")
        self.lbl_vi_val.pack(anchor="w")

        # 5. EFFICIENCY & SHAFT MECHANICAL POWER CARD
        self.card_eff = tk.Frame(self.cards_frame, bg="#202020", highlightbackground="#333333", highlightthickness=1, padx=8, pady=4)
        self.card_eff.pack(fill=tk.X, pady=2)

        c5_top = tk.Frame(self.card_eff, bg="#202020")
        c5_top.pack(fill=tk.X)
        tk.Label(c5_top, text="EFFICIENCY & SHAFT", font=("Arial", 8, "bold"), fg="#888888", bg="#202020").pack(side=tk.LEFT)
        self.lbl_mech_power = tk.Label(c5_top, text="Pmech: 0.0 W", font=("Arial", 8, "bold"), fg="#90E0EF", bg="#202020")
        self.lbl_mech_power.pack(side=tk.RIGHT)

        c5_bottom = tk.Frame(self.card_eff, bg="#202020")
        c5_bottom.pack(fill=tk.X, pady=(2, 0))
        self.lbl_aero_eff = tk.Label(c5_bottom, text="0.00 g/W", font=("Consolas", 9, "bold"), fg="#00E5FF", bg="#202020")
        self.lbl_aero_eff.pack(side=tk.LEFT)
        tk.Label(c5_bottom, text="•", fg="#444444", bg="#202020").pack(side=tk.LEFT, padx=6)
        self.lbl_motor_eff = tk.Label(c5_bottom, text="η: 0.0%", font=("Consolas", 9, "bold"), fg="#A7C957", bg="#202020")
        self.lbl_motor_eff.pack(side=tk.LEFT)

        # Backwards-compatibility hidden label
        self.metrics_lbl = tk.Label(self.sidebar, text=self.generate_metrics_string(0, 0, 0, 0, 0))

        # ------------------ RIGHT PANELS ------------------
        self.right_container = tk.Frame(self.root, bg="#111111")
        self.right_container.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # --- TOP PANEL: Actuator Control Deck (2-Row Responsive Layout) ---
        self.control_panel = tk.LabelFrame(self.right_container, text=" Actuator Control Deck ", font=("Arial", 11, "bold"),
                                           fg="#FF6600", bg="#1A1A1A", padx=12, pady=8, relief=tk.GROOVE)
        self.control_panel.pack(side=tk.TOP, fill=tk.X, pady=(0, 8))

        # ROW 1: Manual ESC Throttle (Left) + Sweep Trigger Action (Right)
        deck_row1 = tk.Frame(self.control_panel, bg="#1A1A1A")
        deck_row1.pack(side=tk.TOP, fill=tk.X, pady=(0, 6))

        man_frame = tk.Frame(deck_row1, bg="#1A1A1A")
        man_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)

        man_hdr = tk.Frame(man_frame, bg="#1A1A1A")
        man_hdr.pack(fill=tk.X)
        tk.Label(man_hdr, text="Manual ESC PWM (µs):", font=("Arial", 9, "bold"), fg="#FFFFFF", bg="#1A1A1A").pack(side=tk.LEFT)

        self.ent_pwm = tk.Entry(man_hdr, width=5, bg="#333333", fg="#00FFCC", font=("Arial", 9, "bold"), justify="center", insertbackground="white", relief=tk.FLAT)
        self.ent_pwm.insert(0, "1000")
        self.ent_pwm.pack(side=tk.LEFT, padx=6)
        self.ent_pwm.bind("<Return>", self.on_pwm_entry_submit)
        self.ent_pwm.bind("<FocusOut>", self.on_pwm_entry_submit)

        # Quick Presets & Nudge Buttons placed inline with header
        tk.Button(man_hdr, text="0%", font=("Arial", 8, "bold"), bg="#333333", fg="#FF7777", relief=tk.FLAT, padx=5, pady=1,
                  command=lambda: self.set_pwm_direct(1000)).pack(side=tk.LEFT, padx=2)
        tk.Button(man_hdr, text="25%", font=("Arial", 8), bg="#262626", fg="#FFFFFF", relief=tk.FLAT, padx=5, pady=1,
                  command=lambda: self.set_pwm_direct(1250)).pack(side=tk.LEFT, padx=2)
        tk.Button(man_hdr, text="50%", font=("Arial", 8), bg="#262626", fg="#FFFFFF", relief=tk.FLAT, padx=5, pady=1,
                  command=lambda: self.set_pwm_direct(1500)).pack(side=tk.LEFT, padx=2)
        tk.Button(man_hdr, text="75%", font=("Arial", 8), bg="#262626", fg="#FFFFFF", relief=tk.FLAT, padx=5, pady=1,
                  command=lambda: self.set_pwm_direct(1750)).pack(side=tk.LEFT, padx=2)
        tk.Button(man_hdr, text="-10", font=("Arial", 8), bg="#222222", fg="#00FFCC", relief=tk.FLAT, padx=4, pady=1,
                  command=lambda: self.step_pwm(-10)).pack(side=tk.LEFT, padx=(5, 1))
        tk.Button(man_hdr, text="+10", font=("Arial", 8), bg="#222222", fg="#00FFCC", relief=tk.FLAT, padx=4, pady=1,
                  command=lambda: self.step_pwm(10)).pack(side=tk.LEFT, padx=1)

        self.pwm_slider = tk.Scale(man_frame, from_=1000, to=2000, orient=tk.HORIZONTAL, length=380,
                                   bg="#1A1A1A", fg="#FFFFFF", highlightthickness=0, troughcolor="#333333",
                                   activebackground="#FF6600", command=self.on_slider_moved)
        self.pwm_slider.pack(anchor="w", fill=tk.X, pady=(2, 0))

        # Sweep Trigger Action Box (Right side of Row 1)
        sweep_action_box = tk.Frame(deck_row1, bg="#1A1A1A")
        sweep_action_box.pack(side=tk.RIGHT, padx=(15, 0))

        self.sweep_btn = tk.Button(sweep_action_box, text="Start Auto Sweep", font=("Arial", 11, "bold"),
                                   bg="#006633", fg="#FFFFFF", relief=tk.FLAT, padx=22, pady=6, command=self.toggle_rpm_sweep)
        self.sweep_btn.pack(side=tk.TOP, fill=tk.X)

        self.lbl_sweep_status = tk.Label(sweep_action_box, text="Sweep: Idle", font=("Arial", 8),
                                         fg="#888888", bg="#1A1A1A")
        self.lbl_sweep_status.pack(side=tk.TOP, pady=(2, 0))

        # Divider between rows
        sep = tk.Frame(self.control_panel, height=1, bg="#282828")
        sep.pack(fill=tk.X, pady=(4, 6))

        # ROW 2: Closed-Loop Sweep Parameters (Left) + Research Trial Setup (Right)
        deck_row2 = tk.Frame(self.control_panel, bg="#1A1A1A")
        deck_row2.pack(side=tk.TOP, fill=tk.X)

        # Automatic RPM Closed-Loop Controls Configuration Box
        auto_frame = tk.Frame(deck_row2, bg="#1A1A1A")
        auto_frame.pack(side=tk.LEFT, padx=(0, 20))

        tk.Label(auto_frame, text="Automatic Closed-Loop RPM Sweep Setup", font=("Arial", 9, "bold"), fg="#FFCC00", bg="#1A1A1A").grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 2))

        tk.Label(auto_frame, text="Start RPM", fg="#BBBBBB", bg="#1A1A1A", font=("Arial", 8)).grid(row=1, column=0, padx=4, sticky="w")
        self.ent_start = tk.Entry(auto_frame, width=7, bg="#333333", fg="#FFFFFF", insertbackground="white", relief=tk.FLAT, font=("Arial", 9))
        self.ent_start.insert(0, "2000")
        self.ent_start.grid(row=2, column=0, padx=4, pady=1)

        tk.Label(auto_frame, text="End RPM", fg="#BBBBBB", bg="#1A1A1A", font=("Arial", 8)).grid(row=1, column=1, padx=4, sticky="w")
        self.ent_end = tk.Entry(auto_frame, width=7, bg="#333333", fg="#FFFFFF", insertbackground="white", relief=tk.FLAT, font=("Arial", 9))
        self.ent_end.insert(0, "5000")
        self.ent_end.grid(row=2, column=1, padx=4, pady=1)

        tk.Label(auto_frame, text="Step RPM", fg="#BBBBBB", bg="#1A1A1A", font=("Arial", 8)).grid(row=1, column=2, padx=4, sticky="w")
        self.ent_step = tk.Entry(auto_frame, width=7, bg="#333333", fg="#FFFFFF", insertbackground="white", relief=tk.FLAT, font=("Arial", 9))
        self.ent_step.insert(0, "500")
        self.ent_step.grid(row=2, column=2, padx=4, pady=1)

        tk.Label(auto_frame, text="Step Duration (s)", fg="#BBBBBB", bg="#1A1A1A", font=("Arial", 8)).grid(row=1, column=3, padx=4, sticky="w")
        self.ent_dur = tk.Entry(auto_frame, width=9, bg="#333333", fg="#FFFFFF", insertbackground="white", relief=tk.FLAT, font=("Arial", 9))
        self.ent_dur.insert(0, "5.0")
        self.ent_dur.grid(row=2, column=3, padx=4, pady=1)

        # Research Trial & Session Configuration Box
        trial_frame = tk.Frame(deck_row2, bg="#1A1A1A", padx=4)
        trial_frame.pack(side=tk.LEFT, fill=tk.Y)

        tk.Label(trial_frame, text="Research Trial Setup", font=("Arial", 9, "bold"), fg="#00FFCC", bg="#1A1A1A").grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 2))

        tk.Label(trial_frame, text="Trial Tag / Name", fg="#BBBBBB", bg="#1A1A1A", font=("Arial", 8)).grid(row=1, column=0, padx=4, sticky="w")
        self.ent_trial_tag = tk.Entry(trial_frame, width=14, bg="#333333", fg="#FFFFFF", insertbackground="white", relief=tk.FLAT, font=("Arial", 9))
        self.ent_trial_tag.insert(0, "Baseline")
        self.ent_trial_tag.grid(row=2, column=0, padx=4, pady=1)

        tk.Label(trial_frame, text="Trial #", fg="#BBBBBB", bg="#1A1A1A", font=("Arial", 8)).grid(row=1, column=1, padx=4, sticky="w")
        self.ent_trial_num = tk.Spinbox(trial_frame, from_=1, to=999, width=4, bg="#333333", fg="#FFFFFF", relief=tk.FLAT, font=("Arial", 9))
        self.ent_trial_num.delete(0, tk.END)
        self.ent_trial_num.insert(0, "1")
        self.ent_trial_num.grid(row=2, column=1, padx=4, pady=1)

        chk_opts = tk.Frame(trial_frame, bg="#1A1A1A")
        chk_opts.grid(row=1, column=2, rowspan=2, padx=6, sticky="w")
        self.chk_auto_inc = tk.Checkbutton(chk_opts, text="Auto +1", variable=self.auto_increment_trial,
                                           bg="#1A1A1A", fg="#CCCCCC", selectcolor="#262626", activebackground="#1A1A1A",
                                           activeforeground="#00FFCC", font=("Arial", 8))
        self.chk_auto_inc.pack(anchor="w")
        self.chk_auto_exp = tk.Checkbutton(chk_opts, text="Auto .xlsx", variable=self.auto_export_excel,
                                           bg="#1A1A1A", fg="#CCCCCC", selectcolor="#262626", activebackground="#1A1A1A",
                                           activeforeground="#00FFCC", font=("Arial", 8))
        self.chk_auto_exp.pack(anchor="w")

        # --- VIEW NAVIGATION & DIAGNOSTICS BAR ---
        self.toolbar_frame = tk.Frame(self.right_container, bg="#1A1A1A", padx=8, pady=4)
        self.toolbar_frame.pack(side=tk.TOP, fill=tk.X, pady=(0, 2))

        # View Mode Buttons (Left)
        self.btn_mode_time = tk.Button(self.toolbar_frame, text="⏱ Live Strip-Chart (Time)", font=("Arial", 9, "bold"),
                                       bg="#006633", fg="#FFFFFF", relief=tk.FLAT, padx=10, pady=4,
                                       command=lambda: self.set_view_mode("time"))
        self.btn_mode_time.pack(side=tk.LEFT, padx=3)

        self.btn_mode_bemt = tk.Button(self.toolbar_frame, text="📊 Aero BEMT (RPM)", font=("Arial", 9, "bold"),
                                       bg="#333333", fg="#AAAAAA", relief=tk.FLAT, padx=10, pady=4,
                                       command=lambda: self.set_view_mode("bemt"))
        self.btn_mode_bemt.pack(side=tk.LEFT, padx=3)

        self.btn_mode_table = tk.Button(self.toolbar_frame, text="📋 Research Table (BEMT)", font=("Arial", 9, "bold"),
                                        bg="#333333", fg="#AAAAAA", relief=tk.FLAT, padx=10, pady=4,
                                        command=lambda: self.set_view_mode("table"))
        self.btn_mode_table.pack(side=tk.LEFT, padx=3)

        # Hardware & Points Diagnostics (Right)
        self.btn_clear_bemt = tk.Button(self.toolbar_frame, text="Reset BEMT Points", font=("Arial", 8),
                                        bg="#262626", fg="#FFFFFF", relief=tk.FLAT, padx=8, pady=3,
                                        command=self.clear_bemt_history)
        self.btn_clear_bemt.pack(side=tk.RIGHT, padx=4)

        # Smart Rotor Direction Pill (Auto CW / CCW Sensing with manual click override)
        self.lbl_rotor_dir = tk.Label(self.toolbar_frame, text="🔄 Rotor: Auto (CCW)", font=("Arial", 8, "bold"),
                                      bg="#003322", fg="#00FFCC", relief=tk.RIDGE, bd=1, padx=8, pady=3, cursor="hand2")
        self.lbl_rotor_dir.pack(side=tk.RIGHT, padx=6)
        self.lbl_rotor_dir.bind("<Button-1>", self.cycle_rotor_direction_mode)

        # --- AERODYNAMIC CONFIGURATION SUB-BAR ---
        self.aero_bar = tk.Frame(self.right_container, bg="#161616", padx=8, pady=3)
        self.aero_bar.pack(side=tk.TOP, fill=tk.X, pady=(0, 6))

        tk.Label(self.aero_bar, text="Aerodynamic Model:", font=("Arial", 9, "bold"), fg="#62B6CB", bg="#161616").pack(side=tk.LEFT, padx=(2, 6))

        self.btn_load_pe0 = tk.Button(self.aero_bar, text="📁 Choose Prop (.PE0)", font=("Arial", 8, "bold"),
                                      bg="#1B4965", fg="#FFFFFF", relief=tk.FLAT, padx=8, pady=2,
                                      command=self.choose_pe0_file)
        self.btn_load_pe0.pack(side=tk.LEFT, padx=4)

        self.lbl_loaded_prop = tk.Label(self.aero_bar, text=f"Prop: {self.prop_filename}",
                                        font=("Arial", 8, "italic"), fg="#A0E0F0", bg="#161616")
        self.lbl_loaded_prop.pack(side=tk.LEFT, padx=6)

        tk.Label(self.aero_bar, text="|", fg="#444444", bg="#161616").pack(side=tk.LEFT, padx=4)

        tk.Label(self.aero_bar, text="Prop Dia (in):", font=("Arial", 9, "bold"), fg="#FFCC00", bg="#161616").pack(side=tk.LEFT, padx=(4, 2))
        self.ent_prop_dia = tk.Entry(self.aero_bar, width=5, bg="#333333", fg="#FFFFFF", insertbackground="white", relief=tk.FLAT, font=("Arial", 9))
        self.ent_prop_dia.insert(0, f"{self.prop_diameter_in:.2f}")
        self.ent_prop_dia.pack(side=tk.LEFT, padx=2)
        self.ent_prop_dia.bind("<Return>", self.update_prop_diameter)
        self.ent_prop_dia.bind("<FocusOut>", self.update_prop_diameter)

        tk.Label(self.aero_bar, text="|", fg="#444444", bg="#161616").pack(side=tk.LEFT, padx=4)

        tk.Label(self.aero_bar, text="Flex Rigidity:", font=("Arial", 9, "bold"), fg="#00FFCC", bg="#161616").pack(side=tk.LEFT, padx=(4, 2))
        self.cbo_rigidity = ttk.Combobox(self.aero_bar, values=list(RIGIDITY_PRESETS.keys()), width=18, state="readonly")
        self.cbo_rigidity.set(self.current_rigidity_name)
        self.cbo_rigidity.pack(side=tk.LEFT, padx=2)
        self.cbo_rigidity.bind("<<ComboboxSelected>>", self.on_rigidity_changed)

        tk.Label(self.aero_bar, text="Thrust Min Span: 100g", font=("Arial", 8), fg="#777777", bg="#161616").pack(side=tk.RIGHT, padx=6)

        # --- BOTTOM PANEL: Graphical Data Windows ---
        self.plot_frame = tk.Frame(self.right_container, bg="#121212")
        self.plot_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 5), facecolor='#121212')
        self.fig.tight_layout(pad=2.2)
        self.init_plots()
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().configure(bg='#121212')
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.get_tk_widget().bind('<Configure>', self.on_canvas_configure)

        # Research Table Container (Created ready in background)
        self.create_research_table_widget()

    def on_canvas_configure(self, event):
        dims = (event.width, event.height)
        if dims == self._last_resize_dims or event.width <= 10 or event.height <= 10:
            return

        now = time.time()
        if self._resize_job is not None:
            try:
                self.root.after_cancel(self._resize_job)
            except Exception:
                pass
            self._resize_job = None

        if now - self._last_resize_time >= 0.08:
            self._apply_canvas_resize(event.width, event.height)
        else:
            self._resize_job = self.root.after(60, self._apply_canvas_resize, event.width, event.height)

    def _apply_canvas_resize(self, width, height):
        self._resize_job = None
        self._last_resize_dims = (width, height)
        self._last_resize_time = time.time()
        try:
            self.canvas.resize(type('Event', (), {'width': width, 'height': height})())
        except Exception as e:
            print(f"[RESIZE ERROR] {e}")

    def set_view_mode(self, mode):
        self.view_mode = mode
        if mode == "time":
            self.btn_mode_time.configure(bg="#006633", fg="#FFFFFF")
            self.btn_mode_bemt.configure(bg="#333333", fg="#AAAAAA")
            self.btn_mode_table.configure(bg="#333333", fg="#AAAAAA")
            if hasattr(self, 'table_container'):
                self.table_container.pack_forget()
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            self.update_plot_mode_styling()
            self.plot_dirty = True
            self.render_plots()
        elif mode == "bemt":
            self.btn_mode_bemt.configure(bg="#006633", fg="#FFFFFF")
            self.btn_mode_time.configure(bg="#333333", fg="#AAAAAA")
            self.btn_mode_table.configure(bg="#333333", fg="#AAAAAA")
            if hasattr(self, 'table_container'):
                self.table_container.pack_forget()
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            self.update_plot_mode_styling()
            self.plot_dirty = True
            self.render_plots()
        elif mode == "table":
            self.btn_mode_table.configure(bg="#006633", fg="#FFFFFF")
            self.btn_mode_time.configure(bg="#333333", fg="#AAAAAA")
            self.btn_mode_bemt.configure(bg="#333333", fg="#AAAAAA")
            self.canvas.get_tk_widget().pack_forget()
            if hasattr(self, 'table_container'):
                self.table_container.pack(fill=tk.BOTH, expand=True)

    def create_research_table_widget(self):
        self.table_container = tk.Frame(self.plot_frame, bg="#121212")

        # Top Action Bar above Table
        tbl_bar = tk.Frame(self.table_container, bg="#181818", padx=8, pady=6)
        tbl_bar.pack(side=tk.TOP, fill=tk.X, pady=(0, 4))

        tk.Button(tbl_bar, text="📥 Export Trial to Excel (.xlsx)", font=("Arial", 9, "bold"),
                  bg="#006633", fg="#FFFFFF", activebackground="#008844", activeforeground="#FFFFFF", relief=tk.FLAT, padx=12, pady=4,
                  command=self.export_current_trial_excel).pack(side=tk.LEFT, padx=4)

        tk.Button(tbl_bar, text="💾 Save CSV", font=("Arial", 9),
                  bg="#2E2E2E", fg="#FFFFFF", activebackground="#3E3E3E", activeforeground="#FFFFFF", relief=tk.FLAT, padx=10, pady=4,
                  command=self.export_current_trial_csv).pack(side=tk.LEFT, padx=4)

        tk.Button(tbl_bar, text="📸 Snapshot Point", font=("Arial", 9, "bold"),
                  bg="#1B4965", fg="#FFFFFF", activebackground="#24648A", activeforeground="#FFFFFF", relief=tk.FLAT, padx=10, pady=4,
                  command=self.snapshot_current_point).pack(side=tk.LEFT, padx=6)

        tk.Button(tbl_bar, text="🗑 Clear Table", font=("Arial", 8),
                  bg="#3D1A1A", fg="#FF8888", activebackground="#522222", activeforeground="#FFAAAA", relief=tk.FLAT, padx=8, pady=4,
                  command=self.clear_table_data).pack(side=tk.LEFT, padx=4)

        tk.Button(tbl_bar, text="📁 Open Results Folder", font=("Arial", 8),
                  bg="#222222", fg="#CCCCCC", activebackground="#333333", activeforeground="#FFFFFF", relief=tk.FLAT, padx=8, pady=4,
                  command=self.open_results_folder).pack(side=tk.LEFT, padx=6)

        self.lbl_table_status = tk.Label(tbl_bar, text="Ready. Run Auto Sweep or click Snapshot Point.",
                                         font=("Arial", 9), fg="#00FFCC", bg="#181818")
        self.lbl_table_status.pack(side=tk.RIGHT, padx=8)

        # Treeview + Scrollbars
        tree_frame = tk.Frame(self.table_container, bg="#121212")
        tree_frame.pack(fill=tk.BOTH, expand=True)

        columns = [
            ("step", "Step", 45),
            ("pwm", "PWM", 50),
            ("target_rpm", "Target RPM", 75),
            ("meas_rpm", "Actual RPM", 100),
            ("meas_th", "Meas Thrust (g)", 95),
            ("bemt_th", "BEMT Thrust (g)", 95),
            ("th_err", "Thrust Err %", 80),
            ("meas_tq", "Meas Torque (Nm)", 105),
            ("bemt_tq", "BEMT Torque (Nm)", 105),
            ("tq_err", "Torque Err %", 80),
            ("meas_mech", "Mech Power (W)", 95),
            ("bemt_mech", "BEMT Power (W)", 95),
            ("elec_pwr", "Elec Power (W)", 90),
            ("motor_eff", "Motor Eff %", 75),
            ("aero_eff", "g/W", 55),
            ("ct_ratio", "CT (Meas/BEMT)", 95),
            ("cp_ratio", "CP (Meas/BEMT)", 95)
        ]

        col_ids = [c[0] for c in columns]
        self.table_tree = ttk.Treeview(tree_frame, columns=col_ids, show="headings", height=15)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Treeview",
                        background="#1A1A1A",
                        foreground="#FFFFFF",
                        fieldbackground="#141414",
                        rowheight=24,
                        font=("Consolas", 9))
        style.configure("Treeview.Heading",
                        background="#242424",
                        foreground="#00FFCC",
                        font=("Arial", 8, "bold"))
        style.map("Treeview", background=[("selected", "#004d40")])
        style.map("Treeview.Heading", background=[("active", "#333333")])

        # Alternating row tag colors for scientific clarity
        self.table_tree.tag_configure("oddrow", background="#181818", foreground="#FFFFFF")
        self.table_tree.tag_configure("evenrow", background="#222222", foreground="#FFFFFF")

        for col_id, col_name, width in columns:
            self.table_tree.heading(col_id, text=col_name)
            self.table_tree.column(col_id, width=width, anchor="center")

        vsb = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=self.table_tree.yview)
        hsb = ttk.Scrollbar(tree_frame, orient=tk.HORIZONTAL, command=self.table_tree.xview)
        self.table_tree.configure(xscrollcommand=hsb.set, yscrollcommand=vsb.set)

        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        hsb.pack(side=tk.BOTTOM, fill=tk.X)
        self.table_tree.pack(fill=tk.BOTH, expand=True)

    def get_active_trial_folder(self):
        prop_base = os.path.splitext(os.path.basename(self.prop_filename))[0]
        safe_prop = "".join(c for c in prop_base if c.isalnum() or c in ('_', '-')).strip() or "DefaultProp"
        folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_results", safe_prop)
        os.makedirs(folder, exist_ok=True)
        return folder

    def calculate_bemt_comparison(self, rpm, meas_th_g, meas_tq_nm):
        bemt_th_g = 0.0
        bemt_tq_nm = 0.0
        bemt_mech_w = 0.0

        if rpm > 100:
            if BEMT_SOLVER_AVAILABLE and self.cached_pe0_geom is not None:
                try:
                    radius_m, hub_radius_m, num_blades, radii_m, chords_m, twists_deg = self.cached_pe0_geom
                    flex_deg = RIGIDITY_PRESETS.get(self.current_rigidity_name, 0.0)
                    solver = StaticBEMTQSolver(radius_m, hub_radius_m, num_blades, radii_m, chords_m, twists_deg, cd_max=0.14)
                    T_bemt_N, Q_bemt_Nm = solver.compute_performance(rpm, flex_deg=flex_deg)
                    bemt_th_g = float(T_bemt_N * (1000.0 / 9.80665))
                    bemt_tq_nm = float(Q_bemt_Nm)
                except Exception as err:
                    print(f"[BEMT CALC WARN] Solver evaluation failed at {rpm:.0f} RPM: {err}")
                    bemt_th_g = 0.0
                    bemt_tq_nm = 0.0

            # Fallback to CT/CP proxies if solver is unavailable or returned zero
            if bemt_th_g <= 0.0 and self.ct_proxy > 0:
                n_rev = rpm / 60.0
                dia_m = self.prop_diameter_m
                T_N = self.ct_proxy * RHO_AIR * (n_rev**2) * (dia_m**4)
                bemt_th_g = float(T_N * (1000.0 / 9.80665))
                bemt_tq_nm = float((self.cp_proxy * RHO_AIR * (n_rev**2) * (dia_m**5)) / (2.0 * np.pi))

            omega = rpm * (2.0 * np.pi / 60.0)
            bemt_mech_w = float(bemt_tq_nm * omega)

        # Residual percentage errors (with signed strings in table)
        th_err_pct = ((meas_th_g - bemt_th_g) / bemt_th_g * 100.0) if bemt_th_g > 1e-3 else 0.0
        tq_err_pct = ((meas_tq_nm - bemt_tq_nm) / bemt_tq_nm * 100.0) if bemt_tq_nm > 1e-4 else 0.0

        # Empirical dimensionless coefficients CT and CP
        n_rev = rpm / 60.0
        dia_m = self.prop_diameter_m
        if n_rev > 1.0 and dia_m > 0:
            meas_T_N = meas_th_g * (9.80665 / 1000.0)
            meas_ct = float(meas_T_N / (RHO_AIR * (n_rev**2) * (dia_m**4)))
            meas_cp = float((meas_tq_nm * 2.0 * np.pi * n_rev) / (RHO_AIR * (n_rev**3) * (dia_m**5)))
            ct_ratio_str = f"{meas_ct:.4f} / {self.ct_proxy:.4f}"
            cp_ratio_str = f"{meas_cp:.4f} / {self.cp_proxy:.4f}"
        else:
            meas_ct, meas_cp = 0.0, 0.0
            ct_ratio_str, cp_ratio_str = "N/A", "N/A"

        return {
            "bemt_th_g": bemt_th_g,
            "bemt_tq_nm": bemt_tq_nm,
            "bemt_mech_w": bemt_mech_w,
            "th_err_pct": th_err_pct,
            "tq_err_pct": tq_err_pct,
            "meas_ct": meas_ct,
            "meas_cp": meas_cp,
            "ct_ratio_str": ct_ratio_str,
            "cp_ratio_str": cp_ratio_str
        }

    def finalize_step_statistics(self, target_rpm, pwm_val, manual_snapshot=False):
        if manual_snapshot:
            if len(self.thrust_data) < 3:
                self.lbl_table_status.configure(text="Not enough samples for snapshot!", fg="#FF5555")
                return
            n_take = min(25, len(self.thrust_data))
            rpms = np.array(self.rpm_data[-n_take:])
            thrusts = np.array(self.thrust_data[-n_take:])
            torques = np.array(self.torque_data[-n_take:])
            currents = np.array(self.current_data[-n_take:])
            powers = np.array(self.power_data[-n_take:])
        else:
            if not self.current_step_buffer:
                print(f"[SWEEP STEP] Warning: No telemetry buffer collected for target {target_rpm} RPM.")
                return
            # Discard initial 35% settling window to isolate steady-state condition
            raw_len = len(self.current_step_buffer)
            start_idx = int(raw_len * 0.35) if raw_len >= 5 else 0
            settled_samples = self.current_step_buffer[start_idx:]
            if not settled_samples:
                settled_samples = self.current_step_buffer

            rpms = np.array([s["rpm"] for s in settled_samples])
            thrusts = np.array([s["thrust"] for s in settled_samples])
            torques = np.array([s["torque"] for s in settled_samples])
            currents = np.array([s["current"] for s in settled_samples])
            powers = np.array([s["power"] for s in settled_samples])

            self.current_step_buffer = []

        mean_rpm = float(np.mean(rpms))
        std_rpm = float(np.std(rpms)) if len(rpms) > 1 else 0.0

        mean_th = float(np.mean(thrusts))
        std_th = float(np.std(thrusts)) if len(thrusts) > 1 else 0.0

        mean_tq = float(np.mean(torques))
        std_tq = float(np.std(torques)) if len(torques) > 1 else 0.0

        mean_cur = float(np.mean(currents))
        std_cur = float(np.std(currents)) if len(currents) > 1 else 0.0

        mean_elec_pwr = float(np.mean(powers))
        std_elec_pwr = float(np.std(powers)) if len(powers) > 1 else 0.0

        # Mechanical Power = Torque * omega
        omega = mean_rpm * (2.0 * np.pi / 60.0)
        meas_mech_w = float(mean_tq * omega)

        # Efficiencies
        motor_eff = (meas_mech_w / mean_elec_pwr * 100.0) if mean_elec_pwr > 0.5 else 0.0
        aero_eff = (mean_th / mean_elec_pwr) if mean_elec_pwr > 0.5 else 0.0

        # BEMT Theoretical Comparison
        bemt = self.calculate_bemt_comparison(mean_rpm, mean_th, mean_tq)

        step_num = len(self.trial_summary_rows) + 1
        t_rpm_val = target_rpm if not manual_snapshot else int(round(mean_rpm))

        # Format display row for Treeview
        row_display = (
            str(step_num),
            str(int(pwm_val)),
            str(int(t_rpm_val)),
            f"{mean_rpm:.0f} ± {std_rpm:.0f}",
            f"{mean_th:.1f} ± {std_th:.1f}",
            f"{bemt['bemt_th_g']:.1f}",
            f"{bemt['th_err_pct']:+.1f}%",
            f"{mean_tq:.4f} ± {std_tq:.4f}",
            f"{bemt['bemt_tq_nm']:.4f}",
            f"{bemt['tq_err_pct']:+.1f}%",
            f"{meas_mech_w:.1f}",
            f"{bemt['bemt_mech_w']:.1f}",
            f"{mean_elec_pwr:.1f}",
            f"{motor_eff:.1f}%",
            f"{aero_eff:.2f}",
            bemt['ct_ratio_str'],
            bemt['cp_ratio_str']
        )

        row_tag = "evenrow" if step_num % 2 == 0 else "oddrow"
        self.table_tree.insert("", tk.END, values=row_display, tags=(row_tag,))
        self.table_tree.yview_moveto(1.0)

        # Structured dict for Excel export
        summary_record = {
            "Step": step_num,
            "PWM (us)": int(pwm_val),
            "Target RPM": int(t_rpm_val),
            "Actual RPM Mean": round(mean_rpm, 1),
            "Actual RPM Std": round(std_rpm, 1),
            "Meas Thrust (g) Mean": round(mean_th, 2),
            "Meas Thrust (g) Std": round(std_th, 2),
            "BEMT Thrust (g)": round(bemt["bemt_th_g"], 2),
            "Thrust Error (%)": round(bemt["th_err_pct"], 2),
            "Meas Torque (Nm) Mean": round(mean_tq, 5),
            "Meas Torque (Nm) Std": round(std_tq, 5),
            "BEMT Torque (Nm)": round(bemt["bemt_tq_nm"], 5),
            "Torque Error (%)": round(bemt["tq_err_pct"], 2),
            "Meas Mech Power (W)": round(meas_mech_w, 2),
            "BEMT Mech Power (W)": round(bemt["bemt_mech_w"], 2),
            "Elec Power (W) Mean": round(mean_elec_pwr, 2),
            "Elec Power (W) Std": round(std_elec_pwr, 2),
            "Current (A) Mean": round(mean_cur, 3),
            "Current (A) Std": round(std_cur, 3),
            "Motor Efficiency (%)": round(motor_eff, 2),
            "Aero Efficiency (g/W)": round(aero_eff, 3),
            "Meas CT": round(bemt["meas_ct"], 5),
            "BEMT CT": round(self.ct_proxy, 5),
            "Meas CP": round(bemt["meas_cp"], 5),
            "BEMT CP": round(self.cp_proxy, 5),
            "Rigidity Preset": self.current_rigidity_name
        }
        self.trial_summary_rows.append(summary_record)

        status_msg = f"Step {step_num} recorded: {mean_rpm:.0f} RPM | {mean_th:.1f}g (BEMT: {bemt['bemt_th_g']:.1f}g, {bemt['th_err_pct']:+.1f}%)"
        self.lbl_table_status.configure(text=status_msg, fg="#00FFCC")
        print(f"[RESEARCH TABLE] {status_msg}")

    def snapshot_current_point(self):
        if len(self.rpm_data) < 3 or (self.rpm_data and self.rpm_data[-1] < 50):
            self.lbl_table_status.configure(text="Motor stopped or idle (<50 RPM). Spin motor before snapshot.", fg="#FFCC00")
            return
        pwm_val = int(self.pwm_slider.get())
        target_rpm = int(self.rpm_data[-1])
        self.finalize_step_statistics(target_rpm=target_rpm, pwm_val=pwm_val, manual_snapshot=True)

    def export_current_trial_excel(self):
        if not self.trial_summary_rows:
            messagebox.showinfo("Export Excel", "No research table rows recorded yet.\nRun an Auto Sweep or click 'Snapshot Point' first.")
            return

        trial_tag = self.ent_trial_tag.get().strip() or "Trial"
        trial_num = self.ent_trial_num.get().strip() or "1"
        prop_base = os.path.splitext(os.path.basename(self.prop_filename))[0]
        safe_prop = "".join(c for c in prop_base if c.isalnum() or c in ('_', '-')).strip() or "DefaultProp"
        safe_tag = "".join(c for c in trial_tag if c.isalnum() or c in ('_', '-')).strip() or "Trial"

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{safe_prop}_{safe_tag}_Trial{trial_num}_{timestamp_str}.xlsx"

        folder = self.get_active_trial_folder()
        filepath = os.path.join(folder, filename)

        try:
            # Sheet 1: Summary
            df_summary = pd.DataFrame(self.trial_summary_rows)

            # Sheet 2: Raw Telemetry
            if self.current_trial_raw_log:
                df_raw = pd.DataFrame(self.current_trial_raw_log)
                col_rename = {
                    "ts": "Timestamp_ms",
                    "thrust": "Thrust_g",
                    "torque": "Torque_Nm",
                    "rpm": "RPM",
                    "current": "Current_A",
                    "power": "Elec_Power_W",
                    "t1": "Cell_TQ1_g",
                    "t2": "Cell_TQ2_g",
                    "pwm": "PWM_Command_us",
                    "target_rpm": "Target_RPM"
                }
                df_raw = df_raw.rename(columns={k: v for k, v in col_rename.items() if k in df_raw.columns})
            else:
                df_raw = pd.DataFrame({
                    "Timestamp_ms": self.timestamps,
                    "Thrust_g": self.thrust_data,
                    "Torque_Nm": self.torque_data,
                    "RPM": self.rpm_data,
                    "Current_A": self.current_data,
                    "Power_W": self.power_data
                })

            # Sheet 3: Trial Metadata
            metadata_items = [
                ("Propeller Model", self.prop_filename),
                ("Propeller Diameter (in)", f"{self.prop_diameter_in:.2f}"),
                ("Propeller Diameter (m)", f"{self.prop_diameter_m:.4f}"),
                ("Rigidity Preset", self.current_rigidity_name),
                ("Rigidity Washout (deg)", RIGIDITY_PRESETS.get(self.current_rigidity_name, 0.0)),
                ("Motor KV Rating", MOTOR_KV),
                ("Motor Resistance (Ohm)", MOTOR_RESISTANCE),
                ("Motor No-Load Current (A)", MOTOR_NO_LOAD_CURRENT),
                ("Fixed Bus Voltage (V)", FIXED_VOLTAGE),
                ("BEMT CT Calibrated Proxy", self.ct_proxy),
                ("BEMT CP Calibrated Proxy", self.cp_proxy),
                ("Air Density Rho (kg/m3)", RHO_AIR),
                ("BEMT Solver Available", BEMT_SOLVER_AVAILABLE),
                ("Trial Tag", trial_tag),
                ("Trial Number", trial_num),
                ("Generated Timestamp", datetime.now().strftime("%Y-%m-%d %H:%M:%S")),
                ("Number of Steady-State Steps", len(self.trial_summary_rows)),
                ("Total Raw Samples", len(df_raw))
            ]
            df_metadata = pd.DataFrame(metadata_items, columns=["Parameter", "Value"])

            # Write multi-sheet Excel file using openpyxl
            with pd.ExcelWriter(filepath, engine='openpyxl') as writer:
                df_summary.to_excel(writer, sheet_name='Validation_Summary', index=False)
                df_raw.to_excel(writer, sheet_name='Raw_Telemetry_10Hz', index=False)
                df_metadata.to_excel(writer, sheet_name='Trial_Metadata', index=False)

                # Format columns in openpyxl for clean viewing
                try:
                    from openpyxl.styles import Font
                    for sheet_name in writer.sheets:
                        ws = writer.sheets[sheet_name]
                        for cell in ws[1]:
                            cell.font = Font(bold=True)
                        for col in ws.columns:
                            max_len = max(len(str(cell.value or '')) for cell in col)
                            col_letter = col[0].column_letter
                            ws.column_dimensions[col_letter].width = max(max_len + 3, 12)
                except Exception as fmt_err:
                    print(f"[EXCEL STYLING WARN] Could not apply font formatting: {fmt_err}")

            # Also generate a companion CSV for quick inspection
            csv_path = os.path.splitext(filepath)[0] + "_summary.csv"
            df_summary.to_csv(csv_path, index=False)

            success_msg = f"Exported: {filename} ({len(self.trial_summary_rows)} steps)"
            self.lbl_table_status.configure(text=success_msg, fg="#00FF66")
            print(f"[EXCEL EXPORT] Successfully created:\n  -> {filepath}\n  -> {csv_path}")

        except Exception as e:
            err_msg = f"Excel export failed: {e}"
            self.lbl_table_status.configure(text=err_msg, fg="#FF5555")
            print(f"[EXCEL ERROR] {err_msg}")
            messagebox.showerror("Export Failed", err_msg)

    def export_current_trial_csv(self):
        if not self.trial_summary_rows:
            messagebox.showinfo("Save CSV", "No research table rows recorded yet.")
            return

        trial_tag = self.ent_trial_tag.get().strip() or "Trial"
        trial_num = self.ent_trial_num.get().strip() or "1"
        prop_base = os.path.splitext(os.path.basename(self.prop_filename))[0]
        safe_prop = "".join(c for c in prop_base if c.isalnum() or c in ('_', '-')).strip() or "DefaultProp"
        safe_tag = "".join(c for c in trial_tag if c.isalnum() or c in ('_', '-')).strip() or "Trial"
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{safe_prop}_{safe_tag}_Trial{trial_num}_{timestamp_str}_summary.csv"

        folder = self.get_active_trial_folder()
        filepath = os.path.join(folder, filename)

        try:
            df_summary = pd.DataFrame(self.trial_summary_rows)
            df_summary.to_csv(filepath, index=False)
            self.lbl_table_status.configure(text=f"Saved CSV: {filename}", fg="#00FF66")
            print(f"[CSV EXPORT] Saved summary CSV to: {filepath}")
        except Exception as e:
            print(f"[CSV ERROR] {e}")
            messagebox.showerror("CSV Save Error", str(e))

    def open_results_folder(self):
        folder = self.get_active_trial_folder()
        try:
            os.startfile(folder)
        except Exception:
            import subprocess
            subprocess.Popen(f'explorer "{folder}"')

    def clear_table_data(self):
        for item in self.table_tree.get_children():
            self.table_tree.delete(item)
        self.trial_summary_rows.clear()
        self.current_step_buffer.clear()
        self.current_trial_raw_log.clear()
        self.lbl_table_status.configure(text="Table cleared. Ready for next trial.", fg="#00FFCC")
        print("[RESEARCH TABLE] Table and trial memory buffer cleared.")

    def clear_bemt_history(self):
        self.bemt_history_rpm.clear()
        self.bemt_history_thrust.clear()
        self.bemt_history_torque.clear()
        self.plot_dirty = True
        self.render_plots()
        print("[GRAPH] BEMT scatter history cleared.")

    def cycle_rotor_direction_mode(self, event=None):
        modes = ["AUTO", "FORCE_CCW", "FORCE_CW"]
        idx = modes.index(self.rotor_direction_mode)
        self.rotor_direction_mode = modes[(idx + 1) % len(modes)]
        self.update_rotor_direction_pill()
        print(f"[ROTOR DIRECTION] Switched mode to: {self.rotor_direction_mode}")

    def update_rotor_direction_pill(self):
        if not hasattr(self, 'lbl_rotor_dir'):
            return
        if self.rotor_direction_mode == "FORCE_CCW":
            self.lbl_rotor_dir.configure(text="🔄 Rotor: CCW (Fixed)", bg="#003322", fg="#00FFCC")
        elif self.rotor_direction_mode == "FORCE_CW":
            self.lbl_rotor_dir.configure(text="🔁 Rotor: CW (Fixed)", bg="#332200", fg="#FFCC00")
        else: # AUTO
            if self.detected_direction == "CCW":
                self.lbl_rotor_dir.configure(text="🔄 Rotor: CCW (Auto)", bg="#003322", fg="#00FFCC")
            elif self.detected_direction == "CW":
                self.lbl_rotor_dir.configure(text="🔁 Rotor: CW (Auto)", bg="#332200", fg="#FFCC00")
            else:
                self.lbl_rotor_dir.configure(text="🔄 Rotor: Auto (CCW)", bg="#1A2E26", fg="#88EEDD")

    def on_rigidity_changed(self, event=None):
        selected = self.cbo_rigidity.get()
        if selected in RIGIDITY_PRESETS:
            self.current_rigidity_name = selected
            print(f"[AERO CONFIG] Propeller rigidity profile changed to: '{selected}' (washout={RIGIDITY_PRESETS[selected]:.1f}°)")
            self.recompute_bemt_coefficients()

    def recompute_bemt_coefficients(self):
        if not BEMT_SOLVER_AVAILABLE or self.cached_pe0_geom is None:
            return
        try:
            radius_m, hub_radius_m, num_blades, radii_m, chords_m, twists_deg = self.cached_pe0_geom
            dia_m = radius_m * 2.0
            dia_in = dia_m / 0.0254
            flex_deg = RIGIDITY_PRESETS.get(self.current_rigidity_name, 0.0)

            # Solve BEMT with Module C (cd_max=0.14) and Module D (flex_deg)
            solver = StaticBEMTQSolver(radius_m, hub_radius_m, num_blades, radii_m, chords_m, twists_deg, cd_max=0.14)
            ref_rpm = 6000.0
            T_ref, Q_ref = solver.compute_performance(ref_rpm, flex_deg=flex_deg)
            n_ref = ref_rpm / 60.0

            ct = T_ref / (RHO_AIR * (n_ref**2) * (dia_m**4))
            cp = (Q_ref * 2.0 * np.pi * n_ref) / (RHO_AIR * (n_ref**3) * (dia_m**5))

            self.prop_diameter_in = dia_in
            self.prop_diameter_m = dia_m
            self.ct_proxy = ct
            self.cp_proxy = cp

            if hasattr(self, 'ent_prop_dia'):
                self.ent_prop_dia.delete(0, tk.END)
                self.ent_prop_dia.insert(0, f"{dia_in:.2f}")
            if hasattr(self, 'lbl_loaded_prop'):
                self.lbl_loaded_prop.configure(text=f"Prop: {self.prop_filename}")

            print(f"[BEMT RECALC] Model='{self.prop_filename}' | Rigidity='{self.current_rigidity_name}' ({flex_deg:.1f}°): CT={ct:.4f}, CP={cp:.4f}")
            if self.view_mode == "bemt":
                self.update_plot_mode_styling()
            self.plot_dirty = True
            if hasattr(self, 'canvas'):
                self.render_plots()

        except Exception as err:
            print(f"[BEMT RECALC ERROR] {err}")

    def choose_pe0_file(self):
        default_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "PE0-FILES_WEB")
        if not os.path.isdir(default_dir):
            default_dir = os.path.dirname(os.path.abspath(__file__))

        selected_file = filedialog.askopenfilename(
            initialdir=default_dir,
            title="Select APC Propeller Geometry File (.PE0)",
            filetypes=[("APC Propeller Geometry (*.PE0)", "*.PE0"), ("All Files (*.*)", "*.*")]
        )
        if not selected_file:
            return

        try:
            self.cached_pe0_geom = parse_pe0_file(selected_file)
            self.prop_filename = os.path.basename(selected_file)

            # Smart auto-detection of prop rigidity based on filename
            fname_upper = self.prop_filename.upper()
            if "SF" in fname_upper or "SLOW" in fname_upper:
                auto_rigidity = "Soft / Slow Flyer"
            elif "CF" in fname_upper or "CARBON" in fname_upper:
                auto_rigidity = "Stiff / Carbon Fiber"
            else:
                auto_rigidity = "Thin Nylon (APC-E)"

            self.current_rigidity_name = auto_rigidity
            if hasattr(self, 'cbo_rigidity'):
                self.cbo_rigidity.set(auto_rigidity)

            self.recompute_bemt_coefficients()
            print(f"[PROP GEOM] Auto-detected '{auto_rigidity}' for '{self.prop_filename}'")

        except Exception as e:
            print(f"[PROP LOAD ERROR] Failed to load {selected_file}: {e}")
            messagebox.showerror("Error Loading Propeller", f"Failed to parse APC .PE0 file:\n{e}")

    def update_prop_diameter(self, event=None):
        try:
            val = float(self.ent_prop_dia.get())
            if 1.0 <= val <= 30.0:
                self.prop_diameter_in = val
                self.prop_diameter_m = val * 0.0254
                print(f"[AERO CONFIG] Propeller diameter updated: {val:.2f} inches ({self.prop_diameter_m:.4f} m)")
                if self.view_mode == "bemt":
                    self.update_plot_mode_styling()
                self.plot_dirty = True
                self.render_plots()
        except ValueError:
            pass

    def init_plots(self):
        self.ax1_twin = self.ax1.twinx()
        self.ax2_twin = self.ax2.twinx()

        for ax in [self.ax1, self.ax1_twin, self.ax2, self.ax2_twin]:
            ax.set_facecolor('#161616')
            for spine in ax.spines.values():
                spine.set_color('#2E2E2E')
                spine.set_linewidth(1.0)

        self.ax1.tick_params(colors='#AAAAAA', labelsize=8)
        self.ax1.grid(True, color='#262626', linestyle='--', linewidth=0.7, alpha=0.7)
        self.ax2.tick_params(colors='#AAAAAA', labelsize=8)
        self.ax2.grid(True, color='#262626', linestyle='--', linewidth=0.7, alpha=0.7)

        # Persistent Line Artists for Mode 1: Time Strip-Chart
        self.line_time_thrust, = self.ax1.plot([], [], color='#00FFCC', linewidth=2, label='Thrust (g)')
        self.line_time_torque, = self.ax1_twin.plot([], [], color='#FF66CC', linewidth=1.8, linestyle='--', label='Torque (N·m)')
        self.line_time_current, = self.ax2.plot([], [], color='#FFCC00', linewidth=2, label='Current (A)')
        self.line_time_rpm, = self.ax2_twin.plot([], [], color='#00FF66', linewidth=1.5, linestyle=':', label='RPM')

        # Persistent Line & Scatter Artists for Mode 2: BEMT Aerodynamic Validation
        self.scatter_bemt_thrust, = self.ax1.plot([], [], color='#00FFCC', marker='o', markersize=4, linestyle='', label='Measured Thrust (g)')
        self.scatter_bemt_torque, = self.ax1_twin.plot([], [], color='#FF66CC', marker='s', markersize=4, linestyle='', label='Measured Torque (N·m)')
        self.line_bemt_theory_thrust, = self.ax1.plot([], [], color='#00B4D8', linestyle='-', linewidth=2, label='BEMT Model')
        self.line_bemt_theory_torque, = self.ax1_twin.plot([], [], color='#E056FD', linestyle='--', linewidth=2, label='BEMT Model')
        self.line_bemt_current, = self.ax2.plot([], [], color='#FFCC00', linewidth=2, label='Current (A)')

        # Setup initial mode styling & persistent legends
        self.update_plot_mode_styling()

    def update_plot_mode_styling(self):
        if self.view_mode == "time":
            # Visibility
            self.line_time_thrust.set_visible(True)
            self.line_time_torque.set_visible(True)
            self.line_time_current.set_visible(True)
            self.line_time_rpm.set_visible(True)

            self.scatter_bemt_thrust.set_visible(False)
            self.scatter_bemt_torque.set_visible(False)
            self.line_bemt_theory_thrust.set_visible(False)
            self.line_bemt_theory_torque.set_visible(False)
            self.line_bemt_current.set_visible(False)

            self.ax1.set_title("Real-Time Telemetry: Live Thrust & Torque Time History", fontsize=11, fontweight='bold', color='#FFFFFF')
            self.ax1.set_xlabel("Time (Seconds)", color='#FFFFFF', fontsize=9)
            self.ax1.set_ylabel("Live Thrust (grams)", color='#00FFCC', fontsize=9, fontweight='bold')
            self.ax1.tick_params(colors='#AAAAAA', labelsize=8)

            self.ax1_twin.set_ylabel("Live Torque (N·m)", color='#FF66CC', fontsize=9, fontweight='bold')
            self.ax1_twin.tick_params(colors='#FFFFFF', labelsize=8)

            self.ax2.set_xlabel("Time (Seconds)", color='#FFFFFF', fontsize=9)
            self.ax2.set_ylabel("ESC Circuit Draw (Amperes)", color='#FFCC00', fontsize=9, fontweight='bold')
            self.ax2.tick_params(colors='#FFFFFF', labelsize=8)

            self.ax2_twin.set_ylabel("Motor Speed (RPM)", color='#00FF66', fontsize=9, fontweight='bold')
            self.ax2_twin.tick_params(colors='#FFFFFF', labelsize=8)

            # Legends
            self.ax1.legend(handles=[self.line_time_thrust], loc='upper left', frameon=True, facecolor='#181818', edgecolor='#333333', framealpha=0.88, labelcolor='#E0E0E0', fontsize=8)
            self.ax1_twin.legend(handles=[self.line_time_torque], loc='upper right', frameon=True, facecolor='#181818', edgecolor='#333333', framealpha=0.88, labelcolor='#E0E0E0', fontsize=8)
            self.ax2.legend(handles=[self.line_time_current], loc='upper left', frameon=True, facecolor='#181818', edgecolor='#333333', framealpha=0.88, labelcolor='#E0E0E0', fontsize=8)
            self.ax2_twin.legend(handles=[self.line_time_rpm], loc='upper right', frameon=True, facecolor='#181818', edgecolor='#333333', framealpha=0.88, labelcolor='#E0E0E0', fontsize=8)

        else: # BEMT mode
            self.line_time_thrust.set_visible(False)
            self.line_time_torque.set_visible(False)
            self.line_time_current.set_visible(False)
            self.line_time_rpm.set_visible(False)

            self.scatter_bemt_thrust.set_visible(True)
            self.scatter_bemt_torque.set_visible(True)
            self.line_bemt_theory_thrust.set_visible(True)
            self.line_bemt_theory_torque.set_visible(True)
            self.line_bemt_current.set_visible(True)

            self.ax2_twin.set_ylabel("")
            self.ax2_twin.set_yticks([])
            leg2_twin = self.ax2_twin.get_legend()
            if leg2_twin:
                leg2_twin.remove()

            self.ax1.set_xlabel("Rotational Velocity (RPM)", color='#FFFFFF', fontsize=9)
            self.ax1.set_ylabel("Static Thrust (grams)", color='#00FFCC', fontsize=9, fontweight='bold')
            rigidity_short = self.current_rigidity_name.split()[0]
            self.ax1.set_title(f"AeroSandbox BEMT Validation ({self.prop_filename} | {self.prop_diameter_in:.2f}\" | {self.current_rigidity_name})", fontsize=10, fontweight='bold', color='#FFFFFF')
            self.ax1_twin.set_ylabel("Aerodynamic Torque (N·m)", color='#FF66CC', fontsize=9, fontweight='bold')
            self.ax1_twin.tick_params(colors='#FFFFFF', labelsize=8)

            self.ax2.set_xlabel("Time (Seconds)", color='#FFFFFF', fontsize=9)
            self.ax2.set_ylabel("ESC Circuit Draw (Amperes)", color='#FFCC00', fontsize=9, fontweight='bold')
            self.ax2.tick_params(colors='#FFFFFF', labelsize=8)

            self.ax1.legend(handles=[self.scatter_bemt_thrust, self.line_bemt_theory_thrust],
                            labels=['Measured Thrust (g)', f'BEMT ({self.prop_filename} | {rigidity_short})'],
                            loc='upper left', frameon=True, facecolor='#181818', edgecolor='#333333', framealpha=0.88, labelcolor='#E0E0E0', fontsize=8)
            self.ax1_twin.legend(handles=[self.scatter_bemt_torque, self.line_bemt_theory_torque],
                                 labels=['Measured Torque (N·m)', f'BEMT Model ({rigidity_short})'],
                                 loc='upper right', frameon=True, facecolor='#181818', edgecolor='#333333', framealpha=0.88, labelcolor='#E0E0E0', fontsize=8)
            self.ax2.legend(handles=[self.line_bemt_current],
                            labels=['Current (A)'],
                            loc='upper left', frameon=True, facecolor='#181818', edgecolor='#333333', framealpha=0.88, labelcolor='#E0E0E0', fontsize=8)

    def update_safety_alarms(self, current, voltage, rpm, pwm):
        if not hasattr(self, 'card_elec') or not hasattr(self, 'card_rpm'):
            return

        # 1. High Current Alarm
        if current >= 30.0:
            self.card_elec.configure(highlightbackground="#FF2222", highlightthickness=2)
            self.lbl_elec_alert.configure(text="⚠️ DANGER (>30A)", fg="#FF2222")
        elif current >= 22.0:
            self.card_elec.configure(highlightbackground="#FF8800", highlightthickness=1)
            self.lbl_elec_alert.configure(text="⚠️ WARN (>22A)", fg="#FF8800")
        else:
            self.card_elec.configure(highlightbackground="#333333", highlightthickness=1)
            self.lbl_elec_alert.configure(text="", fg="#FF3300")

        # 2. Motor Stall Detection (High PWM commanded, but motor not rotating)
        if pwm >= 1250 and rpm < 200:
            self.card_rpm.configure(highlightbackground="#FF2222", highlightthickness=2)
            self.lbl_rpm_target.configure(text="⚠️ MOTOR STALL", fg="#FF2222")
        else:
            self.card_rpm.configure(highlightbackground="#333333", highlightthickness=1)
            if self.in_auto_sweep and self.current_step_idx < len(self.sweep_targets):
                target = self.sweep_targets[self.current_step_idx]
                self.lbl_rpm_target.configure(text=f"Target: {target} RPM", fg="#00E5FF")
            else:
                self.lbl_rpm_target.configure(text="Target: ---", fg="#777777")

    def update_kpi_cards(self, v, cur, p_elec, th_g, tq_nm, rpm=0.0, t1_g=0.0, t2_g=0.0):
        # 1. Thrust Card
        if hasattr(self, 'lbl_th_val'):
            th_N = th_g * 0.00980665
            self.lbl_th_val.configure(text=f"{th_g:.1f}")
            self.lbl_th_secondary.configure(text=f"{th_N:.2f} N")

        # 2. Torque Card
        if hasattr(self, 'lbl_tq_val'):
            tq_mNm = tq_nm * 1000.0
            dir_tag = f" ({self.detected_direction})" if self.detected_direction in ["CCW", "CW"] else ""
            self.lbl_tq_val.configure(text=f"{tq_nm:.4f}")
            self.lbl_tq_secondary.configure(text=f"{tq_mNm:.1f} mN·m{dir_tag}")
            self.lbl_tq_cells.configure(text=f"T1: {t1_g:+.1f}g  |  T2: {t2_g:+.1f}g")

        # 3. RPM Card
        if hasattr(self, 'lbl_rpm_val'):
            self.lbl_rpm_val.configure(text=f"{int(round(rpm))}")

        # 4. Electrical Card
        if hasattr(self, 'lbl_power_val'):
            self.lbl_power_val.configure(text=f"{p_elec:.1f}")
            self.lbl_vi_val.configure(text=f"{v:.2f} V  •  {cur:.2f} A")

        # 5. Efficiency & Mechanical Card
        if hasattr(self, 'lbl_mech_power'):
            omega = rpm * (2.0 * np.pi / 60.0)
            p_mech = tq_nm * omega
            self.lbl_mech_power.configure(text=f"Pmech: {p_mech:.1f} W")

            if p_elec > 1.0:
                aero_eff = th_g / p_elec
                motor_eff = (p_mech / p_elec) * 100.0
                self.lbl_aero_eff.configure(text=f"{aero_eff:.2f} g/W")
                self.lbl_motor_eff.configure(text=f"η: {motor_eff:.1f}%")
            else:
                self.lbl_aero_eff.configure(text="0.00 g/W")
                self.lbl_motor_eff.configure(text="η: 0.0%")

        # 6. Safety Alarms
        cur_pwm = self.sweep_current_pwm if self.in_auto_sweep else int(self.pwm_slider.get())
        self.update_safety_alarms(cur, v, rpm, cur_pwm)

        # 7. Backward Compatibility for metrics_lbl
        if hasattr(self, 'metrics_lbl'):
            self.metrics_lbl.configure(text=self.generate_metrics_string(v, cur, p_elec, th_g, tq_nm, rpm, t1_g, t2_g))

    def generate_metrics_string(self, v, cur, p, th, tq, rpm=0, t1=0.0, t2=0.0):
        dir_tag = f" ({self.detected_direction})" if self.detected_direction in ["CCW", "CW"] else ""
        return (f"Voltage:     {v:5.2f} V\n"
                f"Current:     {cur:5.2f} A\n"
                f"Elec. Power: {p:5.0f} W\n"
                f"Thrust:      {th:5.1f} g\n"
                f"Torque:      {tq:7.5f} N·m{dir_tag}\n"
                f"TQ Cells:    T1={t1:4.1f}g T2={t2:4.1f}g\n"
                f"Motor Speed: {rpm:5.0f} RPM")

    def on_slider_moved(self, val):
        pwm_val = int(float(val))
        if hasattr(self, 'ent_pwm'):
            self.ent_pwm.delete(0, tk.END)
            self.ent_pwm.insert(0, str(pwm_val))
        self.send_manual_pwm(pwm_val)

    def set_pwm_direct(self, val):
        val = max(1000, min(2000, int(val)))
        self.pwm_slider.set(val)
        if hasattr(self, 'ent_pwm'):
            self.ent_pwm.delete(0, tk.END)
            self.ent_pwm.insert(0, str(val))
        if self.is_connected and not self.in_auto_sweep and self.ser:
            try:
                self.ser.write(f"CTRL,{val}\n".encode())
                self.last_slider_send = time.time()
            except Exception as e:
                print(f"[PWM WRITE ERROR] {e}")

    def step_pwm(self, delta):
        cur = int(self.pwm_slider.get())
        self.set_pwm_direct(cur + delta)

    def on_pwm_entry_submit(self, event=None):
        try:
            val = int(self.ent_pwm.get().strip())
            self.set_pwm_direct(val)
        except ValueError:
            cur = int(self.pwm_slider.get())
            self.ent_pwm.delete(0, tk.END)
            self.ent_pwm.insert(0, str(cur))

    def emergency_motor_stop(self):
        print("[EMERGENCY] Emergency Stop triggered! Cutting motor immediately.")
        if self.in_auto_sweep:
            self.in_auto_sweep = False
            self.sweep_btn.configure(text="Start Auto Sweep", bg="#006633")
            self.set_input_fields_state(tk.NORMAL)
        if hasattr(self, 'lbl_sweep_status'):
            self.lbl_sweep_status.configure(text="Sweep: Stopped", fg="#FF4444")
        if hasattr(self, 'lbl_rpm_target'):
            self.lbl_rpm_target.configure(text="E-STOP ACTIVE", fg="#FF3333")
        self.sweep_current_pwm = 1000
        self.pwm_slider.set(1000)
        if hasattr(self, 'ent_pwm'):
            self.ent_pwm.delete(0, tk.END)
            self.ent_pwm.insert(0, "1000")
        if self.is_connected and self.ser:
            try:
                self.ser.write(b"CTRL,1000\n")
            except Exception as e:
                print(f"[E-STOP SERIAL ERROR] {e}")
        if hasattr(self, 'lbl_table_status'):
            self.lbl_table_status.configure(text="EMERGENCY MOTOR CUT ACTIVATED!", fg="#FF3333")

    def on_window_close(self):
        print("[SHUTDOWN] Window close event caught. Safely cutting motor and releasing hardware...")
        if self._resize_job is not None:
            try: self.root.after_cancel(self._resize_job)
            except Exception: pass
            self._resize_job = None
        if self._daq_loop_job is not None:
            try: self.root.after_cancel(self._daq_loop_job)
            except Exception: pass
            self._daq_loop_job = None
        if hasattr(self, 'csv_file') and self.csv_file and not self.csv_file.closed:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception: pass
        if self.is_connected and self.ser:
            try:
                self.ser.write(b"CTRL,1000\n")
                time.sleep(0.05)
                self.ser.close()
            except Exception:
                pass
        self.root.destroy()

    def send_manual_pwm(self, val):
        if self.is_connected and not self.in_auto_sweep:
            now = time.time()
            if now - self.last_slider_send > 0.04: 
                try:
                    self.ser.write(f"CTRL,{val}\n".encode())
                    self.last_slider_send = now
                except Exception as e:
                    print(f"Write error: {e}")

    def toggle_rpm_sweep(self, normal_finish=False):
        if not self.is_connected and not normal_finish:
            print("[SWEEP ERROR] System must be connected to hardware to launch.")
            return
            
        if self.in_auto_sweep:
            self.in_auto_sweep = False
            self.sweep_btn.configure(text="Start Auto Sweep", bg="#006633")
            if hasattr(self, 'lbl_sweep_status'):
                self.lbl_sweep_status.configure(text="Sweep: Idle", fg="#888888")
            self.set_input_fields_state(tk.NORMAL)
            self.sweep_current_pwm = 1000
            try:
                self.ser.write(b"CTRL,1000\n")
            except: pass
            self.pwm_slider.set(1000)

            if normal_finish:
                print("[SWEEP] Sweep sequence completed successfully.")
                if self.auto_export_excel.get():
                    self.export_current_trial_excel()
                if self.auto_increment_trial.get():
                    try:
                        cur_num = int(self.ent_trial_num.get())
                        self.ent_trial_num.delete(0, tk.END)
                        self.ent_trial_num.insert(0, str(cur_num + 1))
                        print(f"[TRIAL] Incremented trial index to {cur_num + 1}")
                    except Exception as err:
                        print(f"[TRIAL INC ERROR] {err}")
            else:
                print("[SWEEP] Automation sequence terminated manually. Motor cut.")
        else:
            try:
                start_rpm = int(self.ent_start.get())
                end_rpm = int(self.ent_end.get())
                step_rpm = int(self.ent_step.get())
                self.step_duration = float(self.ent_dur.get())
                
                self.sweep_targets = list(range(start_rpm, end_rpm + step_rpm, step_rpm))
                if not self.sweep_targets: return

                # Reset previous step buffers and treeview for the fresh sweep trial
                self.clear_table_data()
                self.lbl_table_status.configure(text=f"Sweep running: {len(self.sweep_targets)} steps...", fg="#FFCC00")

                self.in_auto_sweep = True
                self.current_step_idx = 0
                self.sweep_current_pwm = 1150  
                self.step_start_time = time.time()
                
                self.sweep_btn.configure(text="HALT SWEEP", bg="#990000")
                if hasattr(self, 'lbl_sweep_status'):
                    self.lbl_sweep_status.configure(text=f"Sweep: Step 1/{len(self.sweep_targets)} ({self.sweep_targets[0]} RPM)", fg="#00FFCC")
                self.set_input_fields_state(tk.DISABLED)
                print(f"[SWEEP] Initialized target profiles: {self.sweep_targets}")
            except ValueError:
                print("[INPUT ERROR] Please verify sweep entries contain valid numeric targets.")

    def set_input_fields_state(self, state_flag):
        self.pwm_slider.configure(state=state_flag)
        if hasattr(self, 'ent_pwm'):
            self.ent_pwm.configure(state=state_flag)
        self.ent_start.configure(state=state_flag)
        self.ent_end.configure(state=state_flag)
        self.ent_step.configure(state=state_flag)
        self.ent_dur.configure(state=state_flag)
        if hasattr(self, 'ent_trial_tag'):
            self.ent_trial_tag.configure(state=state_flag)
        if hasattr(self, 'ent_trial_num'):
            self.ent_trial_num.configure(state=state_flag)
        if hasattr(self, 'chk_auto_inc'):
            self.chk_auto_inc.configure(state=state_flag)
        if hasattr(self, 'chk_auto_exp'):
            self.chk_auto_exp.configure(state=state_flag)

    def trigger_tare(self):
        if self.is_connected and self.ser:
            try:
                self.ser.write(b"TARE\n")
                self.tare_offset_thrust = 0.0
                self.tare_offset_torque = 0.0
                print("[CALIBRATION] Hardware TARE command transmitted to ESP32-S3.")
            except Exception as e:
                print(f"[TARE ERROR] Failed to send TARE command: {e}")
        elif len(self.thrust_data) > 0:
            self.tare_offset_thrust = self.thrust_data[-1] + self.tare_offset_thrust
            self.tare_offset_torque = self.torque_data[-1] + self.tare_offset_torque
            print("[CALIBRATION] Baseline structural vectors updated (Software Tared).")

        if hasattr(self, 'btn_tare'):
            self.btn_tare.configure(text="⚖ Tared! (Zeroed)", fg="#00FFCC")
            self.root.after(1500, lambda: self.btn_tare.configure(text="⚖ Zero / Tare Rig", fg="#FFFFFF") if hasattr(self, 'btn_tare') else None)

    def open_calibration_window(self):
        cal_win = tk.Toplevel(self.root)
        cal_win.title("Load Cell Calibration (Saves to ESP32 Flash)")
        cal_win.geometry("540x650")
        cal_win.configure(bg="#1A1A1A")
        cal_win.resizable(False, False)

        tk.Label(cal_win, text="LOAD CELL CALIBRATION WIZARD", font=("Arial", 13, "bold"), fg="#FF6600", bg="#1A1A1A").pack(pady=(15, 4))
        tk.Label(cal_win, text="Calibration factors are permanently stored in ESP32 onboard Flash memory.",
                 font=("Arial", 9), fg="#AAAAAA", bg="#1A1A1A").pack(pady=(0, 12))

        # Step 1: Tare
        tare_f = tk.LabelFrame(cal_win, text=" Step 1: Zero / Tare Rig ", font=("Arial", 10, "bold"), fg="#00FFCC", bg="#222222", padx=10, pady=8)
        tare_f.pack(fill=tk.X, padx=15, pady=4)
        tk.Label(tare_f, text="Keep the test stand completely empty and still:", fg="#CCCCCC", bg="#222222", font=("Arial", 9)).pack(side=tk.LEFT, padx=5)
        def do_inv_current():
            if self.is_connected and self.ser:
                self.ser.write(b"REV_CURRENT\n")
        tk.Button(tare_f, text="Invert Current (+/-)", font=("Arial", 9), bg="#444444", fg="#FFFFFF", relief=tk.FLAT, padx=8, pady=4,
                  command=do_inv_current).pack(side=tk.RIGHT, padx=5)
        tk.Button(tare_f, text="Zero All Cells", font=("Arial", 9, "bold"), bg="#008855", fg="#FFFFFF", relief=tk.FLAT, padx=12, pady=4,
                  command=self.trigger_tare).pack(side=tk.RIGHT, padx=5)

        # Step 2: Thrust Cell
        th_f = tk.LabelFrame(cal_win, text=" Step 2: Thrust Load Cell (5kg) ", font=("Arial", 10, "bold"), fg="#00FFCC", bg="#222222", padx=10, pady=8)
        th_f.pack(fill=tk.X, padx=15, pady=4)
        tk.Label(th_f, text="Known Weight (grams):", fg="#CCCCCC", bg="#222222", font=("Arial", 9)).pack(side=tk.LEFT, padx=5)
        ent_th = tk.Entry(th_f, width=8, bg="#333333", fg="#FFFFFF", font=("Arial", 10), insertbackground="white")
        ent_th.insert(0, "500")
        ent_th.pack(side=tk.LEFT, padx=5)
        def do_cal_thrust():
            val = ent_th.get().strip()
            if self.is_connected and self.ser:
                self.ser.write(f"CAL,THRUST,{val}\n".encode())
                if hasattr(self, 'cal_status_lbl') and self.cal_status_lbl.winfo_exists():
                    self.cal_status_lbl.configure(text=f"Calibrating Thrust with {val}g... please wait...", fg="#FFCC00")
        tk.Button(th_f, text="Calibrate Thrust", font=("Arial", 9, "bold"), bg="#FF6600", fg="#FFFFFF", relief=tk.FLAT, padx=10, pady=4,
                  command=do_cal_thrust).pack(side=tk.RIGHT, padx=5)

        # Step 3: Torque Cell 1
        tq1_f = tk.LabelFrame(cal_win, text=" Step 3: Torque Cell 1 (1kg) ", font=("Arial", 10, "bold"), fg="#00FFCC", bg="#222222", padx=10, pady=8)
        tq1_f.pack(fill=tk.X, padx=15, pady=4)
        tk.Label(tq1_f, text="Known Weight (grams):", fg="#CCCCCC", bg="#222222", font=("Arial", 9)).pack(side=tk.LEFT, padx=5)
        ent_tq1 = tk.Entry(tq1_f, width=8, bg="#333333", fg="#FFFFFF", font=("Arial", 10), insertbackground="white")
        ent_tq1.insert(0, "100")
        ent_tq1.pack(side=tk.LEFT, padx=5)
        def do_cal_tq1():
            val = ent_tq1.get().strip()
            if self.is_connected and self.ser:
                self.ser.write(f"CAL,TQ1,{val}\n".encode())
                if hasattr(self, 'cal_status_lbl') and self.cal_status_lbl.winfo_exists():
                    self.cal_status_lbl.configure(text=f"Calibrating Torque 1 with {val}g... please wait...", fg="#FFCC00")
        tk.Button(tq1_f, text="Calibrate TQ 1", font=("Arial", 9, "bold"), bg="#FF6600", fg="#FFFFFF", relief=tk.FLAT, padx=10, pady=4,
                  command=do_cal_tq1).pack(side=tk.RIGHT, padx=5)

        # Step 4: Torque Cell 2
        tq2_f = tk.LabelFrame(cal_win, text=" Step 4: Torque Cell 2 (1kg) ", font=("Arial", 10, "bold"), fg="#00FFCC", bg="#222222", padx=10, pady=8)
        tq2_f.pack(fill=tk.X, padx=15, pady=4)
        tk.Label(tq2_f, text="Known Weight (grams):", fg="#CCCCCC", bg="#222222", font=("Arial", 9)).pack(side=tk.LEFT, padx=5)
        ent_tq2 = tk.Entry(tq2_f, width=8, bg="#333333", fg="#FFFFFF", font=("Arial", 10), insertbackground="white")
        ent_tq2.insert(0, "100")
        ent_tq2.pack(side=tk.LEFT, padx=5)
        def do_cal_tq2():
            val = ent_tq2.get().strip()
            if self.is_connected and self.ser:
                self.ser.write(f"CAL,TQ2,{val}\n".encode())
                if hasattr(self, 'cal_status_lbl') and self.cal_status_lbl.winfo_exists():
                    self.cal_status_lbl.configure(text=f"Calibrating Torque 2 with {val}g... please wait...", fg="#FFCC00")
        tk.Button(tq2_f, text="Calibrate TQ 2", font=("Arial", 9, "bold"), bg="#FF6600", fg="#FFFFFF", relief=tk.FLAT, padx=10, pady=4,
                  command=do_cal_tq2).pack(side=tk.RIGHT, padx=5)

        # Step 5: Torque Lever Arm Radius
        arm_f = tk.LabelFrame(cal_win, text=" Step 5: Torque Lever Arm Radius ", font=("Arial", 10, "bold"), fg="#00FFCC", bg="#222222", padx=10, pady=8)
        arm_f.pack(fill=tk.X, padx=15, pady=4)
        tk.Label(arm_f, text="Arm Radius (mm):", fg="#CCCCCC", bg="#222222", font=("Arial", 9)).pack(side=tk.LEFT, padx=5)
        ent_arm = tk.Entry(arm_f, width=8, bg="#333333", fg="#FFFFFF", font=("Arial", 10), insertbackground="white")
        ent_arm.insert(0, "40.0")
        ent_arm.pack(side=tk.LEFT, padx=5)
        def do_set_arm():
            val = ent_arm.get().strip()
            if self.is_connected and self.ser:
                self.ser.write(f"ARM,{val}\n".encode())
                if hasattr(self, 'cal_status_lbl') and self.cal_status_lbl.winfo_exists():
                    self.cal_status_lbl.configure(text=f"Setting Lever Arm to {val}mm... please wait...", fg="#FFCC00")
        tk.Button(arm_f, text="Save Lever Arm", font=("Arial", 9, "bold"), bg="#FF6600", fg="#FFFFFF", relief=tk.FLAT, padx=10, pady=4,
                  command=do_set_arm).pack(side=tk.RIGHT, padx=5)

        # Step 6: Dual Torque Cell Combination Mode
        mode_f = tk.LabelFrame(cal_win, text=" Step 6: Dual Torque Cell Mode ", font=("Arial", 10, "bold"), fg="#00FFCC", bg="#222222", padx=10, pady=8)
        mode_f.pack(fill=tk.X, padx=15, pady=4)
        def set_tq_mode(mode_code):
            if self.is_connected and self.ser:
                self.ser.write(f"TQMODE,{mode_code}\n".encode())
                if hasattr(self, 'cal_status_lbl') and self.cal_status_lbl.winfo_exists():
                    self.cal_status_lbl.configure(text=f"Switching Torque Mode to {mode_code}...", fg="#FFCC00")
        tk.Button(mode_f, text="Parallel Sum (T1+T2)", font=("Arial", 9, "bold"), bg="#008855", fg="#FFFFFF", relief=tk.FLAT, padx=6, pady=3,
                  command=lambda: set_tq_mode("SUM")).pack(side=tk.LEFT, padx=3)
        tk.Button(mode_f, text="Difference (T1-T2)", font=("Arial", 9), bg="#444444", fg="#FFFFFF", relief=tk.FLAT, padx=6, pady=3,
                  command=lambda: set_tq_mode("DIFF")).pack(side=tk.LEFT, padx=3)
        tk.Button(mode_f, text="T1 Only", font=("Arial", 8), bg="#333333", fg="#CCCCCC", relief=tk.FLAT, padx=5, pady=3,
                  command=lambda: set_tq_mode("T1")).pack(side=tk.LEFT, padx=2)
        tk.Button(mode_f, text="T2 Only", font=("Arial", 8), bg="#333333", fg="#CCCCCC", relief=tk.FLAT, padx=5, pady=3,
                  command=lambda: set_tq_mode("T2")).pack(side=tk.LEFT, padx=2)

        # Live Firmware Feedback Label
        status_box = tk.Frame(cal_win, bg="#111111", padx=10, pady=8, highlightbackground="#444444", highlightthickness=1)
        status_box.pack(fill=tk.X, padx=15, pady=(10, 4))
        self.cal_status_lbl = tk.Label(status_box, text="Status: Ready. Place weight and click Calibrate.",
                                       font=("Courier New", 9), fg="#00FFCC", bg="#111111", wraplength=490, justify=tk.LEFT)
        self.cal_status_lbl.pack(anchor="w")

        # Bottom Actions
        btn_box = tk.Frame(cal_win, bg="#1A1A1A")
        btn_box.pack(fill=tk.X, padx=15, pady=(8, 10))
        def do_cal_show():
            if self.is_connected and self.ser:
                self.ser.write(b"CAL,SHOW\n")
        def do_cal_reset():
            if self.is_connected and self.ser:
                self.ser.write(b"CAL,RESET\n")
        tk.Button(btn_box, text="Check Status", font=("Arial", 9), bg="#444444", fg="#FFFFFF", relief=tk.FLAT, padx=8, command=do_cal_show).pack(side=tk.LEFT, padx=(0, 6))
        tk.Button(btn_box, text="Reset Defaults", font=("Arial", 9), bg="#662222", fg="#FFFFFF", relief=tk.FLAT, padx=8, command=do_cal_reset).pack(side=tk.LEFT)
        tk.Button(btn_box, text="Done", font=("Arial", 9, "bold"), bg="#333333", fg="#FFFFFF", relief=tk.FLAT, padx=18, command=cal_win.destroy).pack(side=tk.RIGHT)

    def manage_closed_loop_sweep(self):
        if not self.in_auto_sweep or not self.is_connected:
            return
            
        now = time.time()
        elapsed = now - self.step_start_time
        target_rpm = self.sweep_targets[self.current_step_idx]
        
        rpm_error = target_rpm - self.current_live_rpm
        
        pwm_adjustment = 0
        if abs(rpm_error) > 35: 
            pwm_adjustment = int(rpm_error * 0.015) 
            pwm_adjustment = max(-12, min(12, pwm_adjustment))
            
        self.sweep_current_pwm += pwm_adjustment
        self.sweep_current_pwm = max(1100, min(1950, self.sweep_current_pwm)) 
        
        try:
            self.ser.write(f"CTRL,{self.sweep_current_pwm}\n".encode())
        except Exception as e:
            print(f"Sweep Serial Error: {e}")

        if elapsed >= self.step_duration:
            # Aggregate steady-state metrics and compare against BEMT for this completed step
            self.finalize_step_statistics(target_rpm, self.sweep_current_pwm)

            self.current_step_idx += 1
            if self.current_step_idx < len(self.sweep_targets):
                self.step_start_time = time.time()
                next_rpm = self.sweep_targets[self.current_step_idx]
                if hasattr(self, 'lbl_sweep_status'):
                    self.lbl_sweep_status.configure(text=f"Sweep: Step {self.current_step_idx + 1}/{len(self.sweep_targets)} ({next_rpm} RPM)", fg="#00FFCC")
                print(f"[SWEEP] Advancing step target profiles to {next_rpm} RPM")
            else:
                print("[SWEEP] Entire profile sequence successfully run and exported to validation reports.")
                self.toggle_rpm_sweep(normal_finish=True)

    def attempt_hardware_connection(self):
        # 1. Check if background discovery worker found a hardware node
        if self.discovered_ser is not None:
            self.ser = self.discovered_ser
            self.discovered_ser = None
            self.ser.timeout = 0.01
            self.ser.reset_input_buffer()
            self.is_connected = True
            self.status_frame.configure(bg="#003311", highlightbackground="#00FF33")
            self.status_lbl.configure(text=f"Hardware:\nConnected!\nESP32-S3 ({self.active_port})", fg="#00FF33", bg="#003311")
            print(f"[SERIAL] Successfully bound to telemetry node at {self.active_port}")
            return

        now = time.time()
        if now - self.last_scan_time < 2.0:
            return
        if self.scan_thread is not None and self.scan_thread.is_alive():
            return
        self.last_scan_time = now

        # Launch background port scanning thread to avoid blocking Tkinter main loop
        self.scan_thread = threading.Thread(target=self._scan_com_ports_worker, daemon=True)
        self.scan_thread.start()

    def _scan_com_ports_worker(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if not getattr(self, 'root', None) or not self.root.winfo_exists() or self.is_connected:
                return
            if p.device == 'COM1' and len(ports) > 1:
                continue
            try:
                test_ser = serial.Serial(p.device, BAUD_RATE, timeout=0.2)
                test_ser.dtr = True
                test_ser.rts = True
                time.sleep(0.1)
                
                if test_ser.in_waiting > 0:
                    lines = test_ser.read(test_ser.in_waiting).decode('utf-8', errors='ignore').splitlines()
                    for line in lines:
                        if line.strip().startswith("$RIG"):
                            self.active_port = p.device
                            self.discovered_ser = test_ser
                            return
                test_ser.close()
            except Exception:
                continue

    def update_daq_loop(self):
        if not self.is_connected:
            self.attempt_hardware_connection()
        else:
            try:
                latest_kpi_sample = None
                while self.ser and self.ser.in_waiting > 0:
                    raw_line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not raw_line.startswith("$RIG,"):
                        if any(raw_line.startswith(k) for k in ["[CAL", "[NVS", "[CMD", "[DEBUG", "[CURRENT", "[ARM", "[TQMODE", "---", "Thrust", "Torque"]):
                            print(f"[FIRMWARE] {raw_line}")
                            if hasattr(self, 'cal_status_lbl') and self.cal_status_lbl.winfo_exists():
                                color = "#00FFCC" if "SUCCESS" in raw_line or "saved" in raw_line or "[ARM]" in raw_line or "[TQMODE]" in raw_line else ("#FF5555" if "ERROR" in raw_line else "#FFCC00")
                                self.cal_status_lbl.configure(text=raw_line, fg=color)
                        continue
                    
                    # Protected Inner Packet Decoding (Ignores corrupted lines safely)
                    try:
                        data_tokens = raw_line.split(',')
                        if len(data_tokens) >= 6:
                            ts = int(data_tokens[1])
                            thrust = float(data_tokens[2]) - self.tare_offset_thrust
                            raw_torque = float(data_tokens[3]) - self.tare_offset_torque
                            rpm = float(data_tokens[4])
                            current = max(0.0, float(data_tokens[5]))
                            power = FIXED_VOLTAGE * current

                            if len(data_tokens) >= 8:
                                self.last_tq1_g = float(data_tokens[6])
                                self.last_tq2_g = float(data_tokens[7])
                            
                            # Smart CW / CCW Auto-Detection & Aerodynamic Torque Rectification
                            if rpm >= 300:
                                if self.rotor_direction_mode == "AUTO":
                                    # Reaction torque couple:
                                    # T2 pushes down (> T1) => stator reaction CW => rotor is CCW
                                    # T1 pushes down (> T2) => stator reaction CCW => rotor is CW
                                    if self.last_tq2_g > (self.last_tq1_g + 1.0):
                                        if self.detected_direction != "CCW":
                                            self.detected_direction = "CCW"
                                            self.update_rotor_direction_pill()
                                    elif self.last_tq1_g > (self.last_tq2_g + 1.0):
                                        if self.detected_direction != "CW":
                                            self.detected_direction = "CW"
                                            self.update_rotor_direction_pill()

                                # Aerodynamic resisting torque magnitude must always be positive for power & BEMT
                                torque = abs(raw_torque)
                            else:
                                if self.rotor_direction_mode == "AUTO" and self.detected_direction != "IDLE":
                                    self.detected_direction = "IDLE"
                                    self.update_rotor_direction_pill()
                                torque = raw_torque
                            
                            self.current_live_rpm = rpm
                            
                            self.timestamps.append(ts)
                            self.thrust_data.append(thrust)
                            self.torque_data.append(torque)
                            self.rpm_data.append(rpm)
                            self.current_data.append(current)
                            self.power_data.append(power)
                            
                            # Log sample into active sweep buffers
                            raw_sample = {
                                "ts": ts,
                                "thrust": thrust,
                                "torque": torque,
                                "rpm": rpm,
                                "current": current,
                                "power": power,
                                "t1": self.last_tq1_g,
                                "t2": self.last_tq2_g,
                                "pwm": self.sweep_current_pwm if self.in_auto_sweep else int(self.pwm_slider.get()),
                                "target_rpm": self.sweep_targets[self.current_step_idx] if (self.in_auto_sweep and self.current_step_idx < len(self.sweep_targets)) else 0
                            }
                            if self.in_auto_sweep:
                                self.current_step_buffer.append(raw_sample)
                                self.current_trial_raw_log.append(raw_sample)

                            # Accumulate points for BEMT validation when motor is spinning
                            if rpm >= 300:
                                self.bemt_point_counter += 1
                                if self.bemt_point_counter % 3 == 0:  # Sample at ~10 Hz
                                    self.bemt_history_rpm.append(rpm)
                                    self.bemt_history_thrust.append(thrust)
                                    self.bemt_history_torque.append(torque)
                                    if len(self.bemt_history_rpm) > 600:
                                        self.bemt_history_rpm.pop(0)
                                        self.bemt_history_thrust.pop(0)
                                        self.bemt_history_torque.pop(0)

                            if hasattr(self, 'csv_writer') and self.csv_writer:
                                self.csv_writer.writerow([ts, thrust, torque, rpm, current, power])
                            else:
                                with open(CSV_FILE_NAME, mode='a', newline='') as file:
                                    writer = csv.writer(file)
                                    writer.writerow([ts, thrust, torque, rpm, current, power])
                                
                            latest_kpi_sample = (FIXED_VOLTAGE, current, power, thrust, torque, rpm, self.last_tq1_g, self.last_tq2_g)
                            
                            # Rolling strip chart window: 250 points (~7.5 to 8 seconds of continuous live data)
                            if len(self.timestamps) > 250:
                                self.timestamps.pop(0)
                                self.thrust_data.pop(0)
                                self.torque_data.pop(0)
                                self.rpm_data.pop(0)
                                self.current_data.pop(0)
                                self.power_data.pop(0)

                            self.plot_dirty = True
                    except (ValueError, IndexError) as parse_err:
                        # Malformed frame caught and discarded without killing serial connection
                        continue
                
                # Batched KPI updates & CSV flush
                if latest_kpi_sample is not None:
                    self.update_kpi_cards(*latest_kpi_sample)
                if hasattr(self, 'csv_file') and self.csv_file and not self.csv_file.closed:
                    try: self.csv_file.flush()
                    except Exception: pass

                if self.in_auto_sweep:
                    self.manage_closed_loop_sweep()
                            
                now = time.time()
                if now - self.last_plot_time >= 0.10:
                    if self.view_mode != "table" and self.plot_dirty:
                        self.render_plots()
                    self.last_plot_time = now
                
            except (serial.SerialException, OSError) as serial_err:
                # True hardware disconnection (unplugged cable)
                print(f"[SERIAL HARDWARE DISCONNECT] {serial_err}")
                self.is_connected = False
                if self.ser:
                    try: self.ser.close()
                    except: pass
            except Exception as err:
                print(f"[DAQ LOOP EXCEPTION] {err}")

        if hasattr(self, 'root') and self.root.winfo_exists():
            self._daq_loop_job = self.root.after(30, self.update_daq_loop)

    def render_plots(self):
        if not self.timestamps:
            return

        time_sec = [(t - self.timestamps[0]) / 1000.0 for t in self.timestamps]

        # -------------------------------------------------------------
        # MODE 1: Continuous Real-Time Telemetry (Time Strip-Chart)
        # Always moves forward in time (left to right)! Never goes back.
        # -------------------------------------------------------------
        if self.view_mode == "time":
            self.line_time_thrust.set_data(time_sec, self.thrust_data)
            self.line_time_torque.set_data(time_sec, self.torque_data)
            self.line_time_current.set_data(time_sec, self.current_data)
            self.line_time_rpm.set_data(time_sec, self.rpm_data)

            # Minimum 100g span for Thrust; Torque autoscales naturally
            max_th = max(self.thrust_data) if self.thrust_data else 0.0
            min_th = min(self.thrust_data) if self.thrust_data else 0.0
            y_top = max(100.0, max_th * 1.15)
            y_bot = min(-5.0, min_th * 1.15) if min_th < -2.0 else -2.0
            self.ax1.set_ylim(y_bot, y_top)

            # Synchronize Time X-axis across BOTH plots
            t_min = time_sec[0] if time_sec else 0.0
            t_max = time_sec[-1] if time_sec else 10.0
            x_right = max(t_min + 10.0, t_max)
            self.ax1.set_xlim(t_min, x_right)
            self.ax2.set_xlim(t_min, x_right)

            self.ax1_twin.relim()
            self.ax1_twin.autoscale_view(scalex=False, scaley=True)

            max_cur = max(self.current_data) if self.current_data else 0.0
            self.ax2.set_ylim(-0.2, max(5.0, max_cur * 1.15))

            max_rpm = max(self.rpm_data) if self.rpm_data else 0.0
            self.ax2_twin.set_ylim(-50, max(2000.0, max_rpm * 1.15))

            self.ax2_twin.relim()
            self.ax2_twin.autoscale_view(scalex=False, scaley=True)

        # -------------------------------------------------------------
        # MODE 2: AeroSandbox BEMT Aerodynamic Validation (RPM Scatter)
        # Compares empirical measured data against theoretical BEMT curve
        # -------------------------------------------------------------
        else:
            plot_rpm = self.bemt_history_rpm if self.bemt_history_rpm else [r for r in self.rpm_data if r > 50]
            plot_th = self.bemt_history_thrust if self.bemt_history_thrust else [t for r, t in zip(self.rpm_data, self.thrust_data) if r > 50]
            plot_tq = self.bemt_history_torque if self.bemt_history_torque else [q for r, q in zip(self.rpm_data, self.torque_data) if r > 50]

            self.scatter_bemt_thrust.set_data(plot_rpm, plot_th)
            self.scatter_bemt_torque.set_data(plot_rpm, plot_tq)

            # Theoretical BEMT Model Curve
            max_meas_rpm = max(plot_rpm) if plot_rpm else 5000.0
            upper_rpm = max(max_meas_rpm * 1.15, 6000.0)
            theory_rpm = np.linspace(1000, upper_rpm, 100)
            n_rev_sec = theory_rpm / 60.0

            th_newtons = self.ct_proxy * RHO_AIR * (n_rev_sec**2) * (self.prop_diameter_m**4)
            th_grams = th_newtons * (1000.0 / 9.80665)
            tq_nm = (self.cp_proxy * RHO_AIR * (n_rev_sec**2) * (self.prop_diameter_m**5)) / (2.0 * np.pi)

            self.line_bemt_theory_thrust.set_data(theory_rpm, th_grams)
            self.line_bemt_theory_torque.set_data(theory_rpm, tq_nm)

            # Minimum 100g span for Thrust
            max_th_meas = max(plot_th) if plot_th else 0.0
            max_th_model = max(th_grams) if len(th_grams) else 100.0
            self.ax1.set_ylim(-2.0, max(100.0, max(max_th_meas, max_th_model) * 1.15))

            self.ax1.set_xlim(0, upper_rpm)
            self.ax1_twin.relim()
            self.ax1_twin.autoscale_view(scalex=False, scaley=True)

            self.line_bemt_current.set_data(time_sec, self.current_data)
            max_cur = max(self.current_data) if self.current_data else 0.0
            self.ax2.set_ylim(-0.2, max(5.0, max_cur * 1.15))

            t_min = time_sec[0] if time_sec else 0.0
            t_max = time_sec[-1] if time_sec else 10.0
            self.ax2.set_xlim(t_min, max(t_min + 10.0, t_max))

        self.canvas.draw_idle()
        self.plot_dirty = False

if __name__ == "__main__":
    root = tk.Tk()
    app = RCBenchmarkCloneApp(root)
    root.mainloop()