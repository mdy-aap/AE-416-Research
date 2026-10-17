#include <ESP32Servo.h>
#include <Preferences.h>

Preferences preferences;

// =================================================================
// HARDWARE TOGGLES & POLARITY
// =================================================================
#define ENABLE_THRUST  true   // 5kg Thrust Load Cell (DT=4, SCK=5)
#define ENABLE_TORQUE1 true   // 1kg Torque Cell A (DT=16, SCK=17)
#define ENABLE_TORQUE2 true   // 1kg Torque Cell B (DT=6, SCK=7)
bool reverse_thrust  = true;  // Reverse thrust sign (true = inverted)
bool reverse_current = false; // Reverse current sign (true if forward current drops sensor voltage)

// =================================================================
// DEDICATED PIN DEFINITIONS (ESP32-S3 Left Header - USB at bottom)
// =================================================================
// OPTION 2: Software 3.3V Power Rail for 3rd HX711 Amp
#define HX711_PWR_PIN     3   // Pin 15 from top (GPIO 3) -> Outputs clean 3.3V
#define HX711_PWR_PIN_ALT 8   // Pin 12 from top (GPIO 8) -> Also outputs clean 3.3V

// Thrust Load Cell (5kg)
#define THRUST_DOUT_PIN   4   // DT  - Pin 4 from top (GPIO 4)
#define THRUST_SCK_PIN    5   // SCK - Pin 5 from top (GPIO 5)

// Torque Load Cell #1 (1kg)
#define TORQUE1_DOUT_PIN  16  // DT  - Pin 9 from top (GPIO 16)
#define TORQUE1_SCK_PIN   17  // SCK - Pin 10 from top (GPIO 17)

// Torque Load Cell #2 (1kg)
#define TORQUE2_DOUT_PIN  6   // DT  - Pin 6 from top (GPIO 6)
#define TORQUE2_SCK_PIN   7   // SCK - Pin 7 from top (GPIO 7)

// ESC PWM Output & Signal Ground
#define ESC_PWM_PIN       18  // PWM - Pin 11 from top (GPIO 18)
#define ESC_GND_PIN       46  // GND - Pin 16 from top (GPIO 46) -> Software 0V Ground
#define ESC_GND_PIN_ALT   20  // GND - Pin 14 from top (GPIO 20) -> Also Software 0V Ground

// TCRT5000 Optical RPM Sensor
#define RPM_PIN           19  // DO (Digital Out) - Pin 13 from top (GPIO 19)
int pulses_per_rev = 1;       // 1 for 1 reflective tape strip, 2 for 2-blade prop

// HW-525 ACS758 50A Current Sensor (Left Header - Untaken Pins 17 to 20 from top)
#define CURRENT_PIN       9       // ADC1_CH8 - Pin 17 from top (GPIO 9) -> Primary OUT2 (Buffered)
#define CURRENT_PIN_ALT   10      // ADC1_CH9 - Pin 18 from top (GPIO 10) -> Alternate OUT1 (Direct)
#define ACS_PWR_PIN       11      // Pin 19 from top (GPIO 11) -> Outputs clean 3.3V VCC
#define ACS_GND_PIN       12      // Pin 20 from top (GPIO 12) -> Holds clean 0.0V Ground (or use Pin 22 GND)

float current_zero_v   = 2.50f;   // Auto-tared at resting zero throttle
float current_sens_v_a = 0.040f;  // Ratiometric sensitivity: 40mV/A at 5V, 26.4mV/A at 3.3V
float last_current_A   = 0.0f;

volatile unsigned long last_pulse_us = 0;
volatile unsigned long pulse_interval_us = 0;
volatile unsigned long total_pulse_count = 0;
unsigned long prev_pulse_count = 0;
float current_rpm = 0.0f;

void IRAM_ATTR rpm_isr() {
  unsigned long now = micros();
  unsigned long interval = now - last_pulse_us;
  // 1200µs debounce filter (~50,000 RPM max physical limit, filters optical tape chatter)
  if (interval > 1200) {
    pulse_interval_us = interval;
    last_pulse_us = now;
    total_pulse_count++;
  }
}

// --- Calibration Factors (Loaded from Flash NVS) ---
// Defaults: 5kg cell ~420 counts/g, 1kg cell ~2100 counts/g
float thrust_cal_factor  = 420.0f;
float torque1_cal_factor = 2100.0f;
float torque2_cal_factor = 2100.0f;
float torque_arm_meters  = 0.040f; // 40mm physical lever arm radius
int   torque_mode        = 0;      // 0 = Diff (T1 - T2), 1 = Sum (T1 + T2), 2 = T1 only, 3 = T2 only

Servo esc;

// Tare offsets
long thrust_offset  = 0;
long torque1_offset = 0;
long torque2_offset = 0;

// Live smoothed values
float last_thrust_g  = 0.0;
float last_torque1_g = 0.0;
float last_torque2_g = 0.0;
float last_torque_Nm = 0.0;
float net_torque_g   = 0.0;

long last_raw_thrust  = 0;
long last_raw_torque1 = 0;
long last_raw_torque2 = 0;

unsigned long last_sample_millis = 0;
unsigned long last_diag_millis   = 0;
const unsigned long SAMPLE_INTERVAL_MS = 100; // 10Hz telemetry rate

static portMUX_TYPE hx711_mux = portMUX_INITIALIZER_UNLOCKED;

// =================================================================
// DIRECT HARDWARE 24-BIT BIT-BANG READER
// =================================================================
long readHX711Channel(int dout_pin, int sck_pin) {
  portENTER_CRITICAL(&hx711_mux);

  unsigned long raw_data = 0;
  for (int i = 0; i < 24; i++) {
    digitalWrite(sck_pin, HIGH);
    delayMicroseconds(2);
    raw_data = (raw_data << 1) | (digitalRead(dout_pin) ? 1 : 0);
    digitalWrite(sck_pin, LOW);
    delayMicroseconds(2);
  }

  // 25th pulse: Channel A, Gain 128 (pulls DOUT back HIGH to begin next conversion)
  digitalWrite(sck_pin, HIGH);
  delayMicroseconds(2);
  digitalWrite(sck_pin, LOW);
  delayMicroseconds(2);

  portEXIT_CRITICAL(&hx711_mux);

  // Sign-extend 24-bit two's complement to 32-bit signed long
  if (raw_data & 0x800000) {
    raw_data |= 0xFF000000;
  }
  return (long)raw_data;
}

// Reset an HX711 chip via SCK (>60us pulse)
void resetHX711(int sck_pin) {
  digitalWrite(sck_pin, HIGH);
  delayMicroseconds(80);
  digitalWrite(sck_pin, LOW);
  delay(10);
}

// Wait for a channel to become ready
bool waitForReady(int dout_pin, uint32_t timeout_ms = 250) {
  unsigned long start = millis();
  while (digitalRead(dout_pin) == HIGH) {
    if (millis() - start >= timeout_ms) {
      return false;
    }
    delay(1);
  }
  return true;
}

// Tare all enabled load cells
void tareAll(int samples = 10) {
  Serial.println("\n[CMD] Taring all enabled load cells... Please keep rig still...");

  if (ENABLE_THRUST) {
    long sum = 0; int n = 0;
    for (int i = 0; i < samples; i++) {
      if (waitForReady(THRUST_DOUT_PIN, 200)) { sum += readHX711Channel(THRUST_DOUT_PIN, THRUST_SCK_PIN); n++; }
      delay(5);
    }
    if (n > 0) thrust_offset = sum / n;
    Serial.printf("  --> Thrust Offset:  %ld\n", thrust_offset);
  }

  if (ENABLE_TORQUE1) {
    long sum = 0; int n = 0;
    for (int i = 0; i < samples; i++) {
      if (waitForReady(TORQUE1_DOUT_PIN, 200)) { sum += readHX711Channel(TORQUE1_DOUT_PIN, TORQUE1_SCK_PIN); n++; }
      delay(5);
    }
    if (n > 0) torque1_offset = sum / n;
    Serial.printf("  --> Torque1 Offset: %ld\n", torque1_offset);
  }

  if (ENABLE_TORQUE2) {
    long sum = 0; int n = 0;
    for (int i = 0; i < samples; i++) {
      if (waitForReady(TORQUE2_DOUT_PIN, 200)) { sum += readHX711Channel(TORQUE2_DOUT_PIN, TORQUE2_SCK_PIN); n++; }
      delay(5);
    }
    if (n > 0) torque2_offset = sum / n;
    Serial.printf("  --> Torque2 Offset: %ld\n", torque2_offset);
  }

  // Auto-tare HW-525 ACS758 Current Sensor zero baseline & calibrate ratiometric sensitivity
  // Check GPIO 9 (OUT2) first; if unconnected (<0.3V), check GPIO 10 (OUT1)
  uint32_t cur_sum_mv = 0;
  for (int i = 0; i < 25; i++) {
    cur_sum_mv += analogReadMilliVolts(CURRENT_PIN);
    delay(4);
  }
  float detected_zero_v = (cur_sum_mv / 25.0f) / 1000.0f;
  if (detected_zero_v < 0.3f) {
    cur_sum_mv = 0;
    for (int i = 0; i < 25; i++) {
      cur_sum_mv += analogReadMilliVolts(CURRENT_PIN_ALT);
      delay(4);
    }
    detected_zero_v = (cur_sum_mv / 25.0f) / 1000.0f;
  }
  current_zero_v = (detected_zero_v > 0.4f) ? detected_zero_v : 2.50f;

  // Ratiometric sensitivity: Sens = 0.040 * (Vcc / 5.0) = 0.016 * Vzero
  if (current_zero_v > 0.8f) {
    current_sens_v_a = 0.016f * current_zero_v;
  } else {
    current_sens_v_a = 0.040f;
  }
  last_current_A = 0.0f;
  Serial.printf("  --> Current Zero V:  %.3f V (Calculated Sens: %.4f V/A)\n", current_zero_v, current_sens_v_a);

  Serial.println("[CMD] TARE COMPLETE!\n");
}

// =================================================================
// FLASH CALIBRATION STORAGE (NVS PREFERENCES)
// =================================================================
void saveCalibration() {
  preferences.begin("daq_cal", false);
  preferences.putFloat("thrust_cal", thrust_cal_factor);
  preferences.putFloat("tq1_cal", torque1_cal_factor);
  preferences.putFloat("tq2_cal", torque2_cal_factor);
  preferences.putFloat("arm_m", torque_arm_meters);
  preferences.putInt("tq_mode", torque_mode);
  preferences.end();
  Serial.println("[NVS] Calibration factors, lever arm & torque mode saved to permanent Flash memory!");
}

void loadCalibration() {
  preferences.begin("daq_cal", false);
  thrust_cal_factor  = preferences.getFloat("thrust_cal", 420.0f);
  torque1_cal_factor = preferences.getFloat("tq1_cal", 2100.0f);
  torque2_cal_factor = preferences.getFloat("tq2_cal", 2100.0f);
  torque_arm_meters  = preferences.getFloat("arm_m", 0.040f);
  torque_mode        = preferences.getInt("tq_mode", 0);
  preferences.end();
  const char* mode_names[] = {"Diff (T1-T2)", "Sum (T1+T2)", "T1 Only", "T2 Only"};
  Serial.printf("[NVS] Loaded Cal Factors -> Thrust: %.2f | TQ1: %.2f | TQ2: %.2f | Arm: %.4f m (%.1f mm) | Mode: %s\n",
                thrust_cal_factor, torque1_cal_factor, torque2_cal_factor, torque_arm_meters, torque_arm_meters * 1000.0f, mode_names[constrain(torque_mode, 0, 3)]);
}

// Calibrate a single channel with a known weight (in grams)
void calibrateChannel(int channel, float known_grams) {
  if (known_grams <= 0.0f) {
    Serial.println("[CAL ERROR] Calibration weight must be greater than 0 grams!");
    return;
  }
  Serial.printf("[CAL] Calibrating Channel %d with %.1f grams... Please keep rig steady...\n", channel, known_grams);
  
  long sum = 0;
  int n = 0;
  int dout = (channel == 0) ? THRUST_DOUT_PIN : ((channel == 1) ? TORQUE1_DOUT_PIN : TORQUE2_DOUT_PIN);
  int sck  = (channel == 0) ? THRUST_SCK_PIN  : ((channel == 1) ? TORQUE1_SCK_PIN  : TORQUE2_SCK_PIN);
  long offset = (channel == 0) ? thrust_offset : ((channel == 1) ? torque1_offset : torque2_offset);

  for (int i = 0; i < 20; i++) {
    if (waitForReady(dout, 250)) {
      sum += readHX711Channel(dout, sck);
      n++;
    }
    delay(10);
  }

  if (n < 5) {
    Serial.println("[CAL ERROR] Sensor timeout! Could not read stable samples.");
    return;
  }

  long raw_avg = sum / n;
  long delta = raw_avg - offset;
  if (abs(delta) < 500) {
    Serial.printf("[CAL WARNING] Small count difference (%ld counts). Ensure weight is resting on sensor!\n", delta);
  }

  float new_factor = (float)delta / known_grams;
  if (channel == 0) {
    if (reverse_thrust) new_factor = -new_factor;
    thrust_cal_factor = new_factor;
    Serial.printf("[CAL SUCCESS] Thrust calibrated! New Factor = %.2f counts/g\n", thrust_cal_factor);
  } else if (channel == 1) {
    torque1_cal_factor = new_factor;
    Serial.printf("[CAL SUCCESS] Torque 1 calibrated! New Factor = %.2f counts/g\n", torque1_cal_factor);
  } else if (channel == 2) {
    torque2_cal_factor = new_factor;
    Serial.printf("[CAL SUCCESS] Torque 2 calibrated! New Factor = %.2f counts/g\n", torque2_cal_factor);
  }

  saveCalibration();
}

// =================================================================
// SERIAL ESC CONTROL & COMMAND HANDLER
// =================================================================
String rxBuffer = "";

void handleIncomingSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      rxBuffer.trim();
      if (rxBuffer.startsWith("CTRL,")) {
        int pwm_val = rxBuffer.substring(5).toInt();
        pwm_val = constrain(pwm_val, 1000, 2000);
        esc.writeMicroseconds(pwm_val);
      } else if (rxBuffer.equalsIgnoreCase("TARE") || rxBuffer.equalsIgnoreCase("t")) {
        tareAll(15);
      } else if (rxBuffer.startsWith("CAL,THRUST,") || rxBuffer.startsWith("CAL,T,")) {
        int idx = rxBuffer.lastIndexOf(',');
        float grams = rxBuffer.substring(idx + 1).toFloat();
        calibrateChannel(0, grams);
      } else if (rxBuffer.startsWith("CAL,TQ1,")) {
        int idx = rxBuffer.lastIndexOf(',');
        float grams = rxBuffer.substring(idx + 1).toFloat();
        calibrateChannel(1, grams);
      } else if (rxBuffer.startsWith("CAL,TQ2,")) {
        int idx = rxBuffer.lastIndexOf(',');
        float grams = rxBuffer.substring(idx + 1).toFloat();
        calibrateChannel(2, grams);
      } else if (rxBuffer.startsWith("ARM,")) {
        float arm_val = rxBuffer.substring(4).toFloat();
        if (arm_val > 1.0f) {
          torque_arm_meters = arm_val / 1000.0f;
        } else if (arm_val > 0.001f) {
          torque_arm_meters = arm_val;
        } else {
          Serial.println("[ARM ERROR] Invalid arm length! Must be > 0.");
        }
        if (arm_val > 0.001f) {
          saveCalibration();
          Serial.printf("[ARM] Lever arm set to %.1f mm (%.4f m) and saved to Flash!\n",
                        torque_arm_meters * 1000.0f, torque_arm_meters);
        }
      } else if (rxBuffer.startsWith("TQMODE,")) {
        String mode_str = rxBuffer.substring(7);
        mode_str.toUpperCase();
        if (mode_str.startsWith("SUM") || mode_str.startsWith("ADD") || mode_str == "1") {
          torque_mode = 1;
        } else if (mode_str.startsWith("DIFF") || mode_str.startsWith("SUB") || mode_str == "0") {
          torque_mode = 0;
        } else if (mode_str.startsWith("T1") || mode_str == "2") {
          torque_mode = 2;
        } else if (mode_str.startsWith("T2") || mode_str == "3") {
          torque_mode = 3;
        }
        saveCalibration();
        const char* mode_names[] = {"Difference (T1 - T2)", "Parallel Sum (T1 + T2)", "Cell 1 Only", "Cell 2 Only"};
        Serial.printf("[TQMODE] Torque Dual-Cell Mode set to: %s\n", mode_names[constrain(torque_mode, 0, 3)]);
      } else if (rxBuffer.equalsIgnoreCase("CAL,SHOW") || rxBuffer.equalsIgnoreCase("CAL")) {
        const char* mode_names[] = {"Difference (T1 - T2)", "Parallel Sum (T1 + T2)", "Cell 1 Only", "Cell 2 Only"};
        Serial.println("\n--- CALIBRATION STATUS ---");
        Serial.printf("Thrust Factor:   %.2f counts/g (Offset: %ld, Live: %.1f g)\n", thrust_cal_factor, thrust_offset, last_thrust_g);
        Serial.printf("Torque 1 Factor: %.2f counts/g (Offset: %ld, Live: %.1f g)\n", torque1_cal_factor, torque1_offset, last_torque1_g);
        Serial.printf("Torque 2 Factor: %.2f counts/g (Offset: %ld, Live: %.1f g)\n", torque2_cal_factor, torque2_offset, last_torque2_g);
        Serial.printf("Torque Arm:      %.1f mm (%.4f m)\n", torque_arm_meters * 1000.0f, torque_arm_meters);
        Serial.printf("Torque Mode:     %s (Net: %.1f g -> %.5f Nm)\n", mode_names[constrain(torque_mode, 0, 3)], net_torque_g, last_torque_Nm);
        Serial.println("--------------------------\n");
      } else if (rxBuffer.startsWith("PULSES,")) {
        pulses_per_rev = rxBuffer.substring(7).toInt();
        if (pulses_per_rev < 1) pulses_per_rev = 1;
        Serial.printf("[RPM] Pulses per revolution set to %d\n", pulses_per_rev);
      } else if (rxBuffer.equalsIgnoreCase("REV_THRUST") || rxBuffer.equalsIgnoreCase("INVERT_THRUST")) {
        reverse_thrust = !reverse_thrust;
        Serial.printf("[THRUST] Thrust polarity set to: %s\n", reverse_thrust ? "REVERSED (INVERTED)" : "NORMAL");
      } else if (rxBuffer.equalsIgnoreCase("REV_CURRENT") || rxBuffer.equalsIgnoreCase("INVERT_CURRENT")) {
        reverse_current = !reverse_current;
        Serial.printf("[CURRENT] Current polarity set to: %s\n", reverse_current ? "INVERTED (V_fall)" : "NORMAL (V_rise)");
      } else if (rxBuffer.equalsIgnoreCase("CAL,RESET")) {
        thrust_cal_factor = 420.0f;
        torque1_cal_factor = 2100.0f;
        torque2_cal_factor = 2100.0f;
        torque_arm_meters = 0.040f;
        saveCalibration();
        Serial.println("[CAL] Reset to default calibration factors (Thrust: 420, TQ1: 2100, TQ2: 2100, Arm: 40mm).");
      }
      rxBuffer = "";
    } else if (c != '\r') {
      rxBuffer += c;
      if (rxBuffer.length() > 64) rxBuffer = "";
    }
  }
}

// =================================================================
// SETUP
// =================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Software 3.3V Power Rails & Software 0V Ground
  pinMode(HX711_PWR_PIN, OUTPUT);
  digitalWrite(HX711_PWR_PIN, HIGH); // Outputs clean 3.3V on GPIO 3
  pinMode(HX711_PWR_PIN_ALT, OUTPUT);
  digitalWrite(HX711_PWR_PIN_ALT, HIGH); // Outputs clean 3.3V on GPIO 8
  
  pinMode(ESC_GND_PIN, OUTPUT);
  digitalWrite(ESC_GND_PIN, LOW); // Holds clean 0.0V (Ground) on GPIO 46 (Pin 16)
  pinMode(ESC_GND_PIN_ALT, OUTPUT);
  digitalWrite(ESC_GND_PIN_ALT, LOW); // Holds clean 0.0V (Ground) on GPIO 20 (Pin 14)

  // Dedicated Software Power Rails for ACS758 Current Sensor (Left Header Pins 19 & 20)
  pinMode(ACS_PWR_PIN, OUTPUT);
  digitalWrite(ACS_PWR_PIN, HIGH); // Outputs clean 3.3V VCC on GPIO 11 (Pin 19 from top)
  pinMode(ACS_GND_PIN, OUTPUT);
  digitalWrite(ACS_GND_PIN, LOW);  // Holds clean 0.0V Ground on GPIO 12 (Pin 20 from top)
  delay(100); // Allow rails to stabilize

  Serial.println("\n==============================================");
  Serial.println("  ESP32-S3 3-AXIS LOAD CELL DAQ RIG FIRMWARE  ");
  Serial.println("==============================================");
  Serial.printf("3.3V Power Pins: GPIO %d & GPIO %d (HX711)\n", HX711_PWR_PIN, HX711_PWR_PIN_ALT);
  Serial.printf("ACS758 Power:   VCC=GPIO %d (Pin 19), GND=GPIO %d (Pin 20)\n", ACS_PWR_PIN, ACS_GND_PIN);
  Serial.printf("ACS758 Signal:  OUT2=GPIO %d (Pin 17), OUT1=GPIO %d (Pin 18)\n", CURRENT_PIN, CURRENT_PIN_ALT);
  Serial.printf("Software GNDs:  GPIO %d & GPIO %d (ESC GND)\n", ESC_GND_PIN, ESC_GND_PIN_ALT);
  Serial.printf("Thrust (5kg):   DT=GPIO %d, SCK=GPIO %d (Enabled=%d)\n", THRUST_DOUT_PIN, THRUST_SCK_PIN, ENABLE_THRUST);
  Serial.printf("Torque 1 (1kg): DT=GPIO %d, SCK=GPIO %d (Enabled=%d)\n", TORQUE1_DOUT_PIN, TORQUE1_SCK_PIN, ENABLE_TORQUE1);
  Serial.printf("Torque 2 (1kg): DT=GPIO %d, SCK=GPIO %d (Enabled=%d)\n", TORQUE2_DOUT_PIN, TORQUE2_SCK_PIN, ENABLE_TORQUE2);
  Serial.printf("RPM Sensor:     DO=GPIO %d (TCRT5000 IR Interrupt)\n", RPM_PIN);
  Serial.printf("ESC Pins:       PWM=GPIO %d, GND=GPIO %d (or GPIO %d)\n", ESC_PWM_PIN, ESC_GND_PIN, ESC_GND_PIN_ALT);
  Serial.println("==============================================");

  // Load saved calibration factors from NVS Flash
  loadCalibration();

  // Configure HW-525 ACS758 Current Sensor ADCs (12-bit, 11dB attenuation)
  analogReadResolution(12);
  analogSetPinAttenuation(CURRENT_PIN, ADC_11db);
  analogSetPinAttenuation(CURRENT_PIN_ALT, ADC_11db);

  // Arm ESC with neutral throttle (1000us)
  esc.attach(ESC_PWM_PIN, 1000, 2000);
  esc.writeMicroseconds(1000);

  // Configure TCRT5000 RPM sensor interrupt
  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpm_isr, FALLING);

  // Configure pin modes & reset chips
  if (ENABLE_THRUST) {
    pinMode(THRUST_DOUT_PIN, INPUT_PULLUP);
    pinMode(THRUST_SCK_PIN, OUTPUT);
    resetHX711(THRUST_SCK_PIN);
  }

  if (ENABLE_TORQUE1) {
    pinMode(TORQUE1_DOUT_PIN, INPUT_PULLUP);
    pinMode(TORQUE1_SCK_PIN, OUTPUT);
    resetHX711(TORQUE1_SCK_PIN);
  }

  if (ENABLE_TORQUE2) {
    pinMode(TORQUE2_DOUT_PIN, INPUT_PULLUP);
    pinMode(TORQUE2_SCK_PIN, OUTPUT);
    resetHX711(TORQUE2_SCK_PIN);
  }

  delay(200);

  // Startup Tare
  tareAll(10);

  Serial.println("[INIT] Ready. Streaming $RIG telemetry packets at 115200 baud...\n");
}

// =================================================================
// MAIN LOOP
// =================================================================
void loop() {
  handleIncomingSerial();

  // Non-blocking channel polling: read whenever a channel finishes conversion (DOUT goes LOW)
  if (ENABLE_THRUST && digitalRead(THRUST_DOUT_PIN) == LOW) {
    last_raw_thrust = readHX711Channel(THRUST_DOUT_PIN, THRUST_SCK_PIN);
    float raw_val = (float)(last_raw_thrust - thrust_offset) / thrust_cal_factor;
    last_thrust_g = reverse_thrust ? -raw_val : raw_val;
  }

  if (ENABLE_TORQUE1 && digitalRead(TORQUE1_DOUT_PIN) == LOW) {
    last_raw_torque1 = readHX711Channel(TORQUE1_DOUT_PIN, TORQUE1_SCK_PIN);
    last_torque1_g = (float)(last_raw_torque1 - torque1_offset) / torque1_cal_factor;
  }

  if (ENABLE_TORQUE2 && digitalRead(TORQUE2_DOUT_PIN) == LOW) {
    last_raw_torque2 = readHX711Channel(TORQUE2_DOUT_PIN, TORQUE2_SCK_PIN);
    last_torque2_g = (float)(last_raw_torque2 - torque2_offset) / torque2_cal_factor;
  }

  // Periodic Telemetry Stream (10Hz)
  unsigned long current_millis = millis();
  if (current_millis - last_sample_millis >= SAMPLE_INTERVAL_MS) {
    last_sample_millis = current_millis;

    // 1. Calculate live RPM with Dual-Stage Optical Window Filter (kills tape chatter)
    unsigned long now_us = micros();
    unsigned long count_now = total_pulse_count;
    unsigned long pulses_in_window = count_now - prev_pulse_count;
    prev_pulse_count = count_now;

    if (now_us - last_pulse_us > 350000 || pulses_in_window == 0) {
      current_rpm = 0.0f; // Motor stopped (<170 RPM)
    } else {
      float instant_rpm = (pulse_interval_us > 0) ? ((60000000.0f / pulse_interval_us) / (float)pulses_per_rev) : 0.0f;
      // Window RPM: calculated from pulses accumulated in this 100ms sample window
      float window_rpm = (pulses_in_window * 60000.0f) / ((float)SAMPLE_INTERVAL_MS * (float)pulses_per_rev);

      // Rejection logic for optical tape lingering / chatter:
      // If instant RPM diverges wildly (>2.2x) from window pulse count, instant RPM was a chattered sub-pulse!
      if (window_rpm > 100.0f && instant_rpm > window_rpm * 2.2f) {
        current_rpm = window_rpm;
      } else if (window_rpm > 100.0f && instant_rpm < window_rpm * 0.45f) {
        current_rpm = window_rpm;
      } else {
        current_rpm = instant_rpm;
      }
    }

    // 2. Read HW-525 ACS758 50A Current Sensor (16x oversampled ADC read with settling delay)
    // Supports OUT2 on GPIO 9 (primary) or OUT1 on GPIO 10 (alternate)
    uint32_t mv_acc = 0;
    for (int i = 0; i < 16; i++) {
      mv_acc += analogReadMilliVolts(CURRENT_PIN);
      delayMicroseconds(100); // 100µs ADC S&H settling delay prevents droop
    }
    float v_cur = (mv_acc / 16.0f) / 1000.0f;
    if (v_cur < 0.25f) {
      uint32_t mv_acc_alt = 0;
      for (int i = 0; i < 16; i++) {
        mv_acc_alt += analogReadMilliVolts(CURRENT_PIN_ALT);
        delayMicroseconds(100);
      }
      float v_cur_alt = (mv_acc_alt / 16.0f) / 1000.0f;
      if (v_cur_alt > 0.4f) {
        v_cur = v_cur_alt;
      }
    }

    // Auto-detect current polarity if motor is running and producing thrust:
    // If Vcur drops while motor is running, reverse_current must be true!
    if (current_rpm > 500.0f && last_thrust_g > 8.0f) {
      if (!reverse_current && (v_cur - current_zero_v) < -0.015f) {
        reverse_current = true;
      } else if (reverse_current && (current_zero_v - v_cur) < -0.015f) {
        reverse_current = false;
      }
    }

    float delta_v = reverse_current ? (current_zero_v - v_cur) : (v_cur - current_zero_v);
    float calc_current = delta_v / current_sens_v_a;

    // Resting baseline auto-tracking:
    // When motor is idle (<50 RPM), current is locked to 0.00A and baseline gently tracks resting voltage
    if (current_rpm < 50.0f) {
      if (fabs(v_cur - current_zero_v) < 0.25f) {
        current_zero_v = (current_zero_v * 0.96f) + (v_cur * 0.04f);
      }
      calc_current = 0.0f;
    } else {
      if (calc_current < 0.0f) calc_current = 0.0f;
    }

    // Exponential moving average filter (smoothes motor commutation switching spikes)
    last_current_A = (last_current_A * 0.6f) + (calc_current * 0.4f);

    // 3. Calculate net torque in N·m from the torque cells based on configured mode
    net_torque_g = 0.0f;
    if (torque_mode == 1) {
      net_torque_g = last_torque1_g + last_torque2_g; // Parallel Sum (T1 + T2)
    } else if (torque_mode == 2) {
      net_torque_g = last_torque1_g;                   // Cell 1 Only
    } else if (torque_mode == 3) {
      net_torque_g = last_torque2_g;                   // Cell 2 Only
    } else {
      net_torque_g = ENABLE_TORQUE2 ? (last_torque1_g - last_torque2_g) : last_torque1_g; // Difference (T1 - T2)
    }
    last_torque_Nm = (net_torque_g / 1000.0f) * 9.80665f * torque_arm_meters;

    // 4. Single formatted telemetry packet for Python GUI (gui_logger.py)
    char buf[128];
    snprintf(buf, sizeof(buf), "$RIG,%lu,%.2f,%.5f,%.0f,%.2f,%.2f,%.2f\n", 
             current_millis, last_thrust_g, last_torque_Nm, current_rpm, last_current_A, last_torque1_g, last_torque2_g);
    Serial.print(buf);

    if (current_millis - last_diag_millis > 1500) {
      last_diag_millis = current_millis;
      const char* mode_short[] = {"DIFF", "SUM", "T1", "T2"};
      Serial.printf("[DEBUG] Thrust: %6.1f g | TQ: %.4f Nm (T1: %5.1f g, T2: %5.1f g, %s) | RPM: %5.0f | Current: %5.2f A\n",
                    last_thrust_g, last_torque_Nm, last_torque1_g, last_torque2_g, mode_short[constrain(torque_mode, 0, 3)], current_rpm, last_current_A);
    }
  }
}


