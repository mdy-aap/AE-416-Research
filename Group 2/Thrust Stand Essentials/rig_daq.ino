#include "HX711.h"

// --- Pin Definitions ---
#define THRUST_DOUT_PIN  4   // HX711 #1 Data Pin (5kg cell)
#define TORQUE_DOUT_PIN  5   // HX711 #2 Data Pin (1kg cell)
#define SHARED_CLK_PIN   6   // Dual HX711 Shared Clock Pin
#define RPM_PIN          7   // TCRT5000 DO Pin (Interrupt capable)
#define CURRENT_PIN      1   // ACS758 Analog Out Pin (ESP32-S3 ADC1_CH0)

// --- Calibration Variables ---
// Adjust these factors after performing your dead-weight calibrations
const float THRUST_CALIBRATION_FACTOR = 420.0; 
const float TORQUE_CALIBRATION_FACTOR = 1250.0; 
const float TORQUE_ARM_METERS = 0.040; // 40mm physical lever arm

// --- Sensor Objects ---
HX711 thrust_scale;
HX711 torque_scale;

// --- Interrupt & Telemetry Variables ---
volatile unsigned long rpm_pulse_count = 0;
unsigned long last_millis = 0;
const unsigned long SAMPLE_INTERVAL_MS = 100; // 10 Hz output rate

// Hardware Interrupt Service Routine (ISR) for RPM
void IRAM_ATTR countRpmPulse() {
  rpm_pulse_count++;
}

void setup() {
  Serial.begin(115200);
  
  // Initialize Load Cells sharing the same clock pin
  thrust_scale.begin(THRUST_DOUT_PIN, SHARED_CLK_PIN);
  torque_scale.begin(TORQUE_DOUT_PIN, SHARED_CLK_PIN);
  
  // Apply Calibration
  thrust_scale.set_scale(THRUST_CALIBRATION_FACTOR);
  torque_scale.set_scale(TORQUE_CALIBRATION_FACTOR);
  
  // Set baselines (Ensure rig is completely unloaded on boot!)
  thrust_scale.tare();
  torque_scale.tare();
  
  // Configure RPM IR Interrupt Pin
  pinMode(RPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), countRpmPulse, RISING);
  
  // Configure Analog Pin for ACS758 Current Sensor
  analogReadResolution(12); // ESP32-S3 12-bit ADC (0-4095 range)
}

void loop() {
  unsigned long current_millis = millis();
  
  if (current_millis - last_millis >= SAMPLE_INTERVAL_MS) {
    float time_elapsed_sec = (current_millis - last_millis) / 1000.0;
    
    // 1. Safely snapshot and reset the high-speed RPM pulse counter
    noInterrupts();
    unsigned long pulses = rpm_pulse_count;
    rpm_pulse_count = 0;
    interrupts();
    
    // Calculate RPM (Assuming 1 reflective tape marker on 1 blade element)
    float rpm = (pulses / time_elapsed_sec) * 60.0;
    
    // 2. Read Aerodynamic Load Cells (in Grams of force)
    float thrust_g = thrust_scale.get_units(1); // 1-sample non-blocking read
    float torque_g = torque_scale.get_units(1); 
    
    // Convert Force to Torque (Grams to Kilograms -> Newtons -> N*m)
    float torque_Nm = (torque_g / 1000.0) * 9.80665 * TORQUE_ARM_METERS;
    
    // 3. Read Electrical Current from ACS758
    int raw_adc = analogRead(CURRENT_PIN);
    float adc_voltage = (raw_adc / 4095.0) * 3.3;
    // ACS758-050B rests at VCC/2 (1.65V) at 0 Amps, outputs 40mV per Amp
    float current_A = (adc_voltage - 1.65) / 0.040; 
    if (current_A < 0.1) current_A = 0.0; // Filter out idle baseline float noise
    
    // 4. Output Data Stream via CSV Serial Formatting
    // Schema: Timestamp(ms), Thrust(g), Torque(Nm), RPM, Current(A)
    Serial.print(current_millis);
    Serial.print(",");
    Serial.print(thrust_g, 2);
    Serial.print(",");
    Serial.print(torque_Nm, 5);
    Serial.print(",");
    Serial.print(rpm, 0);
    Serial.print(",");
    Serial.println(current_A, 2);
    
    last_millis = current_millis;
  }
}