// =============================================================================
// ESP32-S3 ISOLATED LOAD CELL & HX711 TEST BENCH
// =============================================================================
// Hardware: ESP32-S3 + XFW-HX711 Amplifier + YZC-133 Load Cell
// Baud Rate: 115200 baud
// =============================================================================

// --- PIN DEFINITIONS ---
const int PIN_DT    = 16;  // Data pin (HX711 DT  -> ESP32 GPIO 16)
const int PIN_SCK   = 17;  // Clock pin (HX711 SCK -> ESP32 GPIO 17)
const int PIN_PROBE = 6;   // Voltmeter Probe wire (ESP32 GPIO 6)

// --- CALIBRATION ---
float cal_factor = 420.0;  // Standard for 5kg YZC-133 cell (counts per gram)
int32_t tare_offset = 0;
bool is_tared = false;

// --- READ RAW 24-BIT VALUE FROM HX711 ---
int32_t readHX711(bool &ready) {
  // Step 1: Wait for DOUT to go LOW (ready flag)
  unsigned long start = millis();
  while (digitalRead(PIN_DT) == HIGH) {
    if (millis() - start > 150) {
      ready = false;
      return 0; // Timed out (HX711 not converting or disconnected)
    }
  }
  ready = true;

  // Step 2: Shift in 24 bits with 10us pulses (protected from RTOS context switches)
  uint32_t raw = 0;
  noInterrupts();
  for (int i = 0; i < 24; i++) {
    digitalWrite(PIN_SCK, HIGH);
    delayMicroseconds(10);
    raw = (raw << 1) | digitalRead(PIN_DT);
    digitalWrite(PIN_SCK, LOW);
    delayMicroseconds(10);
  }

  // Step 3: 25th pulse sets Gain 128 on Channel A for the next conversion
  digitalWrite(PIN_SCK, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SCK, LOW);
  delayMicroseconds(10);
  interrupts();

  // Step 4: Sign-extend 24-bit two's complement to 32-bit signed integer
  if (raw & 0x800000) {
    raw |= 0xFF000000;
  }

  return (int32_t)raw;
}

// --- HARDWARE ELECTRICAL INTEGRITY CHECK ---
void runPinDiagnostics() {
  Serial.println("\n========================================");
  Serial.println("  ESP32 ALL-PIN AUTO-DISCOVERY SCANNER   ");
  Serial.println("========================================");
  
  // Specific check for configured pins 16 & 17
  pinMode(16, INPUT_PULLUP); delay(5); int pu16 = digitalRead(16);
  pinMode(16, INPUT_PULLDOWN); delay(5); int pd16 = digitalRead(16);
  pinMode(16, INPUT);

  pinMode(17, INPUT_PULLUP); delay(5); int pu17 = digitalRead(17);
  pinMode(17, INPUT_PULLDOWN); delay(5); int pd17 = digitalRead(17);
  pinMode(17, INPUT);

  Serial.printf("Configured DT (GPIO 16):  PullUp=%d, PullDown=%d -> %s\n", pu16, pd16, 
                (pu16==1 && pd16==1) ? "Driven HIGH" : (pu16==0 && pd16==0) ? "Held LOW (Active/GND)" : "FLOATING (No wire)");
  Serial.printf("Configured SCK (GPIO 17): PullUp=%d, PullDown=%d -> %s\n", pu17, pd17, 
                (pu17==1 && pd17==1) ? "Driven HIGH" : (pu17==0 && pd17==0) ? "Held LOW (Active/GND)" : "FLOATING (No wire)");

  Serial.println("\nScanning entire board for any connected wires...");
  const int scanPins[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 38, 39, 40, 41, 42, 45, 46, 47, 48};
  int foundCount = 0;
  for (int p : scanPins) {
    pinMode(p, INPUT_PULLUP);
    delay(2);
    int pu = digitalRead(p);
    pinMode(p, INPUT_PULLDOWN);
    delay(2);
    int pd = digitalRead(p);
    pinMode(p, INPUT);

    // If it's NOT floating (floating is pu=1, pd=0)
    if (!(pu == 1 && pd == 0)) {
      Serial.printf("  >>> [WIRE DETECTED] GPIO %d: PullUp=%d, PullDown=%d (%s)\n", 
                    p, pu, pd, (pu==0 && pd==0) ? "Held LOW / GND / HX711" : "Driven HIGH / VCC");
      foundCount++;
    }
  }
  if (foundCount == 0) {
    Serial.println("  --> No connected signals found on any scanned GPIO! Check if HX711 has Power (VCC & GND).");
  }
  Serial.println("========================================\n");
}

void tareScale() {
  Serial.print("Taring (averaging 10 samples)... ");
  int32_t sum = 0;
  int count = 0;
  for (int i = 0; i < 15 && count < 10; i++) {
    bool ready = false;
    int32_t val = readHX711(ready);
    if (ready) {
      sum += val;
      count++;
    }
    delay(50);
  }
  if (count > 0) {
    tare_offset = sum / count;
    is_tared = true;
    Serial.printf("DONE! Tare Offset = %ld\n", (long)tare_offset);
  } else {
    Serial.println("FAILED! HX711 not responding.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=======================================================");
  Serial.println("   ESP32-S3 LOAD CELL & HX711 STANDALONE TEST BENCH    ");
  Serial.println("=======================================================");
  Serial.println("Wiring Check:");
  Serial.println("  HX711 VCC -> ESP32 5V (or 5V0)");
  Serial.println("  HX711 GND -> ESP32 GND");
  Serial.println("  HX711 DT  -> ESP32 GPIO 16");
  Serial.println("  HX711 SCK -> ESP32 GPIO 17");
  Serial.println("  Probe     -> ESP32 GPIO 6 (touch to any pin to check mV)");
  Serial.println("Commands in Serial Monitor:");
  Serial.println("  't'       -> Tare (Zero the scale)");
  Serial.println("  'p'       -> Pin self-test");
  Serial.println("=======================================================\n");

  pinMode(PIN_SCK, OUTPUT);
  digitalWrite(PIN_SCK, LOW); // Hold LOW to keep HX711 awake

  pinMode(PIN_DT, INPUT);
  pinMode(PIN_PROBE, INPUT);

  runPinDiagnostics();

  // Attempt initial tare
  tareScale();
}

unsigned long last_print = 0;
String serial_cmd = "";

void loop() {
  // Handle user serial input
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serial_cmd.trim();
      if (serial_cmd.equalsIgnoreCase("t") || serial_cmd.equalsIgnoreCase("tare")) {
        tareScale();
      } else if (serial_cmd.equalsIgnoreCase("p")) {
        runPinDiagnostics();
      }
      serial_cmd = "";
    } else {
      serial_cmd += c;
    }
  }

  // Read HX711 every 200ms
  unsigned long now = millis();
  if (now - last_print >= 200) {
    last_print = now;

    bool ready = false;
    int32_t raw = readHX711(ready);
    uint32_t probe_mv = analogReadMilliVolts(PIN_PROBE);

    if (!ready) {
      Serial.printf("[STATUS] DT(GPIO %d) BUSY/NO DATA | Probe: %lu mV | (Check VCC/GND/SCK/DT)\n", 
                    PIN_DT, (unsigned long)probe_mv);
    } else {
      // Analyze the raw reading
      int32_t delta = raw - tare_offset;
      float grams = (float)delta / cal_factor;

      // Detect rail / saturation
      bool railed = (raw >= 8388600 || raw <= -8388600);

      Serial.printf("[ADC] Raw: %9ld | Offset: %9ld | Delta: %7ld | Force: %7.2f g | Probe: %4lu mV", 
                    (long)raw, (long)tare_offset, (long)delta, grams, (unsigned long)probe_mv);

      if (railed) {
        Serial.print("  <-- [ALERT: RAILED/SATURATED! Check White/Green wire connection!]");
      } else {
        Serial.print("  [OK: Active signal]");
      }
      Serial.println();
    }
  }
}
