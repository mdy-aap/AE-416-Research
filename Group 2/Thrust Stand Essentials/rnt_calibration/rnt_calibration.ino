// =============================================================================
// ESP32-S3 ROCK-SOLID LIVE DIGITAL SCALE (DIRECT HARDWARE BIT-BANG)
// =============================================================================
#include <Arduino.h>

// Hardware Pin Definitions (ESP32-S3 Left Header)
const int PIN_DOUT = 4;  // DT  -> Pin 4 from top
const int PIN_SCK  = 5;  // SCK -> Pin 5 from top

// Calibration factor (counts per gram)
// 1kg load cell: ~2100.0
// 5kg load cell: ~420.0
float cal_factor = 2100.0;
long tare_offset = 0;

static portMUX_TYPE hx711_mux = portMUX_INITIALIZER_UNLOCKED;

// Direct hardware 24-bit read with ESP32 FreeRTOS critical section
long readHX711Raw() {
  portENTER_CRITICAL(&hx711_mux);

  unsigned long raw_data = 0;
  for (int i = 0; i < 24; i++) {
    digitalWrite(PIN_SCK, HIGH);
    delayMicroseconds(2);
    raw_data = (raw_data << 1) | (digitalRead(PIN_DOUT) ? 1 : 0);
    digitalWrite(PIN_SCK, LOW);
    delayMicroseconds(2);
  }

  // 25th pulse: Channel A, Gain 128 (pulls DOUT back HIGH to start next conversion)
  digitalWrite(PIN_SCK, HIGH);
  delayMicroseconds(2);
  digitalWrite(PIN_SCK, LOW);
  delayMicroseconds(2);

  portEXIT_CRITICAL(&hx711_mux);

  // Sign-extend 24-bit two's complement to 32-bit signed long
  if (raw_data & 0x800000) {
    raw_data |= 0xFF000000;
  }
  return (long)raw_data;
}

bool waitForReady(uint32_t timeout_ms = 250) {
  unsigned long start = millis();
  while (digitalRead(PIN_DOUT) == HIGH) {
    if (millis() - start >= timeout_ms) {
      return false;
    }
    delay(1);
  }
  return true;
}

void tare(int samples = 10) {
  Serial.print("\n>>> Taring scale... ");
  long sum = 0;
  int valid = 0;
  for (int i = 0; i < samples; i++) {
    if (waitForReady(300)) {
      long val = readHX711Raw();
      sum += val;
      valid++;
    }
    delay(10);
  }
  if (valid > 0) {
    tare_offset = sum / valid;
    Serial.printf("DONE! New Offset = %ld (0x%06lX)\n\n", tare_offset, tare_offset & 0xFFFFFF);
  } else {
    Serial.println("FAILED! HX711 not responding.");
  }
}

void setup() {
  Serial.begin(57600);
  delay(500);

  Serial.println("\n=======================================================");
  Serial.println("     ESP32-S3 DIRECT BIT-BANG LOAD CELL BENCH         ");
  Serial.println("=======================================================");
  Serial.printf("Pins: DT=GPIO %d, SCK=GPIO %d\n", PIN_DOUT, PIN_SCK);
  Serial.printf("Calibration Factor: %.1f counts/gram\n", cal_factor);
  Serial.println("Serial Commands:");
  Serial.println("  't'         -> Tare / Zero the scale");
  Serial.println("  'c <val>'   -> Change calibration factor (e.g. 'c 420' or 'c 2100')");
  Serial.println("  '+' or '-'  -> Fine tune calibration factor by +/- 50");
  Serial.println("=======================================================\n");

  pinMode(PIN_DOUT, INPUT);
  pinMode(PIN_SCK, OUTPUT);
  digitalWrite(PIN_SCK, LOW);

  // Power cycle / reset HX711
  digitalWrite(PIN_SCK, HIGH);
  delayMicroseconds(80);
  digitalWrite(PIN_SCK, LOW);
  delay(150);

  Serial.print("Taring at startup... please keep load cell still... ");
  tare(10);
}

unsigned long last_print = 0;
String rx_buffer = "";

void loop() {
  // Handle Serial Commands
  while (Serial.available() > 0) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      rx_buffer.trim();
      if (rx_buffer.equalsIgnoreCase("t") || rx_buffer.equalsIgnoreCase("tare")) {
        tare(10);
      } else if (rx_buffer.startsWith("c ") || rx_buffer.startsWith("C ")) {
        float new_cal = rx_buffer.substring(2).toFloat();
        if (new_cal > 1.0) {
          cal_factor = new_cal;
          Serial.printf("\n>>> Updated Calibration Factor: %.1f\n\n", cal_factor);
        }
      } else if (rx_buffer.equals("+")) {
        cal_factor += 50.0;
        Serial.printf("\n>>> Cal Factor: %.1f\n\n", cal_factor);
      } else if (rx_buffer.equals("-")) {
        cal_factor = max(10.0f, cal_factor - 50.0f);
        Serial.printf("\n>>> Cal Factor: %.1f\n\n", cal_factor);
      }
      rx_buffer = "";
    } else {
      rx_buffer += ch;
    }
  }

  // Check if HX711 is ready (DOUT goes LOW)
  if (digitalRead(PIN_DOUT) == LOW) {
    long raw = readHX711Raw();
    long delta = raw - tare_offset;
    float grams = (float)delta / cal_factor;

    Serial.printf("[RAW: 0x%06lX | %8ld]  Delta: %8ld  |  Weight: %7.1f g\n", 
                  raw & 0xFFFFFF, raw, delta, grams);
  }
}
