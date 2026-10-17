import serial
import time

try:
    ser = serial.Serial('COM8', 115200, timeout=1)
    ser.dtr = True  # CRITICAL for ESP32-S3 Native USB CDC to transmit data
    ser.rts = True
    time.sleep(1)
    
    print("--- Connected to COM8! Waiting for $RIG lines... ---")
    
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"RECEIVED: {line}")
        time.sleep(0.01)

except Exception as e:
    print(f"\nSERIAL ERROR: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()