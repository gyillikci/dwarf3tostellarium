import sys
import time
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0

ser = serial.Serial(port, baud, timeout=0.5)
print(f"Listening on {port} @ {baud} for {seconds}s (DTR/RTS toggled)...")
ser.dtr = True
ser.rts = True

end = time.time() + seconds
total = 0
while time.time() < end:
    chunk = ser.read(256)
    if chunk:
        total += len(chunk)
        print(chunk.hex(" "))
ser.close()
print(f"Total bytes received: {total}")
