import serial
import serial.tools.list_ports
import time

def find_pico_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if ("Pico" in p.description or "RP2" in p.description):
            # Prefer /tty.*, but accept /cu.* if that's all we have
            if "/tty." in p.device:
                return p.device
            elif "/cu." in p.device:
                return p.device
    return None


print("Scanning for Pico...")

port = None
while port is None:
    port = find_pico_port()
    if port is None:
        print("Waiting for Pico...")
        time.sleep(0.5)

print(f"Pico detected at {port}")
ser = serial.Serial(port, 115200)

print("=== Serial Monitor Started ===")
while True:
    try:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print(line)
    except KeyboardInterrupt:
        print("\nStopped.")
        break
    except serial.SerialException:
        print("Pico disconnected — waiting...")
        ser.close()
        port = None
        while port is None:
            port = find_pico_port()
            time.sleep(0.5)
        ser = serial.Serial(port, 115200)
