# 3 Dec 2025 created by Nin : Test serial monitor auto detect


import serial
import serial.tools.list_ports
import time

def find_pico_port():
    ports = serial.tools.list_ports.comports() # get serial via USB-port between Pico and mac
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
    #loop print searching pico when pico not ready yet
    port = find_pico_port()
    if port is None:
        print("Waiting for Pico...")
        time.sleep(0.5)

print(f"Pico detected at {port}") #port -> dev/tty...
ser = serial.Serial(port, 115200)

print("=== Serial Monitor Started ===")
while True:
    try:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print(line) # print line from pico output to main serial monitor
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
