import tkinter as tk
import serial
import serial.tools.list_ports
import threading
import csv
from datetime import datetime
import os

# CSV output directory
CSV_OUTPUT_DIR = "/Users/punnaratsuttinual/SSI_rocketry/data/document_image/csvfile"

if CSV_OUTPUT_DIR != ".":
    os.makedirs(CSV_OUTPUT_DIR, exist_ok=True)
    print(f"CSV files will be saved to: {os.path.abspath(CSV_OUTPUT_DIR)}")
else:
    print(f"CSV files will be saved to: {os.path.abspath('.')}")

def find_pico_port():
    """Auto-detect Pico serial port"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'USB Serial' in port.description or 'Pico' in port.description:
            return port.device
        if port.vid == 0x2E8A:
            return port.device
    for port in ports:
        if 'usbmodem' in port.device or 'ttyACM' in port.device:
            return port.device
    return None

PORT = find_pico_port()
if PORT is None:
    print("ERROR: Pico not found! Using default port.")
    PORT = "/dev/cu.usbmodem11401"
else:
    print(f"Found Pico on: {PORT}")

BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

# CSV state
csv_file = None
csv_writer = None
csv_filename = None

def send_cmd(cmd):
    ser.write(cmd.encode("ascii"))

def reader_loop(text_widget, status_label, pitch_label, roll_label, dpitch_label, droll_label):
    global csv_file, csv_writer, csv_filename
    data_count = 0

    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
        except Exception as e:
            print(f"Serial error: {e}")
            continue

        if not line:
            continue

        # Show everything in the text box
        text_widget.insert(tk.END, line + "\n")
        text_widget.see(tk.END)

        # Parse pitch/roll from human-readable output and update live labels
        # Line format: "  Pitch:   15.23 deg  (delta: +3.45 deg from sample 0)"
        if line.strip().startswith("Pitch:"):
            try:
                parts = line.split()
                pitch_val  = float(parts[1])
                delta_val  = float(parts[4])
                pitch_label.config(text=f"{pitch_val:+.2f} deg")
                dpitch_label.config(text=f"{delta_val:+.2f} deg",
                                    fg="red" if abs(delta_val) > 10 else "green")
            except:
                pass
        elif line.strip().startswith("Roll:"):
            try:
                parts = line.split()
                roll_val   = float(parts[1])
                delta_val  = float(parts[4])
                roll_label.config(text=f"{roll_val:+.2f} deg")
                droll_label.config(text=f"{delta_val:+.2f} deg",
                                   fg="red" if abs(delta_val) > 10 else "green")
            except:
                pass

        # CSV header → open file and write column names
        if line.startswith('CSV_HEADER'):
            parts = line.split(',', 1)
            if len(parts) > 1:
                headers = parts[1].split(',')
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                csv_filename = os.path.join(CSV_OUTPUT_DIR, f"rocket_data_{timestamp}.csv")
                csv_file = open(csv_filename, 'w', newline='')
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(headers)
                csv_file.flush()
                data_count = 0
                status_label.config(text=f"CSV: Logging to {os.path.basename(csv_filename)}", fg="green")
                text_widget.insert(tk.END, f"\n CSV file created: {csv_filename}\n\n")

        # CSV data row → write to file
        elif line.startswith('CSV_DATA'):
            if csv_writer is not None:
                parts = line.split(',', 1)
                if len(parts) > 1:
                    csv_writer.writerow(parts[1].split(','))
                    data_count += 1
                    if data_count % 10 == 0:
                        csv_file.flush()
                        status_label.config(text=f"CSV: {data_count} samples logged")

        # CSV done → close file
        elif line.startswith('CSV_END'):
            if csv_file:
                csv_file.close()
                csv_file = None
                csv_writer = None
                status_label.config(
                    text=f"CSV: Saved {data_count} samples to {os.path.basename(csv_filename)}",
                    fg="blue"
                )
                text_widget.insert(tk.END, f"\n CSV complete: {data_count} samples saved\n\n")

def start_csv_timed(duration_sec, text_widget, status_label):
    send_cmd(f"t{duration_sec}")
    text_widget.insert(tk.END, f"\n>>> Starting {duration_sec}s CSV logging...\n")
    status_label.config(text=f"CSV: Recording for {duration_sec}s...", fg="orange")

def main():
    root = tk.Tk()
    root.title("Pico Sensor Tester")

    # Sensor buttons
    frame = tk.Frame(root)
    frame.pack(padx=10, pady=10)

    tk.Button(frame, text="IMU",       command=lambda: send_cmd("i"), width=12).grid(row=0, column=0, padx=5, pady=5)
    tk.Button(frame, text="Barometer", command=lambda: send_cmd("b"), width=12).grid(row=0, column=1, padx=5, pady=5)
    tk.Button(frame, text="GPS",       command=lambda: send_cmd("g"), width=12).grid(row=0, column=2, padx=5, pady=5)
    tk.Button(frame, text="All",       command=lambda: send_cmd("a"), width=12).grid(row=0, column=3, padx=5, pady=5)
    tk.Button(frame, text="Stop",      command=lambda: send_cmd("n"), width=12).grid(row=0, column=4, padx=5, pady=5)

    # CSV duration buttons
    csv_frame = tk.Frame(root)
    csv_frame.pack(padx=10, pady=5)

    tk.Label(csv_frame, text="CSV Log Duration:", font=("Arial", 10, "bold")).grid(row=0, column=0, padx=5)
    tk.Button(csv_frame, text="5 sec",  command=lambda: start_csv_timed(5,  text, status_label), width=8, bg="#90EE90").grid(row=0, column=1, padx=5)
    tk.Button(csv_frame, text="10 sec", command=lambda: start_csv_timed(10, text, status_label), width=8, bg="#90EE90").grid(row=0, column=2, padx=5)
    tk.Button(csv_frame, text="30 sec", command=lambda: start_csv_timed(30, text, status_label), width=8, bg="#90EE90").grid(row=0, column=3, padx=5)
    tk.Button(csv_frame, text="60 sec", command=lambda: start_csv_timed(60, text, status_label), width=8, bg="#90EE90").grid(row=0, column=4, padx=5)

    # Status bar — shows CSV status
    status_label = tk.Label(root, text="Status: Idle", font=("Arial", 9), fg="gray")
    status_label.pack(pady=5)

    # Live IMU display — pitch and roll update in real time
    imu_frame = tk.Frame(root, bg="#F0F0F0", padx=10, pady=4)
    imu_frame.pack(fill=tk.X)

    tk.Label(imu_frame, text="Pitch:", bg="#F0F0F0", font=("Courier", 10, "bold")).pack(side=tk.LEFT)
    pitch_label = tk.Label(imu_frame, text="---", bg="#F0F0F0", fg="blue", font=("Courier", 10), width=14)
    pitch_label.pack(side=tk.LEFT, padx=(2, 20))

    tk.Label(imu_frame, text="Roll:", bg="#F0F0F0", font=("Courier", 10, "bold")).pack(side=tk.LEFT)
    roll_label = tk.Label(imu_frame, text="---", bg="#F0F0F0", fg="blue", font=("Courier", 10), width=14)
    roll_label.pack(side=tk.LEFT, padx=(2, 20))

    tk.Label(imu_frame, text="ΔPitch:", bg="#F0F0F0", font=("Courier", 10, "bold")).pack(side=tk.LEFT)
    dpitch_label = tk.Label(imu_frame, text="---", bg="#F0F0F0", fg="green", font=("Courier", 10), width=14)
    dpitch_label.pack(side=tk.LEFT, padx=(2, 20))

    tk.Label(imu_frame, text="ΔRoll:", bg="#F0F0F0", font=("Courier", 10, "bold")).pack(side=tk.LEFT)
    droll_label = tk.Label(imu_frame, text="---", bg="#F0F0F0", fg="green", font=("Courier", 10), width=14)
    droll_label.pack(side=tk.LEFT, padx=(2, 0))

    # Text output
    text = tk.Text(root, width=80, height=20)
    text.pack(padx=10, pady=10)

    t = threading.Thread(target=reader_loop,
                          args=(text, status_label, pitch_label, roll_label, dpitch_label, droll_label),
                          daemon=True)
    t.start()

    root.mainloop()

if __name__ == "__main__":
    main()