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

def reader_loop(text_widget, status_label):
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

    # Status label
    status_label = tk.Label(root, text="Status: Idle", font=("Arial", 9), fg="gray")
    status_label.pack(pady=5)

    # Text output
    text = tk.Text(root, width=80, height=20)
    text.pack(padx=10, pady=10)

    t = threading.Thread(target=reader_loop, args=(text, status_label), daemon=True)
    t.start()

    root.mainloop()

if __name__ == "__main__":
    main()