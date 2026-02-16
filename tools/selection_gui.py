import tkinter as tk
import serial
import serial.tools.list_ports
import threading
import csv
from datetime import datetime
import os
import time

# CSV output directory
CSV_OUTPUT_DIR = "/Users/punnaratsuttinual/SSI_rocketry/data/document_image/csvfile"

os.makedirs(CSV_OUTPUT_DIR, exist_ok=True)

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


class SimpleIMUMonitor:
    def __init__(self, root):
        self.root = root
        self.root.title("IMU Monitor")
        self.root.geometry("700x500")
        
        # Connect to Pico
        self.port = find_pico_port()
        if self.port:
            self.ser = serial.Serial(self.port, 115200, timeout=0.05)
            print(f"Connected to {self.port}")
        else:
            self.ser = None
            print("Pico not found!")
        
        # State
        self.mode = "idle"
        self.csv_file = None
        self.csv_writer = None
        self.data_count = 0
        self.timer_start = 0
        self.timer_duration = 0
        self.running = True
        
        self.setup_ui()
        
        # Start threads
        threading.Thread(target=self.reader_loop, daemon=True).start()
        threading.Thread(target=self.update_timer, daemon=True).start()
    
    def setup_ui(self):
        # Top controls
        control_frame = tk.Frame(self.root, padx=10, pady=10)
        control_frame.pack(fill=tk.X)
        
        # Mode buttons
        tk.Button(control_frame, text="Test", command=self.start_test, 
                 width=10, bg="#5DADE2").grid(row=0, column=0, padx=5)
        tk.Button(control_frame, text="Log CSV", command=self.start_logging, 
                 width=10, bg="#52BE80").grid(row=0, column=1, padx=5)
        tk.Button(control_frame, text="Stop", command=self.stop, 
                 width=10, bg="#EC7063").grid(row=0, column=2, padx=5)
        
        # Timer selection
        tk.Label(control_frame, text="Duration:").grid(row=0, column=3, padx=(20, 5))
        self.duration_var = tk.IntVar(value=10)
        for i, dur in enumerate([5, 10, 30, 60]):
            tk.Radiobutton(control_frame, text=f"{dur}s", 
                          variable=self.duration_var, value=dur).grid(row=0, column=4+i, padx=2)
        
        # Status bar
        status_frame = tk.Frame(self.root, bg="#ECF0F1", padx=10, pady=5)
        status_frame.pack(fill=tk.X)
        
        tk.Label(status_frame, text="Mode:", bg="#ECF0F1").pack(side=tk.LEFT)
        self.mode_label = tk.Label(status_frame, text="IDLE", bg="#ECF0F1", fg="gray")
        self.mode_label.pack(side=tk.LEFT, padx=5)
        
        tk.Label(status_frame, text="Samples:", bg="#ECF0F1").pack(side=tk.LEFT, padx=(20, 0))
        self.sample_label = tk.Label(status_frame, text="0", bg="#ECF0F1", fg="gray")
        self.sample_label.pack(side=tk.LEFT, padx=5)
        
        tk.Label(status_frame, text="Timer:", bg="#ECF0F1").pack(side=tk.LEFT, padx=(20, 0))
        self.timer_label = tk.Label(status_frame, text="--:--", bg="#ECF0F1", fg="gray")
        self.timer_label.pack(side=tk.LEFT, padx=5)
        
        # Text output
        text_frame = tk.Frame(self.root, padx=10, pady=10)
        text_frame.pack(fill=tk.BOTH, expand=True)
        
        scrollbar = tk.Scrollbar(text_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.text = tk.Text(text_frame, width=80, height=20, 
                           yscrollcommand=scrollbar.set, font=("Courier", 9))
        self.text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.text.yview)
        
        self.log("IMU Monitor Ready")
        self.log(f"Port: {self.port if self.port else 'Not connected'}\n")
    
    def log(self, msg):
        self.text.insert(tk.END, msg + "\n")
        self.text.see(tk.END)
    
    def start_test(self):
        if self.mode != "idle" or not self.ser:
            return
        
        self.mode = "test"
        self.data_count = 0
        self.timer_duration = self.duration_var.get()
        self.timer_start = time.time()
        
        self.mode_label.config(text="TEST", fg="blue")
        self.sample_label.config(text="0", fg="blue")
        
        self.log(f"\n>>> TEST MODE - {self.timer_duration}s\n")
        self.ser.write(b"i")
    
    def start_logging(self):
        if self.mode != "idle" or not self.ser:
            return
        
        self.mode = "logging"
        self.data_count = 0
        self.timer_duration = self.duration_var.get()
        self.timer_start = time.time()
        
        # Create CSV
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(CSV_OUTPUT_DIR, f"imu_{timestamp}.csv")
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        self.mode_label.config(text="LOGGING", fg="green")
        self.sample_label.config(text="0", fg="green")
        
        self.log(f"\n>>> LOGGING - {self.timer_duration}s")
        self.log(f"File: {os.path.basename(filename)}\n")
        
        self.ser.write(f"t{self.timer_duration}".encode())
    
    def stop(self):
        if self.mode == "idle":
            return
        
        if self.ser:
            self.ser.write(b"n")
        
        if self.csv_file:
            self.csv_file.close()
            self.log(f"\n>>> CSV saved: {self.data_count} samples\n")
            self.csv_file = None
        
        self.mode = "idle"
        self.mode_label.config(text="IDLE", fg="gray")
        self.timer_label.config(text="--:--", fg="gray")
    
    def update_timer(self):
        while self.running:
            if self.mode != "idle":
                elapsed = time.time() - self.timer_start
                remaining = max(0, self.timer_duration - elapsed)
                
                mins = int(remaining // 60)
                secs = int(remaining % 60)
                self.timer_label.config(text=f"{mins:02d}:{secs:02d}")
                
                if remaining <= 0:
                    self.root.after(0, self.stop)
            
            time.sleep(0.1)
    
    def reader_loop(self):
        while self.running:
            if self.ser and self.mode in ["test", "logging"]:
                try:
                    line = self.ser.readline().decode(errors="ignore").strip()
                    if line:
                        self.root.after(0, self.process_line, line)
                except:
                    pass
            time.sleep(0.01)
    
    def process_line(self, line):
        # Handle CSV header
        if line.startswith('CSV_HEADER'):
            parts = line.split(',', 1)
            if len(parts) > 1 and self.csv_writer:
                self.csv_writer.writerow(parts[1].split(','))
                self.csv_file.flush()
        
        # Handle CSV data
        elif line.startswith('CSV_DATA'):
            if self.csv_writer:
                parts = line.split(',', 1)
                if len(parts) > 1:
                    self.csv_writer.writerow(parts[1].split(','))
                    self.data_count += 1
                    
                    if self.data_count % 10 == 0:
                        self.csv_file.flush()
                        self.sample_label.config(text=str(self.data_count))
        
        # Handle end
        elif line.startswith('CSV_END'):
            pass
        
        # Display all messages
        else:
            self.log(line)
            if "Sample" in line and self.mode == "test":
                self.data_count += 1
                self.sample_label.config(text=str(self.data_count))


def main():
    root = tk.Tk()
    app = SimpleIMUMonitor(root)
    root.mainloop()


if __name__ == "__main__":
    main()