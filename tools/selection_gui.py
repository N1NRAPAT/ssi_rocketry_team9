import tkinter as tk
import serial
import threading

PORT = "/dev/cu.usbmodem11401"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.05)

def send_cmd(cmd):
    ser.write(cmd.encode("ascii"))

def reader_loop(text_widget):
    while True:
        line = ser.readline().decode(errors="ignore")
        if line:
            text_widget.insert(tk.END, line)
            text_widget.see(tk.END)

def main():
    root = tk.Tk()
    root.title("Pico Sensor Tester")

    frame = tk.Frame(root)
    frame.pack(padx=10, pady=10)

    tk.Button(frame, text="IMU",        command=lambda: send_cmd("i")).grid(row=0, column=0, padx=5, pady=5)
    tk.Button(frame, text="Barometer",  command=lambda: send_cmd("b")).grid(row=0, column=1, padx=5, pady=5)
    tk.Button(frame, text="All",        command=lambda: send_cmd("a")).grid(row=0, column=2, padx=5, pady=5)
    tk.Button(frame, text="Stop",       command=lambda: send_cmd("n")).grid(row=0, column=3, padx=5, pady=5)

    text = tk.Text(root, width=80, height=20)
    text.pack(padx=10, pady=10)

    t = threading.Thread(target=reader_loop, args=(text,), daemon=True)
    t.start()

    root.mainloop()

if __name__ == "__main__":
    main()
