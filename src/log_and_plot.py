import serial
import csv
import time
import numpy as np
import matplotlib.pyplot as plt

# Change this to match your Arduino's serial port
SERIAL_PORT = "/dev/ttyUSB0"  # Adjust for Windows: "COM3"
BAUD_RATE = 9600
OUTPUT_CSV = "thermal_scan.csv"
HEATMAP_FILE = "thermal_image.png"

def read_serial_data():
    data = []
    metadata = {}
    recording = False

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, open(OUTPUT_CSV, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["X", "Y", "Temperature"])  # CSV Header

            while True:
                line = ser.readline().decode("utf-8").strip()
                if not line:
                    continue

                if "<SCAN_START>" in line:
                    recording = True
                    continue
                if "<SCAN_END>" in line:
                    break

                if "Start X" in line:
                    metadata["X_MIN"] = float(line.split(":")[1].strip())
                elif "End X" in line:
                    metadata["X_MAX"] = float(line.split(":")[1].strip())
                elif "Start Y" in line:
                    metadata["Y_MIN"] = float(line.split(":")[1].strip())
                elif "End Y" in line:
                    metadata["Y_MAX"] = float(line.split(":")[1].strip())
                elif "Resolution" in line:
                    metadata["RESOLUTION"] = float(line.split(":")[1].strip())

                elif recording:
                    writer.writerow(line.split(","))
                    data.append([float(val) for val in line.split(",")])

    except Exception as e:
        print("Error:", e)

    return data, metadata

def plot_thermal_image(data, metadata):
    x_vals, y_vals, temp_vals = zip(*data)
    x_unique = sorted(set(x_vals))
    y_unique = sorted(set(y_vals))

    temp_grid = np.zeros((len(y_unique), len(x_unique)))

    for (x, y, t) in data:
        i = y_unique.index(y)
        j = x_unique.index(x)
        temp_grid[i, j] = t

    plt.imshow(temp_grid, cmap='jet', origin='lower', extent=(metadata["X_MIN"], metadata["X_MAX"], metadata["Y_MIN"], metadata["Y_MAX"]))
    plt.colorbar(label="Temperature (°C)")
    plt.xlabel("X Position (mm)")
    plt.ylabel("Y Position (mm)")
    plt.title("Thermal Map")
    plt.savefig(HEATMAP_FILE)
    plt.show()

data, metadata = read_serial_data()
plot_thermal_image(data, metadata)
