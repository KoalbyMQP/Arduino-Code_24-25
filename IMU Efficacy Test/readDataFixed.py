import serial
import csv
import pandas as pd
import matplotlib.pyplot as plt
ser = serial.Serial('/dev/cu.usbmodem1101', 115200)  # Adjust with your port

n = 50  # Number of data points to collect before plotting
data_points = 0  # Number of data points collected so far

with open('data.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["IMU X Values", "Fixed X Values"])  # Write header

    while True:
        line = ser.readline().decode('utf-8').strip()  # Read line from Arduino
        parts = line.split(' ')
        if len(parts) == 4:  # Check if line can be split into six parts
            try:
                IMU_X = float(parts[1])  # Try to convert parts to floats
                FIXED_X = float(parts[3])
                #z = float(parts[5])
                writer.writerow([IMU_X, FIXED_X])  # Write row to CSV
                data_points += 1  # Increment the number of data points collected
            except ValueError:
                print(f"Cannot convert {line} to floats.")

        # If we have collected n data points, plot the data
        if data_points == n:
            # Read the CSV file
            df = pd.read_csv('data.csv', header=None, names=['IMU_X', 'FIXED_X'])

            # Plot X vs Y
            # plt.figure(figsize=(10, 6))
            # plt.plot(df['X'], df['Y'], marker='o')
            # plt.title('X vs Y')
            # plt.xlabel('X')
            # plt.ylabel('Y')
            # plt.grid(True)
            # plt.show()

            # Reset the number of data points collected
            data_points = 0
            break