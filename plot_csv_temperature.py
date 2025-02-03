import pandas as pd
import argparse
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter

file_name = ""
# max_sample_size = 1000

def parse_arg():
    parser = argparse.ArgumentParser(description="read a csv and plot the time and temperature")
    parser.add_argument("-f", "--file", required=True)

    args = parser.parse_args()

    return args

def main():
    # Read the `CSV file
    df = pd.read_csv(file_name)

    # Select only the columns 'timestamp(ms)' and 'temp'
    # df = df[['timestamp(ms)', 'temp']]
    df.dropna()

    time_data = df[['timestamp(ms)']].to_numpy().flatten()[::10]
    temp_data = df[['temp']].to_numpy().flatten()[::10]

     

    # filtered_temp = savgol_filter(temp_data, args.window_size, polyorder=2)
    # print(f"shape: {time_data.shape}")
    coeffs = np.polyfit(time_data, temp_data, 3)
    time_polyfit = np.linspace(time_data.min(),time_data.max(),time_data.size)
    polyfit_temp = np.polyval(coeffs, time_polyfit)

    # print(time_data.shape)
    # print(polyfit_temp.shape)
    # print(time_polyfit.shape)

    # print(time_polyfit)

    # Display the filtered data
    plt.figure()
    plt.plot(time_data, temp_data, label="raw", color="blue",linestyle='-')
    plt.plot(time_polyfit, polyfit_temp, label="polyfit", color="red",linestyle='-')
    plt.xlabel("Time (ms)")
    plt.ylabel("Temperature")
    plt.title("Temperature vs Time")
    plt.legend()
    plt.grid()

    plt.show()

    # Display the filtered data
    # plt.figure()
    # plt.plot(time_data, polyfit_temp, label="Temperature", color="blue", marker="o")
    # plt.xlabel("Time (ms)")
    # plt.ylabel("Temperature")
    # plt.title("Temperature vs Time")
    # plt.legend()
    # plt.grid()

    # plt.show()

if __name__=="__main__":
    args = parse_arg()
    file_name = args.file

    main()