#!/usr/bin/python3

import os
import numpy as np
import csv
from matplotlib import pyplot as plt

DATA_PATH = "/home/lukas/motion_ws/src/lidar_motion_detection/drift_simulation/config/rollouts/doals"
SCENES = ["hauptgebaeude", "niederdorf", "shopville", "station"]
SEQUENCES = [1, 2]
INTENSITIES = ['light', 'moderate', 'strong', 'severe']
OUTPUT_DIR = "/home/lukas/Documents/motion_detection/drift_analysis"


def read_drift_data(csv_file):
    data = []
    if not os.path.isfile(csv_file):
        print(f"File '{csv_file}' does not exist!")
        return data
    with open(csv_file, newline='\n') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            data.append([float(r) for r in row])
    return np.array(data)


def read_data():
    data = []
    names = []
    for s in SCENES:
        for seq in SEQUENCES:
            name = f"{s}_{seq}"
            d = {'none': read_drift_data(os.path.join(
                DATA_PATH, s, f"sequence_{seq}", "none.csv"))}
            for i in INTENSITIES:
                d[i] = []
                for r in [1, 2, 3]:
                    d[i].append(read_drift_data(os.path.join(
                        DATA_PATH, s, f"sequence_{seq}", f"{i}_{r}.csv")))
            names.append(name)
            data.append(d)
    return data, names


def main():
    data, names = read_data()  # data [bag][intensity][rollout]

    # Plot rollouts.
    if False:
        current_max = np.zeros((1, 8))
        for id, _ in enumerate(names):
            new_rates = plot_rollouts(data, names, id)
            current_max = np.maximum(current_max, new_rates)
        print("Maximum distances and rates:")
        print(current_max)


def plot_rollouts(data, names, id):
    # Get true poses.
    d = data[id]
    gt = d['none']
    linestyles = ['-', '--', ':']
    colors = ['g', 'b', 'k', 'r']
    plt.figure(figsize=(10, 7))
    plt.subplot(2, 1, 1)
    X = []
    Y = []
    drift = []
    max_values = []
    out_file = open(os.path.join(OUTPUT_DIR, f"{names[id]}.txt"), 'w')
    for j in range(3):
        for i, intensity in enumerate(INTENSITIES):
            values = d[intensity][j]
            num_x = np.minimum(len(values), len(gt))
            x = np.linspace(0, num_x, num_x) * 0.1
            y = np.sqrt(np.power(values[:num_x, 0] - gt[:num_x, 0], 2) + np.power(
                values[:num_x, 1] - gt[:num_x, 1], 2) + np.power(values[:num_x, 2] - gt[:num_x, 2], 2))
            X.append(x)
            Y.append(y)
            plt.plot(x, y, color=colors[i], linestyle=linestyles[j])
            drift.append(np.max(np.abs(y)))
            if j == 2:
                max_values.append(
                    np.max([drift[l * len(INTENSITIES) + i] for l in range(3)]))
                out_file.write(
                    f"Max. drift for '{intensity}': {max_values[-1]:.4f} m\n")
    plt.legend(INTENSITIES)
    plt.xlabel("Time [s]")
    plt.ylabel("Drift [m]")
    plt.title(names[id])
    plt.subplot(2, 1, 2)
    rates = []
    for j in range(3):
        for i, intensity in enumerate(INTENSITIES):
            k = j * len(INTENSITIES) + i
            y = Y[k]
            y2 = np.zeros_like(y)
            for l in range(1, len(y)):
                y2[l] = (y[l]-y[l-1])/0.1
            plt.plot(x, y2, color=colors[i], linestyle=linestyles[j])
            rates.append(np.max(np.abs(y2)))
            if j == 2:
                max_values.append(
                    np.max([rates[l * len(INTENSITIES) + i] for l in range(3)]))
                out_file.write(
                    f"Max. drift rate for '{intensity}': {max_values[-1]:.4f} m/s\n")
    plt.xlabel("Time [s]")
    plt.ylabel("Drift Rate [m/s]")
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, f"{names[id]}.svg"))
    out_file.close()
    return np.array(max_values)


if __name__ == "__main__":
    main()
