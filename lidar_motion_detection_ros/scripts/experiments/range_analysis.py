#!/usr/bin/python3

import os
import numpy as np
import csv
from matplotlib import pyplot as plt

DATA_PATH = "/media/lukas/T7/data/doals_nodrift_inf"
SCENES = ["hauptgebaeude", "niederdorf", "shopville"]#, "station"]
SEQUENCES = [1, 2]
OUTPUT_DIR = "/home/lukas/Documents/motion_detection/range_analysis"


def main():
    # Plot config.

    data, names = read_data()  # data [bag_id][TP/TN/FP/FN]
    plot_range(data, names, True)
    plot_range(data, names, False)


def read_range_data(csv_file):
    data = {}
    if not os.path.isfile(csv_file):
        print(f"File '{csv_file}' does not exist!")
        return data
    with open(csv_file, newline='\n') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            data[row[0]] = np.array([float(r) for r in row[1:]])
    return data


def read_data():
    data = []
    names = []
    for s in SCENES:
        for seq in SEQUENCES:
            name = f"{s}_{seq}_none"
            data.append(read_range_data(os.path.join(
                DATA_PATH, name, "ranges.csv")))
            names.append(name)
    return data, names


def plot_range(data, names, summarize):
    # For each range compute the IoU and number of valid points.
    distances = np.linspace(0, 51)

    # Plot.
    plt.figure(figsize=(10, 5))
    iou_m = []
    iou_std = []
    points_m = []
    points_std = []
    if summarize:
        for d in distances:
            points, iou = compute_iou_and_points(data, d)
            iou_m.append(np.mean(iou))
            iou_std.append(np.std(iou))
            points_m.append(np.mean(points))
            points_std.append(np.std(points))
        plt.plot(distances, iou_m, 'b-')
        plt.plot(distances, points_m, 'k-')
        plt.fill_between(distances, np.array(iou_m)-np.array(iou_std),
                         np.array(iou_m) + np.array(iou_std), facecolor='b', alpha=.2)
        plt.fill_between(distances, np.array(points_m)-np.array(points_std),
                         np.array(points_m) + np.array(points_std), facecolor='k', alpha=.2)
    else:
        colors = ['b', 'r', 'g', 'k', 'm', 'c', 'y', 'orange']
        for j, _ in enumerate(names):
            iou_m = []
            points_m = []
            for d in distances:
                points, iou = compute_iou_and_points([data[j]], d)
                iou_m.append(np.mean(iou))
                points_m.append(np.mean(points))
            plt.plot(distances, iou_m, color=colors[j], linestyle='-')
            plt.plot(distances, points_m,
                     color=colors[j], linestyle=':',  label='_nolegend_')
    plt.ylim([0, 100])
    plt.xlabel("Distance [m]")
    if summarize:
        plt.ylabel("IoU / Points [%]")
        plt.legend(["Detection IoU", "GT Points considered"])
        plt.savefig(os.path.join(OUTPUT_DIR, "distances_all.svg"))
    else:
        plt.ylabel("- IoU [%]    : Points [%]")
        plt.legend(names)
        plt.savefig(os.path.join(OUTPUT_DIR, "distances_individual.svg"))


def compute_iou_and_points(data, distance):
    num_points = []
    iou = []
    for d in data:
        tp = np.sum(d['TP'] <= distance)
        fp = np.sum(d['FP'] <= distance)
        tn = np.sum(d['TN'] <= distance)
        fn = np.sum(d['FN'] <= distance)
        points = tp + fp + tn + fn
        total_points = np.sum([len(x) for x in d.values()])
        num_points.append(points/total_points * 100)
        iou.append(tp/(tp+fp+fn)*100)
    return np.array(num_points), np.array(iou)


if __name__ == "__main__":
    main()
