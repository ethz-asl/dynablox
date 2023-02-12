#!/usr/bin/python3

import os
import numpy as np
from matplotlib import pyplot as plt
from plotting_tools import read_cloud_data

DATA_PATH = "/media/lukas/T7/data/ours/standard_inf"    # doals_nodrift_inf
SCENES = ["hauptgebaeude", "niederdorf", "shopville", "station"]
SEQUENCES = [1, 2]
OUTPUT_DIR = "/home/lukas/Documents/motion_detection/range_analysis"


def main():
    # Plot config.
    metrics = ['iou', 'precision', 'recall']
    max_range = 50  # m, Max Distance: 172.732, 82.48% within 20m, 93.30% in 30
    normalize_by_global_points = True
    perform_analysis = False
    plot_ranges = False
    plot_finals = True

    # Data
    data, names, num_points = read_data()  # data [bag_id][TP/TN/FP/FN]

    num_points = None if not normalize_by_global_points else num_points

    # Analysis if needed.
    if perform_analysis:
        considered_range = 20  # m,
        analyze_distances(data, considered_range)
        return

    # Plot
    if plot_ranges:
        for m in metrics:
            for summary in [True, False]:
                plot_range(data, names, m, summary, max_range, num_points)

    if plot_finals:
        plot_final(data, names, max_range, num_points)


def analyze_distances(data, considered_range):
    distances = np.array([])
    for d in data:
        for m in ['TP', 'FP', 'TN', 'FN']:
            distances = np.append(distances, d[m])
    print(f"Max Dist: {np.max(distances)}.")
    print(
        f"Points within {considered_range}: {np.sum(distances<=considered_range)/len(distances)*100:.2f}."
    )


def read_data():
    data = []
    names = []
    num_points = 0
    for s in SCENES:
        for seq in SEQUENCES:
            name = f"{s}_{seq}_none"
            processed_data = {'TP': [], 'FP': [], 'TN': [], 'FN': []}
            cloud_data = read_cloud_data(os.path.join(DATA_PATH, name,
                                                      "clouds.csv"))
            distances = cloud_data['Distance']
            dynamic = cloud_data['ClusterDynamic']
            gt_dynamic = cloud_data['GTDynamic']
            tp = np.logical_and(dynamic, gt_dynamic)
            fp = np.logical_and(dynamic, np.logical_not(gt_dynamic))
            tn = np.logical_and(np.logical_not(dynamic),
                                np.logical_not(gt_dynamic))
            fn = np.logical_and(np.logical_not(dynamic), gt_dynamic)
            num_points += len(distances)
            processed_data['TP'] = distances[tp]
            processed_data['FP'] = distances[fp]
            processed_data['TN'] = distances[tn]
            processed_data['FN'] = distances[fn]
            data.append(processed_data)
            names.append(name)
    return data, names, num_points


def plot_final(data, names, max_range, num_points):
    print("Plotting final.")
    # For each range compute the metric and number of valid points.
    distances = np.linspace(0, max_range, max_range)

    # Get data.
    iou_m = []
    iou_std = []
    pre_m = []
    pre_std = []
    rec_m = []
    rec_std = []
    points_m = []
    points_std = []
    for d in distances:
        points, iou = compute_iou_and_points(data, d, 'iou')
        _, rec = compute_iou_and_points(data, d, 'recall')
        _, pre = compute_iou_and_points(data, d, 'precision')
        iou_m.append(np.mean(iou))
        iou_std.append(np.std(iou))
        points_m.append(np.mean(points))
        points_std.append(np.std(points))
        rec_m.append(np.mean(rec))
        rec_std.append(np.std(rec))
        pre_m.append(np.mean(pre))
        pre_std.append(np.std(pre))

    # Plot.
    plt.figure(figsize=(10, 5))

    plt.plot(distances, iou_m, 'b-')
    plt.plot(distances, pre_m, 'g--')
    plt.plot(distances, rec_m, 'r-.')
    plt.plot(distances, points_m, 'k:')
    plt.fill_between(distances,
                     np.array(iou_m) - np.array(iou_std),
                     np.array(iou_m) + np.array(iou_std),
                     facecolor='b',
                     alpha=.2)
    plt.fill_between(distances,
                     np.array(pre_m) - np.array(pre_std),
                     np.array(pre_m) + np.array(pre_std),
                     facecolor='g',
                     alpha=.2)
    plt.fill_between(distances,
                     np.array(rec_m) - np.array(rec_std),
                     np.array(rec_m) + np.array(rec_std),
                     facecolor='r',
                     alpha=.2)
    plt.fill_between(distances,
                     np.array(points_m) - np.array(points_std),
                     np.array(points_m) + np.array(points_std),
                     facecolor='k',
                     alpha=.2)

    plt.ylim([0, 100])
    plt.xlabel("Distance [m]")
    plt.ylabel("Value [%]")
    plt.legend(["IoU", "Precision", "Recall", "Points considered"])
    plt.savefig(os.path.join(OUTPUT_DIR, "final.svg"))


def plot_range(data, names, metric, summarize, max_range):
    print(f"Plotting '{metric}' {'for all' if summarize else 'individually'}.")
    # For each range compute the IoU and number of valid points.
    distances = np.linspace(0, max_range)

    # Plot.
    plt.figure(figsize=(10, 5))
    iou_m = []
    iou_std = []
    points_m = []
    points_std = []
    if summarize:
        for d in distances:
            points, iou = compute_iou_and_points(data, d, metric)
            iou_m.append(np.mean(iou))
            iou_std.append(np.std(iou))
            points_m.append(np.mean(points))
            points_std.append(np.std(points))
        plt.plot(distances, iou_m, 'b-')
        plt.plot(distances, points_m, 'k-')
        plt.fill_between(distances,
                         np.array(iou_m) - np.array(iou_std),
                         np.array(iou_m) + np.array(iou_std),
                         facecolor='b',
                         alpha=.2)
        plt.fill_between(distances,
                         np.array(points_m) - np.array(points_std),
                         np.array(points_m) + np.array(points_std),
                         facecolor='k',
                         alpha=.2)
    else:
        colors = ['b', 'r', 'g', 'k', 'm', 'c', 'y', 'orange']
        for j, _ in enumerate(names):
            iou_m = []
            points_m = []
            for d in distances:
                points, iou = compute_iou_and_points([data[j]], d, metric)
                iou_m.append(np.mean(iou))
                points_m.append(np.mean(points))
            plt.plot(distances, iou_m, color=colors[j], linestyle='-')
            plt.plot(distances,
                     points_m,
                     color=colors[j],
                     linestyle=':',
                     label='_nolegend_')
    plt.ylim([0, 100])
    plt.xlabel("Distance [m]")
    if summarize:
        plt.ylabel("Value [%]")
        plt.legend([f"Detection {metric}", "GT Points considered"])
        plt.savefig(os.path.join(OUTPUT_DIR, f"{metric}_all.svg"))
    else:
        plt.title(f"- {metric}, : considered points")
        plt.ylabel("Value [%]")
        plt.legend(names)
        plt.savefig(os.path.join(OUTPUT_DIR, f"{metric}_individual.svg"))


def compute_iou_and_points(data, distance, metric):
    num_points = []
    iou = []
    for d in data:
        tp = np.sum(d['TP'] <= distance)
        fp = np.sum(d['FP'] <= distance)
        tn = np.sum(d['TN'] <= distance)
        fn = np.sum(d['FN'] <= distance)
        points = tp + fp + tn + fn
        total_points = np.sum([len(x) for x in d.values()])
        num_points.append(points / total_points * 100)
        if metric == 'iou':
            if tp + fp + fn == 0:
                iou.append(100)
            else:
                iou.append(tp / (tp + fp + fn) * 100)
        elif metric == 'precision':
            if tp + fp == 0:
                iou.append(100)
            else:
                iou.append(tp / (tp + fp) * 100)
        else:  # recall
            if tp + fn == 0:
                iou.append(100)
            else:
                iou.append(tp / (tp + fn) * 100)

    return np.array(num_points), np.array(iou)


if __name__ == "__main__":
    main()
