#!/usr/bin/python3

import os
import numpy as np
from plotting_tools import read_plot_data_csv, verify_data, get_grid

DATA_PATH = "/media/lukas/T7/data"
SCENES = ["hauptgebaeude", "niederdorf", "shopville", "station"]
SEQUENCES = [1, 2]


def read_data(data_set):
    data = []
    names = []
    for s in SCENES:
        for seq in SEQUENCES:
            name = f"{s}_{seq}_none"
            data.append(read_plot_data_csv(os.path.join(
                DATA_PATH, data_set, name, "scores.csv")))
            names.append(name)
    return data, names


def main():
    # What to plot
    # timestamp	point_IoU	point_Precision	point_Recall	point_TP	point_TN	point_FP	point_FN	cluster_IoU	cluster_Precision	cluster_Recall	cluster_TP	cluster_TN	cluster_FP	cluster_FN	object_IoU	object_Precision	object_Recall	object_TP	object_TN	object_FP	object_FN	EvaluatedPoints	TotalPoints

    # Metrics
    # doals_nodrift_20m, doals_nodrift_inf, doals_nodrift_20m_inf_eval
    data_set = 'doals_nodrift_inf'
    metrics = ['cluster_IoU', 'cluster_Precision', 'cluster_Recall']

    # Print configuration
    print_names = True
    print_std = True
    print_nan = True
    print_latex = False
    print_overall = True

    # Run.
    table(metrics, data_set, print_names, print_std,
          print_nan, print_latex, print_overall)


def table(metrics, data_set, print_names=True, print_std=True, print_nan=True, print_latex=False, print_overall=True):
    data, names = read_data(data_set)
    verify_data(data, names)

    def print_row(entries):
        if print_latex:
            print(("".join('%-25s & ' % x for x in entries))[:-2] + "\\\\")
        else:
            print("".join('%-25s' % x for x in entries))

    def evaluate_row(results):
        nans = np.sum(np.isnan(results))
        msg = (f"{np.nanmean(results):.1f}" if nans == 0 else "-")
        if print_std and nans == 0:
            msg = msg + (" $\pm$ " if print_latex else " +- ") + \
                f"{np.nanstd(results):.1f}"
        if print_nan and nans > 0:
            msg = msg + f" ({nans})"
        return msg

    all_data = {}
    for m in metrics:
        all_data[m] = np.array([])

    print_row((["Data"] + metrics) if print_names else metrics)
    for i, n in enumerate(names):
        entries = [n] if print_names else []
        for m in metrics:
            results = get_grid(data[i], m) * 100
            all_data[m] = np.append(all_data[m], results)
            entries.append(evaluate_row(results))
        print_row(entries)

    if print_overall:
        entries = ["All"] if print_names else []
        for m in metrics:
            entries.append(evaluate_row(all_data[m]))
        print_row(entries)


if __name__ == "__main__":
    main()
