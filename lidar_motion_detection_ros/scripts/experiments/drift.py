#!/usr/bin/python3

import os
import numpy as np
from plotting_tools import read_plot_data_csv

DATA_PATH = "/media/lukas/T7/data/doals_drift"
SCENES = ["hauptgebaeude", "niederdorf", "shopville", "station"]
PARAMS = [10000, 150, 50, 40, 15]
DRIFT = ["none", "light", "moderate", "strong", "severe"]
SEQUENCES = [1, 2]


def main():
    # What to plot
    # timestamp	point_IoU	point_Precision	point_Recall	point_TP	point_TN	point_FP	point_FN	cluster_IoU	cluster_Precision	cluster_Recall	cluster_TP	cluster_TN	cluster_FP	cluster_FN	object_IoU	object_Precision	object_Recall	object_TP	object_TN	object_FP	object_FN	EvaluatedPoints	TotalPoints

    # Metrics
    metric = 'cluster_Recall'  # cluster_IoU cluster_Precision cluster_Recall

    # Print configuration
    print_std = False
    print_nan = True
    print_mode = 'csv' # read, latex, csv
    sequences = [f"{s}_{seq}" for s in SCENES for seq in SEQUENCES]

    data = read_data()  # data[scene_sequence][param][drift][rollout_id]

    # Run.
    table(data, sequences, metric, print_std,
          print_nan, print_mode)


def read_data():
    data = {}
    complete=0
    total=0
    for s in SCENES:
        for seq in SEQUENCES:
            name = f"{s}_{seq}"
            data[name] = {}
            for p in PARAMS:
                data[name][p] = {}
                for d in DRIFT:
                    data[name][p][d] = []
                    rollouts = [""] if d == "none" else ["_1", "_2", "_3"]
                    for r in rollouts:
                        total += 1
                        data[name][p][d].append(read_plot_data_csv(os.path.join(
                            DATA_PATH, f"{p}", f"{s}_{seq}_{d}{r}", "scores.csv")))
                        if len(data[name][p][d][-1]['cluster_IoU']) != 10:
                            print(
                                f"Incomplete data: '{p}/{s}_{seq}_{d}{r}' ({len(data[name][p][d][-1]['cluster_IoU'])}/10).")
                        else:
                            complete += 1
    print(f"Read data: {complete}/{total} entries complete.")
    return data


def table(data, sequences, metric, print_std=True, print_nan=True, print_mode='read'):

    def print_row(entries):
        if print_mode == 'latex':
            print(("".join('%-15s & ' % x for x in entries))[:-2] + "\\\\")
        elif print_mode == 'csv':
            print(("".join('%s,' % x for x in entries))[:-1])
        else:
            print("".join('%-15s' % x for x in entries))

    def evaluate_row(results):
        nans = np.sum(np.isnan(results))
        msg = (f"{np.nanmean(results):.1f}" if nans == 0 else "-")
        if print_std and nans == 0:
            msg = msg + (" $\pm$ " if print_mode=='latex' else " +- ") + \
                f"{np.nanstd(results):.1f}"
        if print_nan and nans > 0:
            msg = msg + f" ({nans})"
        return msg

    print_row(["Param/Drift"] + DRIFT)
    for i, p in enumerate(PARAMS):
        entries = [f"{p} ({DRIFT[i]})"]
        for d in DRIFT:
            values = []
            for s in sequences:
                for rollout_data in data[s][p][d]:
                    values.append(rollout_data[metric])
            entries.append(evaluate_row(np.array(values)*100))
        print_row(entries)

if __name__ == "__main__":
    main()
