#!/usr/bin/python3

import os
import numpy as np

DATA_PATH = "/media/lukas/T7/data"
SCENES = ["hauptgebaeude"]  # , "niederdorf", "shopville", "station"]
SEQUENCES = [1, 2]


def read_time_data(file_name):
    data = {}
    file = open(file_name, 'r')
    lines = file.readlines()
    for l in lines[2:-1]:
        entries = l.split("\t")
        key = entries[0].strip(" ")
        calls = int(entries[1])
        total = float(entries[2])
        mean = float(entries[3].partition(" +- ")[0][1:])
        std = float(entries[3].partition(" +- ")[2][:-1])
        min = float(entries[4].partition(",")[0][1:])
        max = float(entries[4].partition(",")[2][:-2])
        data[key] = {'calls': calls, 'total': total,
                     'mean': mean, 'std': std, 'min': min, 'max': max}
    return data


def read_data(data_set):
    data = []
    names = []
    for s in SCENES:
        for seq in SEQUENCES:
            name = f"{s}_{seq}_none"
            data.append(read_time_data(os.path.join(
                DATA_PATH, data_set, name, "timings.txt")))
            names.append(name)
    return data, names


def main():
    # What to plot
    data_set = 'doals_nodrift_inf_range'  # doals_nodrift_inf, doals_nodrift_20m
    metrics = ['mean', 'std', 'min', 'max']  # 'calls', 'total',
    key = 'motion_detection'    # evaluation, frame, motion_detection, motion_detection/clustering, motion_detection/indexing_setup, motion_detection/preprocessing, motion_detection/tf_lookup, motion_detection/tsdf_integration, motion_detection/update_ever_free, update_ever_free/label_free, update_ever_free/remove_occupied, visualizations

    # Print configuration
    print_by = 'key'    # sequence, key
    print_names = True
    print_std = True
    print_latex = False
    print_overall = print_by == 'sequence'

    # Run.
    data, names = read_data(data_set)
    table(data, names, metrics, key, print_by,  print_names, print_std,
          print_latex, print_overall)


def table(data, names, metrics, key, print_by, print_names=True, print_std=True, print_latex=False, print_overall=True):
    def print_row(entries):
        if print_latex:
            print(("".join('%-25s & ' % x for x in entries))[:-2] + "\\\\")
        else:
            print("".join('%-25s' % x for x in entries))

    def evaluate_row(results, metric):
        if metric == 'min':
            return f"{np.min(results):.3f}"
        elif metric == 'max':
            return f"{np.max(results):.3f}"
        else:
            msg = f"{np.nanmean(results):.3f}"
            if print_std:
                msg = msg + (" $\pm$ " if print_latex else " +- ") + \
                    f"{np.nanstd(results):.3f}"
            return msg

    all_data = {}
    for m in metrics:
        all_data[m] = np.array([])
    if print_by == 'sequence':
        print(f"Evaluating Timer '{key}':")
    if print_by == 'sequence':
        print_row((["Data"] + metrics) if print_names else metrics)
        for i, n in enumerate(names):
            entries = [n] if print_names else []
            for m in metrics:
                results = np.array([data[i][key][m]])
                all_data[m] = np.append(all_data[m], results)
                entries.append(evaluate_row(results, m))
            print_row(entries)
    else:   # by keys
        print_row((['%-35s' % "Data"] + metrics) if print_names else metrics)
        keys = data[0].keys()
        for k in keys:
            entries = ['%-35s' % k] if print_names else []
            for m in metrics:
                results = np.array([data[i][k][m]
                                    for i, _ in enumerate(names)])
                all_data[m] = np.append(all_data[m], results)
                entries.append(evaluate_row(results, m))
            print_row(entries)

    if print_overall:
        entries = [('%-35s' % "All" if print_by ==
                    'key' else "All")] if print_names else []
        for m in metrics:
            entries.append(evaluate_row(all_data[m], m))
        print_row(entries)


if __name__ == "__main__":
    main()
