#!/usr/bin/python3

import os
import numpy as np
import argparse
from data_tools import read_score_data, verify_data, get_grid


SCENES = ["hauptgebaeude", "niederdorf", "station", "shopville"]
SEQUENCES = [1, 2]


def main(args):
    # Metrics
    # timestamp	point_IoU	point_Precision	point_Recall	point_TP	point_TN	point_FP	point_FN	cluster_IoU	cluster_Precision	cluster_Recall	cluster_TP	cluster_TN	cluster_FP	cluster_FN	object_IoU	object_Precision	object_Recall	object_TP	object_TN	object_FP	object_FN	EvaluatedPoints	TotalPoints
    metrics = ['object_IoU', 'object_Precision', 'object_Recall']

    # Print configuration
    print_names = True
    print_std = True
    print_nan = True
    print_mode = 'read'  # 'read', 'csv', 'latex'
    print_overall = True

    # Run.
    table(args.data_path, metrics, print_names, print_std, print_nan, print_mode,
          print_overall)


def read_data(data_path):
    data = []
    names = []
    success, name, datum = read_single_dir(data_path)
    if success:
        data.append(datum)
        names.append(name)
    for d in os.listdir(data_path):
        if not os.path.isdir(os.path.join(data_path, d)) or d == '..':
            continue
        success, name, datum = read_single_dir(os.path.join(data_path, d))
        if success:
            data.append(datum)
            names.append(name)
    return data, names


def read_single_dir(path):
    print(path)
    config_file = os.path.join(path, "config.txt")
    scores_file = os.path.join(path, "scores.csv")
    if not os.path.isfile(config_file) or not os.path.isfile(scores_file):
        return False, None, None
    with open(config_file, 'r') as file:
        lines = file.readlines()
        string = "".join(lines).replace('\n', '').replace(' ', '')
        for s in SCENES:
            for seq in SEQUENCES:
                if string.find(f"{s}/sequence_{seq}/indices.csv") != -1:
                    return True, f"{s}_{seq}", read_score_data(scores_file)
    return False, None, None


def table(data_path, metrics,
          print_names=True,
          print_std=True,
          print_nan=True,
          print_mode=False,
          print_overall=True):
    data, names = read_data(data_path)
    verify_data(data, names)

    def print_row(entries):
        if print_mode == 'latex':
            print(("".join('%-15s & ' % x for x in entries))[:-2] + "\\\\")
        elif print_mode == 'csv':
            print(("".join('%s,' % x for x in entries))[:-1])
        else:
            print("".join('%-40s' % x for x in entries))

    def evaluate_row(results):
        nans = np.sum(np.isnan(results))
        msg = (f"{np.nanmean(results):.1f}" if nans == 0 else "-")
        if print_std and nans == 0:
            if print_mode == 'latex':
                msg = msg + f" $\pm$ {np.nanstd(results):.1f}"
            elif print_mode == 'csv':
                msg = msg + f",{np.nanstd(results):.1f}"
            else:
                msg = msg + f" +- {np.nanstd(results):.1f}"

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
    parser = argparse.ArgumentParser(
        prog='evaluate_data',
        description='Present raw data as a human readable table.')
    parser.add_argument('data_path')
    main(parser.parse_args())
