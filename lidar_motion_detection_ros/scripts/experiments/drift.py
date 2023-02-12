#!/usr/bin/python3

import os
import numpy as np
from plotting_tools import read_plot_data_csv
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits import mplot3d

DATA_PATH = "/media/lukas/T7/data/ours/doals_drift"
OUTPUT_PATH = "/home/lukas/Documents/motion_detection/drift"
SCENES = ["hauptgebaeude", "niederdorf", "shopville", "station"]
PARAMS = [10000, 150, 50, 40, 15]  # 10000, 40
DRIFT = ["none",   "light", "moderate",
         "strong", "severe"]   # "light", "strong",
SEQUENCES = [1, 2]


def main():
    # What to plot
    # timestamp	point_IoU	point_Precision	point_Recall	point_TP	point_TN	point_FP	point_FN	cluster_IoU	cluster_Precision	cluster_Recall	cluster_TP	cluster_TN	cluster_FP	cluster_FN	object_IoU	object_Precision	object_Recall	object_TP	object_TN	object_FP	object_FN	EvaluatedPoints	TotalPoints

    # Metrics
    metric = 'cluster_IoU'  # cluster_IoU cluster_Precision cluster_Recall

    # Table print configuration
    generate_plots = True
    print_std = False
    print_nan = True
    print_mode = 'csv'  # read, latex, csv
    sequences = [f"{s}_{seq}" for s in SCENES for seq in SEQUENCES]

    data = read_data()  # data[scene_sequence][param][drift][rollout_id]

    # Run.
    if generate_plots:
        for m in ["cluster_IoU", "cluster_Precision", "cluster_Recall"]:
            plot_surface(data, sequences, m)
    else:
        table(data, sequences, metric, print_std, print_nan, print_mode)


def read_data():
    data = {}
    complete = 0
    total = 0
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
                        data[name][p][d].append(
                            read_plot_data_csv(
                                os.path.join(DATA_PATH, f"{p}",
                                             f"{s}_{seq}_{d}{r}",
                                             "scores.csv")))
                        if len(data[name][p][d][-1]['cluster_IoU']) != 10:
                            print(
                                f"Incomplete data: '{p}/{s}_{seq}_{d}{r}' ({len(data[name][p][d][-1]['cluster_IoU'])}/10)."
                            )
                        else:
                            complete += 1
    print(f"Read data: {complete}/{total} entries complete.")
    return data


def plot_surface(data, sequences, metric):
    plt.rcParams.update({'font.size': 12})    
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})#, figsize=(15,10))

    # Make data.
    x_is_params = True
    reverse_x = False
    reverse_y = False

    # Plot the surface.
    x_it = PARAMS if x_is_params else DRIFT
    y_it = DRIFT if x_is_params else PARAMS
    if reverse_x:
        x_it = list(reversed(x_it))
    if reverse_y:
        y_it = list(reversed(y_it))
    X = range(len(x_it)) 
    Y = range(len(y_it))
    X, Y = np.meshgrid(X, Y)
    Z = np.zeros((len(x_it), len(y_it)))
    for i, x in enumerate(x_it):
        for j, y in enumerate(y_it):
            values = []
            for s in sequences:
                for rollout_data in data[s][x if x_is_params else y][y if x_is_params else x]:
                    values = values + rollout_data[metric]
            Z[j, i] = np.mean(values) * 100

    surf = ax.plot_surface(X, Y, Z, cmap=cm.winter,
                           linewidth=0, antialiased=False, rstride=1, cstride=1)  # winter, plasma, viridis, bone

    # Test fitting stuff.
    # x_fit = X.flatten()
    # y_fit = Y.flatten()
    # a = np.array([X*0+1, X, Y, X**2, X**2*Y, X**2*Y**2, Y**2, X*Y**2, X*Y]).T
    # B = Z.flatten()
    # coeff, r, rank, s = np.linalg.lstsq(A, B)
    # spacing = list(range(len(x_it)))
    # ax.plot3D(spacing, spacing, [Z[i,i] for i in spacing], 'k', linewidth=2)


    # Add a color bar which maps values to colors.
    # fig.colorbar(surf, shrink=0.5, aspect=5)
    ax.set_xticks(range(len(x_it)))
    ax.set_xticklabels((list(reversed(DRIFT)) if reverse_x else DRIFT ) if x_is_params else x_it)
    ax.set_yticks(range(len(y_it)))
    ax.set_yticklabels((list(reversed(DRIFT)) if reverse_y else DRIFT ) if not x_is_params else y_it)
    ax.set_xlabel("Drift Parameter" if x_is_params else "Drift Intensity")
    ax.set_ylabel("Drift Intensity" if x_is_params else "Drift Parameter")
    z_ticks = range(int(np.floor(np.min(Z)/10)*10),int(np.ceil(np.max(Z)/10)*10),10)
    ax.set_zticks(z_ticks)
    ax.set_zticklabels(z_ticks)
    if metric ==  "cluster_IoU": 
        ax.set_zlabel("IoU [%]")
    elif metric == "cluster_Precision":
        ax.set_zlabel("Precision [%]")
    else:
        ax.set_zlabel("Recall [%]")
    ax.xaxis.labelpad=15
    ax.yaxis.labelpad=15
    ax.zaxis.labelpad=5
    ax.view_init(elev=40, azim=50)
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_PATH, f"{metric}.png"), dpi=1000)


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
            msg = msg + (" $\pm$ " if print_mode == 'latex' else " +- ") + \
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
            entries.append(evaluate_row(np.array(values) * 100))
        print_row(entries)


if __name__ == "__main__":
    main()
