import csv
import os
import numpy as np


def read_score_data(csv_file):
    """
    Read a single CSV file of a run (tsdf or ssc) and turn it into a dictionary.
    """
    data = {}
    if not os.path.isfile(csv_file):
        print(f"File '{csv_file}' does not exist!")
        return data
    with open(csv_file, newline='\n') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        header = next(reader)
        for key in header:
            data[key] = []

        for row in reader:
            for i, r in enumerate(row):
                data[header[i]].append(float(r))
    return data


def verify_data(data, names, expected_num_entries=10):
    num_incomplete = 0
    for i, d in enumerate(data):
        if not "timestamp" in d:
            print(f"Warning: failed to read data for '{names[i]}'.")
            num_incomplete = num_incomplete + 1
            continue
        num_entries = len(d["timestamp"])
        if num_entries < expected_num_entries:
            print(
                f"Warning: Incomplete data for '{names[i]}' ({num_entries}/{expected_num_entries})."
            )
            num_incomplete = num_incomplete + 1
    num_samples = len(data)
    print(
        f"{num_samples-num_incomplete}/{num_samples} data entries are complete."
    )


def get_grid(data, field):
    if field not in data:
        print(f"Warning: Did not fiend field '{field}' to create grid from.")
        return np.array([np.nan])
    return np.array(data[field])


def read_cloud_data(csv_file):
    """ Returns data[field_name][point_index] = value. """
    data = {}
    if not os.path.isfile(csv_file):
        print(f"File '{csv_file}' does not exist!")
        return data
    with open(csv_file, newline='\n') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        header = None
        for row in reader:
            if header is None:
                header = row
                for key in header:
                    data[key] = []
            else:
                for i, r in enumerate(row):
                    data[header[i]].append(float(r))
    return data


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
        data[key] = {
            'calls': calls,
            'total': total,
            'mean': mean,
            'std': std,
            'min': min,
            'max': max
        }
    return data