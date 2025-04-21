#!/usr/bin/env python3
import numpy as np

def compute_slope(csv_path):
    # Load data: assumes two columns, x and y, separated by commas
    data = np.loadtxt(csv_path, delimiter=',')
    x = data[:, 0]
    y = data[:, 1]

    # Fit a 1st-degree polynomial (straight line): y = m*x + b
    m, b = np.polyfit(x, y, 1)
    return m

if __name__ == "__main__":
    slope = compute_slope("/Users/kai.helli/MPLABXProjects/micromouseLC.X/full_acceleration.csv")
    print(f"Slope of fitted line: {slope}")
