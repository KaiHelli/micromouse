#!/usr/bin/env python3
import pandas as pd
import argparse
import sys

def invert_columns(input_path: str, output_path: str) -> None:
    # Load CSV (assumes no index column in the file)
    df = pd.read_csv(input_path, header=None)

    # Check we have exactly 3 columns
    if df.shape[1] != 3:
        sys.exit(f"Error: Expected 3 columns, but found {df.shape[1]}.")

    # Multiply first and last columns by -1
    # Use .iloc to refer to them by position
    df.iloc[:, 0] = df.iloc[:, 0] * -1
    df.iloc[:, 2] = df.iloc[:, 2] * -1

    # Write out to new CSV (preserve index=False by default)
    df.to_csv(output_path, index=False, header=False)
    print(f"Processed file saved to: {output_path}")
    
invert_columns("mag3_raw.csv", "mag3_inverted.csv")