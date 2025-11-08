"""Visualize LiDAR sweep data stored in the CSV produced by DataCapture.py.

Example usage:
    python scripts/PlotScan.py scans.csv --scan 0
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Iterable, List, Tuple

import matplotlib.pyplot as plt
import numpy as np


# Type alias for a Cartesian point in millimetres (or converted units).
Point = Tuple[float, float]


def parse_args() -> argparse.Namespace:
    """Create the CLI argument parser and return parsed arguments."""
    parser = argparse.ArgumentParser(
        description="Plot LiDAR sweep points from the CSV produced by DataCapture.py."
    )
    parser.add_argument(
        "csv_path",
        type=Path,
        help="Path to the scans CSV (e.g., scans.csv).",
    )
    # The CSV can contain multiple scans; allow the caller to select which one.
    parser.add_argument(
        "--scan",
        type=int,
        default=0,
        help="Scan index to visualize. Defaults to the first (0).",
    )
    # Provide both millimetres (native) and metres for readability.
    parser.add_argument(
        "--units",
        choices=("mm", "m"),
        default="mm",
        help="Display coordinates in millimeters (default) or meters.",
    )
    return parser.parse_args()


def load_scan_points(path: Path, scan_idx: int) -> Tuple[List[Point], List[float]]:
    """Load (x, y) points plus distances for a single scan."""
    if not path.exists():
        raise FileNotFoundError(f"CSV file not found: {path}")

    points: List[Point] = []
    distances: List[float] = []

    with path.open(newline="") as fh:
        reader = csv.DictReader(fh)
        # Ensure the CSV contains the fields emitted by DataCapture.py.
        required_columns = {
            "scan_idx",
            "angle_deg",
            "distance_mm",
            "quality",
            "x_mm",
            "y_mm",
        }
        missing = required_columns - set(reader.fieldnames or [])
        if missing:
            missing_cols = ", ".join(sorted(missing))
            raise ValueError(f"CSV file is missing required columns: {missing_cols}")

        for row in reader:
            # Filter rows to the selected scan index.
            if int(row["scan_idx"]) != scan_idx:
                continue
            try:
                x = float(row["x_mm"])
                y = float(row["y_mm"])
                dist = float(row["distance_mm"])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"Invalid numeric value in row: {row}") from exc

            points.append((x, y))
            distances.append(dist)

    if not points:
        raise ValueError(
            f"No points found for scan_idx={scan_idx}. "
            "Verify the index and that the CSV contains that scan."
        )

    return points, distances


def convert_units(points: Iterable[Point], distances: Iterable[float], units: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Convert units if needed, returning numpy arrays for plotting.
    Units are either millimeters or meters.
    """
    points_array = np.array(list(points), dtype=float)
    dist_array = np.array(list(distances), dtype=float)

    # Convert to metres on demand; otherwise stay in millimetres.
    if units == "m":
        points_array /= 1000.0
        dist_array /= 1000.0

    x_vals = points_array[:, 0]
    y_vals = points_array[:, 1]
    return x_vals, y_vals, dist_array


def plot_scan(
    points: List[Point],
    distances: List[float],
    args: argparse.Namespace,
) -> None:
    """Render a scatter plot of the LiDAR scan."""
    x_vals, y_vals, dist_vals = convert_units(points, distances, args.units)
    # Convert to polar coordinates for plotting; angle is measured from the +x axis.
    angles = np.arctan2(y_vals, x_vals)
    radii = dist_vals

    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw={"projection": "polar"})
    # Each point represents a single LiDAR measurement; the scanner sits at the origin.
    ax.scatter(angles, radii, s=10, color="tab:blue")

    ax.set_title(f"LiDAR Scan #{args.scan}")
    unit_label = args.units
    ax.set_rlabel_position(135)  # Move radial labels out of the dense region.
    ax.set_ylabel(f"Radius ({unit_label})")

    # Show concentric rings and angular spokes to highlight the scanner-centric layout.
    ax.grid(True, linestyle=":", linewidth=0.5)

    fig.tight_layout()
    plt.show()


def main() -> None:
    """Entry-point: parse arguments, load the scan, and plot it."""
    args = parse_args()
    points, distances = load_scan_points(args.csv_path, args.scan)
    plot_scan(points, distances, args)


if __name__ == "__main__":
    main()

