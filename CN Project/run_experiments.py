"""Run the full experiment plan head-less and export the results.

Usage::

    python run_experiments.py [--duration 90] [--outdir results]

The command writes ``drone_swarm_results.csv`` plus the comparison charts
that are used in the project report.
"""

from __future__ import annotations

import argparse
import os

from drone_swarm.experiments import export_csv, plot_results, run_all_experiments


def main() -> None:
    parser = argparse.ArgumentParser(description="Drone swarm network experiments")
    parser.add_argument("--duration", type=float, default=90.0,
                        help="length of every experiment in simulated seconds")
    parser.add_argument("--outdir", type=str, default="results",
                        help="directory for the CSV file and the charts")
    args = parser.parse_args()

    def progress(name: str, fraction: float) -> None:
        print(f"[{fraction * 100:5.1f}%] {name}")

    print("Running drone swarm network experiments ...")
    results = run_all_experiments(duration=args.duration, outdir=args.outdir,
                                  progress=progress)

    csv_path = os.path.join(args.outdir, "drone_swarm_results.csv")
    export_csv(results, csv_path)
    print(f"\nResults written to {csv_path}")

    chart_paths = plot_results(results, outdir=args.outdir)
    for path in chart_paths:
        print(f"Chart written to {path}")

    header = f"{'experiment':38s} {'PDR%':>7s} {'loss%':>7s} {'lat ms':>8s} {'B/s':>8s} {'links':>6s}"
    print("\n" + header)
    print("-" * len(header))
    for result in results:
        metrics = result.metrics
        print(f"{result.name:38s} {metrics['pdr_percent']:7.2f} "
              f"{metrics['loss_percent']:7.2f} {metrics['avg_latency_ms']:8.2f} "
              f"{metrics['throughput_bytes_s']:8.2f} {metrics['active_links']:6d}")


if __name__ == "__main__":
    main()
