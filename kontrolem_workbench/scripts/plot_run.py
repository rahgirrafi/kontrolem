#!/usr/bin/env python3
"""Plot a `kontrolem_setup analyze --csv` trace.

Usage:  plot_run.py <trace.csv> [out.png]

Three stacked panels: configuration vs its reference, applied torques, and the
law's health (status ok + trust margin). Needs matplotlib; everything else in
the workbench is dependency-free, so this stays a separate optional script.
"""
import csv
import sys


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("plot_run.py needs matplotlib:  pip install matplotlib")
        return 1

    path = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else path.rsplit(".", 1)[0] + ".png"

    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print(f"{path}: empty trace")
        return 1
    cols = rows[0].keys()
    q_cols = sorted(c for c in cols if c.startswith("q") and c[1:].isdigit())
    tau_cols = sorted(c for c in cols if c.startswith("tau") and c[3:].isdigit())
    t = [float(r["t"]) for r in rows]

    fig, (ax_q, ax_tau, ax_ok) = plt.subplots(
        3, 1, sharex=True, figsize=(9, 8),
        gridspec_kw={"height_ratios": [3, 2, 1]})

    for c in q_cols:
        (line,) = ax_q.plot(t, [float(r[c]) for r in rows], label=c)
        ax_q.plot(t, [float(r["qdes" + c[1:]]) for r in rows], "--",
                  color=line.get_color(), alpha=0.5)
    ax_q.set_ylabel("configuration (solid) vs reference (dashed)")
    ax_q.legend(loc="best", fontsize=8)
    ax_q.grid(alpha=0.3)

    for c in tau_cols:
        ax_tau.plot(t, [float(r[c]) for r in rows], label=c)
    ax_tau.set_ylabel("applied torque [Nm]")
    ax_tau.legend(loc="best", fontsize=8)
    ax_tau.grid(alpha=0.3)

    ax_ok.plot(t, [float(r["margin"]) for r in rows], label="margin")
    ax_ok.fill_between(t, 0, [int(r["ok"]) for r in rows],
                       alpha=0.15, step="pre", label="status ok")
    ax_ok.axhline(0.0, color="k", lw=0.5)
    ax_ok.set_ylabel("trust")
    ax_ok.set_xlabel("t [s]")
    ax_ok.legend(loc="best", fontsize=8)
    ax_ok.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
