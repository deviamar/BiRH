#!/usr/bin/env bash
# Overnight exploration driver. Every phase is a resumable morpholab sweep; re-running this
# script skips completed jobs. Log: runs/overnight/overnight.log
cd /home/devi/BiRH || exit 1
J=${J:-20}
PY="python -m morpholab.cli"
mkdir -p runs/overnight/summary

phase() { local name=$1; shift; echo "=== $(date '+%F %T') START $name"; "$@"; local rc=$?; echo "=== $(date '+%F %T') END $name rc=$rc"; }
latest() { ls -td "runs/$1"/*/ 2>/dev/null | head -1; }

phase A-dt001 $PY sweep configs/overnight/A-convergence-dt001.yaml -j $J
phase A-dt002 $PY sweep configs/overnight/A-convergence-dt002.yaml -j $J
phase A-dt004 $PY sweep configs/overnight/A-convergence-dt004.yaml -j $J
phase C-ablation $PY sweep configs/studies/02-ablation.yaml -j $J
phase E-explore $PY sweep configs/overnight/E-explore.yaml -j $J
phase D-survey $PY sweep configs/overnight/D-survey.yaml -j $J
phase B-calibration $PY sweep configs/studies/01-calibration.yaml -j $J
phase F-generalization $PY sweep configs/studies/03-generalization.yaml -j $J
phase G-randomized $PY sweep configs/overnight/G-randomized.yaml -j $J

# Phase H: the 40 best hands from E, re-evaluated on every object and task.
E_DIR=$(latest overnight-E-explore)
python - "$E_DIR" <<'PYEOF'
import sys, yaml, pandas as pd
d = sys.argv[1]
df = pd.read_json(d + "/results.jsonl", lines=True)
g = (df.groupby("morphology_id")
       .agg(success=("success", "mean"), eps=("grasp_epsilon", "mean"), n=("success", "size"))
       .sort_values(["success", "eps"], ascending=False))
top = g.head(40).index.tolist()
g.to_csv("runs/overnight/summary/E-ranking.csv")
cfg = {"extends": "../base.yaml", "experiment": "overnight-H-top40",
       "sweep": {"morphologies": ["birh_v1", "two_finger"] + [f"runs/overnight/hands/{m}.json" for m in top],
                 "objects": ["all"], "tasks": ["all"], "trials": 5, "include_sim_metrics": False}}
open("configs/overnight/H-top40.yaml", "w").write("# Phase H -- top 40 hands from E on every object and task.\n" + yaml.safe_dump(cfg, sort_keys=False))
print("H config written from", d, "top:", top[:5])
PYEOF
phase H-top40 $PY sweep configs/overnight/H-top40.yaml -j $J

# Summaries
for exp in overnight-A-convergence-dt001 overnight-A-convergence-dt002 overnight-A-convergence-dt004 \
           02-ablation overnight-D-survey 01-calibration 03-generalization overnight-G-randomized; do
  d=$(latest $exp); [ -n "$d" ] || continue
  $PY analyze "$d" --failures --pareto --compare --csv runs/overnight/summary/$exp.csv > runs/overnight/summary/$exp.md 2>&1
done
for exp in overnight-E-explore overnight-H-top40; do
  d=$(latest $exp); [ -n "$d" ] || continue
  $PY analyze "$d" --by morphology_id --failures --pareto --csv runs/overnight/summary/$exp.csv > runs/overnight/summary/$exp.md 2>&1
done
echo "=== $(date '+%F %T') ALL DONE"
