#!/usr/bin/env bash
# Exact command order for one full run on the GPU box.
# Usage (on the box):  cd ~/gameday-twin && bash sim/run_all.sh
set -euo pipefail
ISAAC=${ISAAC:-/opt/IsaacSim}
PY="$ISAAC/python.sh"
cd "$(dirname "$0")/.."
mkdir -p results

echo "== 1. venue (before: G3 closed) =="
$PY sim/build_venue.py --out results/venue_before.usd
echo "== 1b. venue (after: G3 open) =="
$PY sim/build_venue.py --out results/venue_after.usd --open-gates G3

echo "== 2. coverage certificate, camera set A and B (venue_before) =="
$PY sim/coverage.py --venue results/venue_before.usd --set A --out results/coverage_A.json --stills
$PY sim/coverage.py --venue results/venue_before.usd --set B --out results/coverage_B.json --stills

echo "== 3. surge density before/after (same seed) =="
if [ -f sim/density.py ]; then
  $PY sim/density.py --venue results/venue_before.usd --out results/surge_before.json --seed 42
  $PY sim/density.py --venue results/venue_after.usd  --out results/surge_after.json  --seed 42
else
  echo "density.py not present yet — skipping step 3"
fi
echo "== done =="
ls -la results
