#!/usr/bin/env bash
# Re-run the coverage certificate for both camera sets with the corrected
# measurement (test point rides the stand tiers; pillar footprints excluded).
#
# Runs coverage.py in its OWN throw-away container so it never contends with
# the streaming GUI for the cache lock (see docs/ISAAC_SIM_ON_EC2.md §2).
# Usage on the box:  bash ~/gameday-twin/sim/remeasure.sh
set -euo pipefail
cd ~/gameday-twin
sudo chown -R 1234:1234 ~/docker/isaac-sim results 2>/dev/null || true
sudo docker rm -f isaac-sim 2>/dev/null || true          # no GUI while measuring
for SET in A B; do
  echo "== coverage set $SET (terrain mode) =="
  sudo docker run --rm --gpus all --network=host \
    -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
    -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v "$HOME/gameday-twin:/workspace:rw" \
    --entrypoint /isaac-sim/python.sh nvcr.io/nvidia/isaac-sim:6.0.1 \
    /workspace/sim/coverage.py --venue /workspace/results/venue_before.usd \
      --set "$SET" --target-mode terrain --out "/workspace/results/coverage_${SET}_terrain.json" \
    2>&1 | grep -E "^\[C|union|regions|Traceback|Error\]" || true
done
echo "== done =="; ls -la results/coverage_*_terrain.json
