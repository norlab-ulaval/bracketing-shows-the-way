#!/usr/bin/env bash
# Run BVSLAM (paper configuration) on one bracketed stereo sequence.
#
#   ./scripts/run_demo.sh [--native] [SEQUENCE_DIR] [CALIBRATION_DIR]
#
# SEQUENCE_DIR     folder with images_left/, images_right/, images_meta_left/, images_meta_right/
#                  (default: the sample B-Adapt tunnel exit, data/bstw_sample/tunnel_exit_b_adapt)
# CALIBRATION_DIR  folder with stereo_calibration_{left,right}.yaml
#                  (default: <SEQUENCE_DIR>/calibration if it exists, else data/bstw_sample/calibration)
# --native         use the local uv environment (cd bvslam && uv sync --frozen) instead of Docker
#
# Writes output/<sequence name>.tum and <name>_global_ba.tum. A live Rerun view opens by default (desktop window natively,
# http://localhost:9090 with Docker); set BVSLAM_VIZ=none to disable it.
set -euo pipefail
cd "$(dirname "$0")/.."

MODE=docker
if [ "${1:-}" = --native ]; then MODE=native; shift; fi

if [ $# -eq 0 ]; then
  [ -d data/bstw_sample ] || ./scripts/download_sample.sh
  set -- data/bstw_sample/tunnel_exit_b_adapt
fi
SEQUENCE=$(realpath "$1")
if [ $# -ge 2 ]; then CALIBRATION=$(realpath "$2")
elif [ -d "${SEQUENCE}/calibration" ]; then CALIBRATION="${SEQUENCE}/calibration"
elif [ -d data/bstw_sample/calibration ]; then CALIBRATION=$(realpath data/bstw_sample/calibration)
else echo "No calibration found: pass CALIBRATION_DIR as the second argument." >&2; exit 1
fi
NAME=$(basename "${SEQUENCE}")
mkdir -p output

echo "=== BVSLAM on ${SEQUENCE} (${MODE}) ==="
if [ "${MODE}" = native ]; then
  (cd bvslam && uv run --frozen bvslam-run \
      --data-dir "${SEQUENCE}" --calibration-dir "${CALIBRATION}" \
      --config configs/paper.yaml --viz "${BVSLAM_VIZ:-auto}" --out "../output/${NAME}.tum")
else
  SERVICE=bvslam-cpu
  docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia' && SERVICE=bvslam
  docker compose build "${SERVICE}"
  docker compose run --rm --service-ports --user "$(id -u):$(id -g)" \
      -v "${SEQUENCE}:/input/sequence:ro" -v "${CALIBRATION}:/input/calibration:ro" \
      "${SERVICE}" bvslam-run \
      --data-dir /input/sequence --calibration-dir /input/calibration \
      --config configs/paper.yaml --viz "${BVSLAM_VIZ:-auto}" --out "/output/${NAME}.tum"
fi
echo "Results written to output/${NAME}*"
