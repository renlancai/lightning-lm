#!/bin/bash
# tests/regression/capture_baseline.sh
#
# Capture baseline_traj.tum BEFORE the first refactoring commit.
# Run once; result goes into git.
#
# Usage:
#   BAG=/data/nclt.db3 CONFIG=config/default_nclt.yaml ./capture_baseline.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

BAG="${BAG:-}"
CONFIG="${CONFIG:-${REPO_ROOT}/config/default_nclt.yaml}"
BINARY="${BINARY:-${REPO_ROOT}/bin/run_slam_offline}"
OUTPUT="${SCRIPT_DIR}/baseline_traj.tum"

if [[ -z "${BAG}" ]]; then
    echo "[ERROR] Set BAG=/path/to/test.db3" >&2; exit 1
fi
if [[ ! -f "${BINARY}" ]]; then
    echo "[ERROR] Binary not found: ${BINARY}" >&2; exit 1
fi

echo "[INFO] Capturing baseline trajectory → ${OUTPUT}"
"${BINARY}" \
    --config "${CONFIG}" \
    --input_bag "${BAG}" \
    --output_traj "${OUTPUT}"

LINES=$(wc -l < "${OUTPUT}")
echo "[INFO] Done. ${LINES} frames written to ${OUTPUT}"
echo "[INFO] Add to git: git add ${OUTPUT}"
