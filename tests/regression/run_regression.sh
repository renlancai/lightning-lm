#!/bin/bash
# tests/regression/run_regression.sh
#
# Compare a freshly-built binary's trajectory against the pre-refactor baseline.
# Run this after each Phase gate before merging.
#
# Usage:
#   BAG=/data/nclt.db3 CONFIG=config/default_nclt.yaml ./run_regression.sh
#
# Dependencies:
#   evo  (pip install evo)
#   python3

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

BAG="${BAG:-}"
CONFIG="${CONFIG:-${REPO_ROOT}/config/default_nclt.yaml}"
BINARY="${BINARY:-${REPO_ROOT}/bin/run_slam_offline}"
BASELINE="${SCRIPT_DIR}/baseline_traj.tum"
TRAJ_OUT="${TMPDIR:-/tmp}/refactored_traj_$$.tum"
RESULT_ZIP="${TMPDIR:-/tmp}/regression_result_$$.zip"
MAX_RMSE="${MAX_RMSE:-0.05}"  # 50 mm — CI hard gate

# ── Pre-flight checks ──────────────────────────────────────────────────────
if [[ -z "${BAG}" ]]; then
    echo "[ERROR] Set BAG=/path/to/test.db3" >&2; exit 1
fi
if [[ ! -f "${BINARY}" ]]; then
    echo "[ERROR] Binary not found: ${BINARY}" >&2; exit 1
fi
if [[ ! -f "${BASELINE}" ]]; then
    echo "[ERROR] Baseline trajectory not found: ${BASELINE}" >&2
    echo "        Run capture_baseline.sh first (Phase 0)." >&2
    exit 1
fi
if ! command -v evo_res &>/dev/null; then
    echo "[ERROR] evo not installed. Run: pip install evo" >&2; exit 1
fi

# ── Run refactored binary ──────────────────────────────────────────────────
echo "[INFO] Running SLAM on ${BAG} ..."
"${BINARY}" \
    --config "${CONFIG}" \
    --input_bag "${BAG}" \
    --output_traj "${TRAJ_OUT}"

if [[ ! -s "${TRAJ_OUT}" ]]; then
    echo "[ERROR] Trajectory output is empty." >&2; exit 1
fi

LINES=$(wc -l < "${TRAJ_OUT}")
echo "[INFO] Trajectory frames: ${LINES}"
if [[ "${LINES}" -lt 100 ]]; then
    echo "[ERROR] Too few frames (${LINES} < 100)" >&2; exit 1
fi

# ── Compute RMSE vs baseline ───────────────────────────────────────────────
echo "[INFO] Computing trajectory diff (evo_res) ..."
evo_res "${TRAJ_OUT}" "${BASELINE}" \
    --align --correct_scale \
    --save_results "${RESULT_ZIP}" 2>&1

RMSE=$(python3 - <<'EOF'
import sys, zipfile, json, os
zpath = os.environ.get("RESULT_ZIP", "")
if not zpath:
    sys.exit("RESULT_ZIP not set")
with zipfile.ZipFile(zpath) as z:
    with z.open("stats.json") as f:
        data = json.load(f)
print(data["rmse"])
EOF
)

export RESULT_ZIP
RMSE=$(python3 - <<'EOF'
import sys, zipfile, json, os
zpath = os.environ["RESULT_ZIP"]
with zipfile.ZipFile(zpath) as z:
    with z.open("stats.json") as f:
        data = json.load(f)
print(data["rmse"])
EOF
)

echo "[INFO] Trajectory RMSE diff: ${RMSE} m  (gate: ${MAX_RMSE} m)"

PASS=$(python3 -c "import sys; sys.exit(0 if float('${RMSE}') < float('${MAX_RMSE}') else 1)" && echo yes || echo no)

rm -f "${TRAJ_OUT}" "${RESULT_ZIP}"

if [[ "${PASS}" == "yes" ]]; then
    echo "[PASS] Regression test PASSED (RMSE ${RMSE} < ${MAX_RMSE} m)"
    exit 0
else
    echo "[FAIL] Regression test FAILED — algorithm behaviour may have changed!"
    echo "       RMSE ${RMSE} >= ${MAX_RMSE} m"
    exit 1
fi
