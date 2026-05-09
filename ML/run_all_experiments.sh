#!/usr/bin/env bash
# Driver script for the full LOSO experiment grid.
# Runs all 11 experiments described in the brief and writes one PDF per experiment.
#
# Usage:
#   ./run_all_experiments.sh /path/to/NewTestData /path/to/output
#
# Each experiment uses the full notebook protocol:
#   - 11-subject LOSO (subjects auto-detected)
#   - All 5 CNN variants per fold (Baseline, Shallow, Deep, BN_GAP, WideKernel)
#   - 3-fold inner CV inside each fold's training set + final retrain + held-out test
#   - 8 epochs per training, batch_size=16
#   - Same data preprocessing as 1D_CNN_variants.ipynb (resample 90, Butterworth 10Hz/4, MinMax)
#
# Each experiment uses --state_file so an interrupted run can be resumed by
# simply re-invoking the same command — completed folds are skipped.

set -euo pipefail

# Defaults assume you're running from inside the ML/ directory:
#   ./run_all_experiments.sh
# Override either path with explicit args:
#   ./run_all_experiments.sh /path/to/data /path/to/output
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DATA_ROOT="${1:-${SCRIPT_DIR}/NewTestData/NewTestData}"
OUT_DIR="${2:-${SCRIPT_DIR}/loso_results}"
RUNNER="${SCRIPT_DIR}/loso_runner.py"
mkdir -p "$OUT_DIR"

run_exp () {
    local id="$1"; shift
    local label="$1"; shift
    echo
    echo "============================================================"
    echo "  Experiment $id"
    echo "  $label"
    echo "============================================================"
    python3 "$RUNNER" \
        --exp_id "$id" \
        --exp_label "$label" \
        --data_root "$DATA_ROOT" \
        --out_dir   "$OUT_DIR" \
        --epochs 8 --cv_folds 3 \
        --state_file "$OUT_DIR/${id}_state.json" \
        "$@"
}

# ──────────────────────────────────────────────────────────────────────────────
# Group A — Nothing-class ablation (full sensors, both hands)
# ──────────────────────────────────────────────────────────────────────────────
run_exp A1 "Full sensors (flex+ypr+accel, both hands), INCLUDING Double_Nothing class" \
    --use_flex --use_ypr --use_accel --use_left --use_right --include_nothing

run_exp A2 "Full sensors (flex+ypr+accel, both hands), EXCLUDING Double_Nothing class" \
    --use_flex --use_ypr --use_accel --use_left --use_right

# ──────────────────────────────────────────────────────────────────────────────
# Group B — Sensor-modality ablation, Nothing excluded, both hands
# ──────────────────────────────────────────────────────────────────────────────
run_exp B_flex       "Flex only (both hands), no Nothing" \
    --use_flex --use_left --use_right
run_exp B_ypr        "YPR only (both hands), no Nothing" \
    --use_ypr --use_left --use_right
run_exp B_accel      "Accelerometers only (both hands), no Nothing" \
    --use_accel --use_left --use_right
run_exp B_flex_ypr   "Flex + YPR (both hands), no Nothing" \
    --use_flex --use_ypr --use_left --use_right
run_exp B_flex_accel "Flex + Accel (both hands), no Nothing" \
    --use_flex --use_accel --use_left --use_right
run_exp B_ypr_accel  "YPR + Accel (both hands), no Nothing" \
    --use_ypr --use_accel --use_left --use_right
# (B_flex_ypr_accel == A2, no need to re-run)

# ──────────────────────────────────────────────────────────────────────────────
# Group C — Hand laterality (full sensors, no Nothing)
# ──────────────────────────────────────────────────────────────────────────────
run_exp C_left  "Full sensors, LEFT hand only, no Nothing" \
    --use_flex --use_ypr --use_accel --use_left
run_exp C_right "Full sensors, RIGHT hand only, no Nothing" \
    --use_flex --use_ypr --use_accel --use_right

echo
echo "All experiments complete. PDFs and JSONs in $OUT_DIR"
ls -la "$OUT_DIR"/*.pdf 2>/dev/null || true
