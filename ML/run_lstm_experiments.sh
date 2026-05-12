#!/usr/bin/env bash
# Driver script for the LSTM-only LOSO experiment grid.
# Runs the same Group B/C experiments but only trains CNN_LSTM and CNN_BiLSTM
# per fold (skips the 5 pure-CNN variants).
#
# Usage:
#   ./run_all_experiments.sh /path/to/NewTestData /path/to/output
#
# Each experiment uses the full notebook protocol:
#   - 11-subject LOSO (subjects auto-detected)
#   - 2 variants per fold: CNN_LSTM, CNN_BiLSTM
#   - 3-fold inner CV inside each fold's training set + final retrain + held-out test
#   - 8 epochs per training, batch_size=16
#   - Same data preprocessing as 1D_CNN_variants_lstm.ipynb
#     (resample 90, Butterworth 10Hz/4, MinMax)
#
# Each experiment uses --state_file so an interrupted run can be resumed by
# simply re-invoking the same command — completed folds are skipped.
# IDs are suffixed with `_lstm` so they don't collide with the earlier
# 5-CNN-variant state files.

set -euo pipefail

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
        --variants CNN_LSTM,CNN_BiLSTM \
        --state_file "$OUT_DIR/${id}_state.json" \
        "$@"
}

# ──────────────────────────────────────────────────────────────────────────────
# Group A — Nothing-class ablation (full sensors, both hands)
# ──────────────────────────────────────────────────────────────────────────────
# run_exp A1_lstm "LSTM only · Full sensors (flex+ypr+accel, both hands), INCLUDING Double_Nothing" \
#     --use_flex --use_ypr --use_accel --use_left --use_right --include_nothing

# run_exp A2_lstm "LSTM only · Full sensors (flex+ypr+accel, both hands), EXCLUDING Double_Nothing" \
#     --use_flex --use_ypr --use_accel --use_left --use_right

# ──────────────────────────────────────────────────────────────────────────────
# Group B — Sensor-modality ablation, Nothing excluded, both hands
# ──────────────────────────────────────────────────────────────────────────────
run_exp B_flex_lstm       "LSTM only · Flex only (both hands), no Nothing" \
    --use_flex --use_left --use_right
run_exp B_ypr_lstm        "LSTM only · YPR only (both hands), no Nothing" \
    --use_ypr --use_left --use_right
run_exp B_accel_lstm      "LSTM only · Accelerometers only (both hands), no Nothing" \
    --use_accel --use_left --use_right
run_exp B_flex_ypr_lstm   "LSTM only · Flex + YPR (both hands), no Nothing" \
    --use_flex --use_ypr --use_left --use_right
run_exp B_flex_accel_lstm "LSTM only · Flex + Accel (both hands), no Nothing" \
    --use_flex --use_accel --use_left --use_right
run_exp B_ypr_accel_lstm  "LSTM only · YPR + Accel (both hands), no Nothing" \
    --use_ypr --use_accel --use_left --use_right

# ──────────────────────────────────────────────────────────────────────────────
# Group C — Hand laterality (full sensors, no Nothing)
# ──────────────────────────────────────────────────────────────────────────────
run_exp C_left_lstm  "LSTM only · Full sensors, LEFT hand only, no Nothing" \
    --use_flex --use_ypr --use_accel --use_left
run_exp C_right_lstm "LSTM only · Full sensors, RIGHT hand only, no Nothing" \
    --use_flex --use_ypr --use_accel --use_right

echo
echo "All LSTM experiments complete. PDFs and JSONs in $OUT_DIR"
ls -la "$OUT_DIR"/*_lstm_*.pdf 2>/dev/null || true