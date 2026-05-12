#!/usr/bin/env bash
# Driver script for the full LOSO experiment grid using loso_runner_glove.py
# (the sklearn / glove_ml_explorer.ipynb pipeline).
#
# Usage:
#   ./run_all_glove_experiments.sh /path/to/NewTestData /path/to/output
#
# Each experiment uses the full notebook protocol:
#   - LOSO across all auto-detected subjects (must have <subject>/Static/<gesture>/*.csv)
#   - 8 classifiers per fold (Gradient Boosting omitted to keep runtime sane):
#       SVM(RBF), SVM(Linear), Random Forest, KNN,
#       Logistic Regression, LDA, Voting (soft), Stacking
#   - 5-fold inner CV inside each fold's training set + final retrain + held-out test
#   - Static-gesture preprocessing (resample 90, optional Butterworth, MinMax)
#   - Feature mode: stats+fft (matches notebook default)
#
# Each experiment uses --state_file so an interrupted run can be resumed by
# simply re-invoking the same command — completed folds are skipped.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DATA_ROOT="${1:-${SCRIPT_DIR}/NewTestData}"
OUT_DIR="${2:-${SCRIPT_DIR}/loso_static_reports}"
RUNNER="${SCRIPT_DIR}/loso_runner_static.py"
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
        --cv_folds 5 \
        --classifiers "SVM (RBF),SVM (Linear),Random Forest,KNN,Logistic Regression,LDA,Voting Soft (SVM+RF+KNN+LogReg),Stacking (SVM+RF+KNN+LogReg)" \
        --state_file "$OUT_DIR/${id}_state.json" \
        "$@"
}

# ──────────────────────────────────────────────────────────────────────────────
# Group A — Nothing-class ablation (full sensors, both hands)
# ──────────────────────────────────────────────────────────────────────────────
run_exp SA1 "Static · full sensors (flex+ypr+accel, both hands), INCLUDING Double_Nothing" \
    --use_flex --use_ypr --use_accel --hands left,right --include_nothing

run_exp SA2 "Static · full sensors (flex+ypr+accel, both hands), EXCLUDING Double_Nothing" \
    --use_flex --use_ypr --use_accel --hands left,right

# ──────────────────────────────────────────────────────────────────────────────
# Group B — Sensor-modality ablation, Nothing excluded, both hands
# ──────────────────────────────────────────────────────────────────────────────
# run_exp SB_flex       "Static · flex only (both hands), no Nothing" \
#     --use_flex --hands left,right
# run_exp SB_ypr        "Static · YPR only (both hands), no Nothing" \
#     --use_ypr --hands left,right
# run_exp SB_accel      "Static · accelerometers only (both hands), no Nothing" \
#     --use_accel --hands left,right
# run_exp SB_flex_ypr   "Static · flex + YPR (both hands), no Nothing" \
#     --use_flex --use_ypr --hands left,right
# run_exp SB_flex_accel "Static · flex + accel (both hands), no Nothing" \
#     --use_flex --use_accel --hands left,right
# run_exp SB_ypr_accel  "Static · YPR + accel (both hands), no Nothing" \
#     --use_ypr --use_accel --hands left,right
# (SB_flex_ypr_accel == SA2 — already covered)

# ──────────────────────────────────────────────────────────────────────────────
# Group C — Hand laterality (full sensors, no Nothing)
# ──────────────────────────────────────────────────────────────────────────────
# run_exp SC_left  "Static · full sensors, LEFT hand only, no Nothing" \
#     --use_flex --use_ypr --use_accel --hands left
run_exp SC_right "Static · full sensors, RIGHT hand only, no Nothing" \
    --use_flex --use_ypr --use_accel --hands right

# ──────────────────────────────────────────────────────────────────────────────
# Group D — Feature-mode ablation on SA2 config (full sensors, both hands, no Nothing)
# ──────────────────────────────────────────────────────────────────────────────

# run_exp SD_flatten "Static · SA2 sensors, flattened raw time series" \
#     --use_flex --use_ypr --use_accel --hands left,right \
#     --feature_mode flatten

echo
echo "All experiments complete. PDFs and JSONs in $OUT_DIR"
ls -la "$OUT_DIR"/*.pdf 2>/dev/null || true
