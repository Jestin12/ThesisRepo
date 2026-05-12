#!/usr/bin/env bash
# Driver script for traditional-classifier LOSO experiments on DYNAMIC data,
# using raw data points (--feature_mode flatten) as features.
#
# Two experiments only:
#   DA1 — full sensors (flex + ypr + accel, both hands), INCLUDE Double_Nothing
#   DA2 — full sensors (flex + ypr + accel, both hands), EXCLUDE Double_Nothing
#
# Each experiment uses the full notebook protocol:
#   - LOSO across all auto-detected subjects (must have <subject>/Dynamic/<gesture>/*.csv)
#   - 8 classifiers per fold (Gradient Boosting omitted to keep runtime sane):
#       SVM(RBF), SVM(Linear), Random Forest, KNN,
#       Logistic Regression, LDA, Voting (soft), Stacking
#   - 5-fold inner CV inside each fold's training set + final retrain + held-out test
#   - Resample 90, optional Butterworth, MinMax on raw flattened features
#
# Each experiment uses --state_file so an interrupted run can be resumed by
# simply re-invoking the same command — completed folds are skipped.
#
# Usage:
#   ./run_dynamic_classical_experiments.sh /path/to/NewTestData /path/to/output

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DATA_ROOT="${1:-${SCRIPT_DIR}/NewTestData}"
OUT_DIR="${2:-${SCRIPT_DIR}/loso_results_dynamic_classical}"
RUNNER="${SCRIPT_DIR}/loso_runner_glove.py"
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
        --gesture_set dynamic \
        --feature_mode flatten \
        --cv_folds 5 \
        --classifiers "SVM (RBF),SVM (Linear),Random Forest,KNN,Logistic Regression,LDA,Voting Soft (SVM+RF+KNN+LogReg),Stacking (SVM+RF+KNN+LogReg)" \
        --state_file "$OUT_DIR/${id}_state.json" \
        "$@"
}

# ──────────────────────────────────────────────────────────────────────────────
# Group D (dynamic) — Nothing-class ablation, full sensors, both hands, raw features
# ──────────────────────────────────────────────────────────────────────────────
run_exp DA1 "Dynamic · raw features · full sensors (flex+ypr+accel, both hands), INCLUDING Double_Nothing" \
    --use_flex --use_ypr --use_accel --hands left,right --include_nothing

run_exp DA2 "Dynamic · raw features · full sensors (flex+ypr+accel, both hands), EXCLUDING Double_Nothing" \
    --use_flex --use_ypr --use_accel --hands left,right

echo
echo "All dynamic classical-classifier experiments complete. PDFs and JSONs in $OUT_DIR"
ls -la "$OUT_DIR"/*.pdf 2>/dev/null || true
