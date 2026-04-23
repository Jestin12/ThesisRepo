"""
filter_pipeline.py
==================
Batch filter pipeline for glove sensor data.

Scans a folder (recursively) for CSV files, applies the configured
filter to every numeric sensor column for the selected hand, and
writes a filtered copy of each file to an output folder — preserving
the original subfolder structure.

Usage
-----
    python filter_pipeline.py

Edit the CONFIG block below to change input folder, filter type,
hand, sampling rate, and output location.

Output file naming
------------------
    <output_dir>/<relative_subfolder>/<original_stem>_<filter_tag>.csv

Example:
    data/gestures/fist/trial_001.csv
    →  data/filtered/fist/trial_001_butterworth_lp.csv
"""

import os
import glob
import shutil
import logging
import numpy as np
import pandas as pd
from scipy import signal as sp_signal

# =============================================================================
# CONFIG
# =============================================================================

# Folder to scan for CSV files (absolute or relative to this script).
INPUT_DIR = '../ML/TwoHandDynamic/TwoHandDynamic_L_Wiggle_R_Wiggle'

# Where to write filtered outputs.
# Set to None to write alongside each source file (adds filter tag to filename).
OUTPUT_DIR = '../ML/TwoHandDynamic/TwoHandDynamic_L_Wiggle_R_Wiggle_filtered_butterworth_lp'

# Which hand's columns to filter.
# Options: 'left' | 'right' | 'both'
HAND = 'both'

# Approximate sampling rate in Hz — used for Butterworth cutoff normalisation.
FS_HZ = 22.0

# Whether to also write a summary CSV listing every file processed.
WRITE_SUMMARY = True

# ---------------------------------------------------------------------------
# FILTER SELECTION
# ---------------------------------------------------------------------------
# Options:
#   'none'           — copy file unchanged
#   'butterworth_lp' — Butterworth low-pass
#   'butterworth_hp' — Butterworth high-pass
#   'butterworth_bp' — Butterworth band-pass
#   'moving_average' — simple moving average
#   'savgol'         — Savitzky-Golay
#   'median'         — median filter

FILTER_TYPE = 'butterworth_lp'

# Butterworth parameters
BW_ORDER      = 4      # filter order
BW_CUTOFF_LO  = 5.0   # low-pass / band-pass lower cutoff (Hz)
BW_CUTOFF_HI  = 10.0  # band-pass upper cutoff (Hz) — ignored for lp/hp
BW_ZERO_PHASE = True   # True = filtfilt (zero-phase), False = lfilter (causal)

# Moving average parameters
MA_WINDOW = 5          # window in samples

# Savitzky-Golay parameters
SG_WINDOW    = 11      # window in samples (must be odd)
SG_POLYORDER = 3       # polynomial order (must be < SG_WINDOW)

# Median filter parameters
MED_KERNEL = 5         # kernel size in samples (must be odd)

# =============================================================================
# LOGGING
# =============================================================================

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s  %(levelname)-8s  %(message)s',
    datefmt='%H:%M:%S',
)
log = logging.getLogger(__name__)

# =============================================================================
# FILTER ENGINE
# =============================================================================

def apply_filter(sig: np.ndarray, filter_type: str, fs: float) -> np.ndarray:
    """Apply the configured filter to a 1-D float array."""
    sig = np.array(sig, dtype=float)
    if np.all(np.isnan(sig)):
        return sig

    ft  = filter_type.lower()
    nyq = fs / 2.0

    if ft == 'none':
        return sig

    elif ft == 'butterworth_lp':
        cutoff = min(BW_CUTOFF_LO / nyq, 0.999)
        b, a = sp_signal.butter(BW_ORDER, cutoff, btype='low')
        fn = sp_signal.filtfilt if BW_ZERO_PHASE else sp_signal.lfilter
        return fn(b, a, sig)

    elif ft == 'butterworth_hp':
        cutoff = min(BW_CUTOFF_LO / nyq, 0.999)
        b, a = sp_signal.butter(BW_ORDER, cutoff, btype='high')
        fn = sp_signal.filtfilt if BW_ZERO_PHASE else sp_signal.lfilter
        return fn(b, a, sig)

    elif ft == 'butterworth_bp':
        lo = min(BW_CUTOFF_LO / nyq, 0.499)
        hi = min(BW_CUTOFF_HI / nyq, 0.999)
        b, a = sp_signal.butter(BW_ORDER, [lo, hi], btype='band')
        fn = sp_signal.filtfilt if BW_ZERO_PHASE else sp_signal.lfilter
        return fn(b, a, sig)

    elif ft == 'moving_average':
        kernel = np.ones(MA_WINDOW) / MA_WINDOW
        return np.convolve(sig, kernel, mode='same')

    elif ft == 'savgol':
        win = SG_WINDOW if SG_WINDOW % 2 == 1 else SG_WINDOW + 1
        return sp_signal.savgol_filter(sig, win, SG_POLYORDER)

    elif ft == 'median':
        ker = MED_KERNEL if MED_KERNEL % 2 == 1 else MED_KERNEL + 1
        return sp_signal.medfilt(sig, ker)

    else:
        raise ValueError(
            f'Unknown FILTER_TYPE: "{filter_type}". '
            'Choose from: none, butterworth_lp, butterworth_hp, '
            'butterworth_bp, moving_average, savgol, median'
        )


def filter_tag() -> str:
    """Short string describing the active filter — used in output filenames."""
    ft = FILTER_TYPE.lower()
    if ft == 'none':           return 'none'
    if ft == 'butterworth_lp': return f'bw_lp_{BW_CUTOFF_LO}hz'
    if ft == 'butterworth_hp': return f'bw_hp_{BW_CUTOFF_LO}hz'
    if ft == 'butterworth_bp': return f'bw_bp_{BW_CUTOFF_LO}-{BW_CUTOFF_HI}hz'
    if ft == 'moving_average': return f'ma_{MA_WINDOW}'
    if ft == 'savgol':         return f'sg_{SG_WINDOW}_{SG_POLYORDER}'
    if ft == 'median':         return f'med_{MED_KERNEL}'
    return ft


def sensor_columns(df: pd.DataFrame, hand: str) -> list[str]:
    """Return all numeric sensor columns belonging to the given hand(s)."""
    hands = ['left', 'right'] if hand == 'both' else [hand]
    cols = []
    for h in hands:
        cols += [
            c for c in df.columns
            if c.startswith(f'{h}_') and pd.api.types.is_numeric_dtype(df[c])
        ]
    return cols


# =============================================================================
# PIPELINE
# =============================================================================

def process_file(csv_path: str, input_root: str, output_root: str, tag: str) -> dict:
    """
    Load one CSV, filter its sensor columns, write output.

    Returns a summary dict for the results log.
    """
    rel_path = os.path.relpath(csv_path, input_root)

    # ── Read ──────────────────────────────────────────────────────────────────
    try:
        df = pd.read_csv(csv_path)
    except Exception as exc:
        log.warning('  SKIP  %s  — could not read: %s', rel_path, exc)
        return {'file': rel_path, 'status': 'error', 'cols_filtered': 0,
                'rows': 0, 'note': str(exc)}

    # ── Identify columns to filter ────────────────────────────────────────────
    cols = sensor_columns(df, HAND)
    if not cols:
        log.warning('  SKIP  %s  — no matching sensor columns for hand="%s"',
                    rel_path, HAND)
        return {'file': rel_path, 'status': 'skipped', 'cols_filtered': 0,
                'rows': len(df), 'note': 'no matching columns'}

    # ── Apply filter ──────────────────────────────────────────────────────────
    df_out = df.copy()
    missing = []
    for col in cols:
        raw  = df[col].to_numpy(dtype=float)
        df_out[col] = apply_filter(raw, FILTER_TYPE, FS_HZ)

    # ── Build output path ─────────────────────────────────────────────────────
    stem, ext = os.path.splitext(os.path.basename(csv_path))
    out_name   = f'{stem}_{tag}{ext}'

    if output_root:
        rel_dir = os.path.dirname(rel_path)
        out_dir = os.path.join(output_root, rel_dir)
    else:
        out_dir = os.path.dirname(csv_path)

    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, out_name)

    df_out.to_csv(out_path, index=False)

    log.info('  OK    %s  →  %s  (%d cols, %d rows)',
             rel_path, os.path.relpath(out_path, input_root),
             len(cols), len(df))

    return {
        'file':         rel_path,
        'status':       'ok',
        'cols_filtered': len(cols),
        'rows':         len(df),
        'output':       os.path.relpath(out_path, input_root),
        'note':         '',
    }


def run():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_root = os.path.normpath(os.path.join(script_dir, INPUT_DIR))
    tag        = filter_tag()

    if not os.path.isdir(input_root):
        log.error('INPUT_DIR not found: %s', input_root)
        log.error('Update INPUT_DIR in the CONFIG block and re-run.')
        return

    # ── Discover CSVs ─────────────────────────────────────────────────────────
    csv_files = sorted(glob.glob(os.path.join(input_root, '**/*.csv'), recursive=True))
    if not csv_files:
        log.warning('No CSV files found under: %s', input_root)
        return

    log.info('=' * 60)
    log.info('Glove Filter Pipeline')
    output_root = os.path.normpath(os.path.join(script_dir, OUTPUT_DIR)) if OUTPUT_DIR else None

    log.info('  Input   : %s', input_root)
    log.info('  Output  : %s', output_root if output_root else 'alongside source files')
    log.info('  Hand    : %s', HAND)
    log.info('  Filter  : %s  (tag: %s)', FILTER_TYPE, tag)
    log.info('  Files   : %d CSV(s) found', len(csv_files))
    log.info('=' * 60)

    results = []
    for csv_path in csv_files:
        results.append(process_file(csv_path, input_root, output_root, tag))

    # ── Summary ───────────────────────────────────────────────────────────────
    ok      = sum(1 for r in results if r['status'] == 'ok')
    skipped = sum(1 for r in results if r['status'] == 'skipped')
    errors  = sum(1 for r in results if r['status'] == 'error')

    log.info('=' * 60)
    log.info('Done.  %d processed  |  %d skipped  |  %d errors', ok, skipped, errors)

    if WRITE_SUMMARY:
        summary_df   = pd.DataFrame(results)
        summary_path = os.path.join(
            output_root if output_root else script_dir,
            f'_pipeline_summary_{tag}.csv'
        )
        os.makedirs(os.path.dirname(summary_path), exist_ok=True)
        summary_df.to_csv(summary_path, index=False)
        log.info('Summary written → %s', summary_path)


if __name__ == '__main__':
    run()
