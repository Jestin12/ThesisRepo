"""
data_loader.py
==============
Shared data-loading and preprocessing utilities for the ThesisA gesture
recognition ML notebooks.

Confirmed column schema (from glove_data_L_Rock_R_Rock_5s_1_... .csv):
=======================================================================
191 total columns:
  Meta / timing  (11 cols, EXCLUDED from features):
      run_index, request_id, request_ts,
      right_recv_time_ms, right_glove_time_ms, right_time,
      left_recv_time_ms,  left_glove_time_ms,  left_time,
      right_hand, left_hand

  Per hand (right_ / left_) — 90 feature columns each = 180 total:
  ----------------------------------------------------------------
  Palm IMUs:
      {hand}_palm_mid_{yaw/pitch/roll/ax/ay/az}      ← ALL ZERO (no sensor) — excluded
      {hand}_palm_prox_{yaw/pitch/roll/ax/ay/az}     ← real back-of-hand IMU

  Palm flex (always -1, no sensor fitted):
      {hand}_palm_mcp_flex, {hand}_palm_pip_flex     ← excluded

  Per finger (thumb / index / middle / ring / pinky):
      {hand}_{finger}_mcp_flex                       ← MCP joint flex
      {hand}_{finger}_pip_flex                       ← PIP joint flex
      {hand}_{finger}_mid_{yaw/pitch/roll/ax/ay/az}  ← distal phalanx IMU
      {hand}_{finger}_prox_{yaw/pitch/roll/ax/ay/az} ← proximal phalanx IMU

  Wrist IMU (note: 'heading' not 'yaw'):
      {hand}_wrist_{ax/ay/az/heading/pitch/roll}

Derived Euler angles (yaw/pitch/roll) are already computed — NO raw quaternion
columns exist in this dataset.

Dataset structure:
    data_root/
        TwoHand_L_Fist_R_Fist_filtered_butterworth_lp/    ← label = folder name
            glove_data_L_Fist_R_Fist_5s_1_...csv
            ...
        TwoHand_L_Flat_R_Flat_filtered_butterworth_lp/
        TwoHand_L_Okay_R_Okay_filtered_butterworth_lp/
        TwoHand_L_Rock_R_Rock_filtered_butterworth_lp/

    4 classes × 100 samples = 400 trials
    ~156 timesteps per trial at ~31.2 Hz (5-second recordings)
"""

import os
import glob
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import signal as sp_signal
from scipy.interpolate import interp1d
from sklearn.preprocessing import MinMaxScaler, StandardScaler

warnings.filterwarnings("ignore")

# ---------------------------------------------------------------------------
# Ground-truth sensor layout (confirmed from real CSV)
# ---------------------------------------------------------------------------
HANDS   = ["right", "left"]
FINGERS = ["thumb", "index", "middle", "ring", "pinky"]
LOCS    = ["mid", "prox"]          # distal / proximal phalanx IMU

IMU_ORIENT_CHANNELS = ["yaw", "pitch", "roll"]
IMU_ACCEL_CHANNELS  = ["ax", "ay", "az"]
IMU_ALL_CHANNELS    = IMU_ORIENT_CHANNELS + IMU_ACCEL_CHANNELS
FLEX_CHANNELS       = ["mcp_flex", "pip_flex"]

# Wrist uses 'heading' instead of 'yaw'
WRIST_ORIENT_CHANNELS = ["heading", "pitch", "roll"]
WRIST_ACCEL_CHANNELS  = ["ax", "ay", "az"]
WRIST_ALL_CHANNELS    = WRIST_ACCEL_CHANNELS + WRIST_ORIENT_CHANNELS

# Columns that are always constant/invalid — always excluded
_ALWAYS_EXCLUDE = []
# palm_mid is always zero (no sensor fitted)
for _h in HANDS:
    for _ch in IMU_ALL_CHANNELS:
        _ALWAYS_EXCLUDE.append(f"{_h}_palm_mid_{_ch}")
# palm flex is always -1 (no sensor fitted)
for _h in HANDS:
    for _flex in FLEX_CHANNELS:
        _ALWAYS_EXCLUDE.append(f"{_h}_palm_{_flex}")

# Meta / timing columns to strip before feature extraction
_META_COLS = {
    "run_index", "request_id", "request_ts",
    "right_recv_time_ms", "right_glove_time_ms", "right_time",
    "left_recv_time_ms",  "left_glove_time_ms",  "left_time",
    "right_hand", "left_hand",
}

# Default sampling rate (confirmed ~31.2 Hz from 156 rows over 5 s)
DEFAULT_FS_HZ = 31.0

# Default target length: keep native ~156 samples or set explicitly
DEFAULT_TARGET_LEN = 156


# ---------------------------------------------------------------------------
# Column-group builder
# ---------------------------------------------------------------------------

def build_column_groups(
    hands=None,
    fingers=None,
    locs=None,
    include_flex=True,
    include_palm_prox=True,
    include_wrist=True,
    include_accel=True,
    include_orient=True,
):
    """
    Build a flat list of feature column names based on selection flags.

    Parameters
    ----------
    hands           : list of hands to include, e.g. ['right'] or ['right','left']
    fingers         : list of finger names (default: all 5)
    locs            : list of IMU locations per finger: ['mid'], ['prox'], or both
    include_flex    : include MCP/PIP flex sensor columns
    include_palm_prox : include the back-of-hand IMU (palm_prox)
    include_wrist   : include the wrist IMU
    include_accel   : include ax/ay/az channels
    include_orient  : include yaw/pitch/roll (and heading for wrist)

    Returns
    -------
    list[str]  — ordered list of column names
    """
    hands   = hands   or HANDS
    fingers = fingers or FINGERS
    locs    = locs    or LOCS

    # Build per-IMU channel set
    imu_channels = []
    if include_orient:
        imu_channels += IMU_ORIENT_CHANNELS
    if include_accel:
        imu_channels += IMU_ACCEL_CHANNELS

    wrist_channels = []
    if include_accel:
        wrist_channels += WRIST_ACCEL_CHANNELS
    if include_orient:
        wrist_channels += WRIST_ORIENT_CHANNELS   # includes 'heading'

    cols = []
    for hand in hands:
        # Palm back-of-hand (palm_prox only — palm_mid is always zero)
        if include_palm_prox:
            for ch in imu_channels:
                cols.append(f"{hand}_palm_prox_{ch}")

        # Per-finger sensors
        for finger in fingers:
            if include_flex:
                for flex in FLEX_CHANNELS:
                    cols.append(f"{hand}_{finger}_{flex}")
            for loc in locs:
                for ch in imu_channels:
                    cols.append(f"{hand}_{finger}_{loc}_{ch}")

        # Wrist IMU
        if include_wrist:
            for ch in wrist_channels:
                cols.append(f"{hand}_wrist_{ch}")

    return cols


# ---------------------------------------------------------------------------
# Dataset discovery
# ---------------------------------------------------------------------------

def discover_dataset(data_root):
    """
    Walk data_root and return a DataFrame: label | filepath | filename.
    Label = immediate parent folder name.
    """
    rows = []
    for csv_path in sorted(Path(data_root).rglob("*.csv")):
        rows.append({
            "label":    csv_path.parent.name,
            "filepath": str(csv_path),
            "filename": csv_path.name,
        })
    df = pd.DataFrame(rows)
    if df.empty:
        raise FileNotFoundError(f"No CSV files found under: {data_root}")
    return df


def get_label_map(data_root):
    """Return {label_str: int_index} sorted alphabetically."""
    manifest = discover_dataset(data_root)
    labels   = sorted(manifest["label"].unique())
    return {lbl: idx for idx, lbl in enumerate(labels)}


# ---------------------------------------------------------------------------
# Single-file loading
# ---------------------------------------------------------------------------

def load_csv(filepath, feature_cols=None):
    """
    Load one gesture CSV and return only feature columns.

    Parameters
    ----------
    filepath     : path to CSV
    feature_cols : explicit list of columns to keep (None = all valid features)

    Returns
    -------
    df : DataFrame  — numeric sensor data, meta/timing columns removed
    """
    df = pd.read_csv(filepath)

    # Drop meta / timing / string identifier columns
    drop = [c for c in df.columns if c in _META_COLS or c in _ALWAYS_EXCLUDE]
    df   = df.drop(columns=drop, errors="ignore")

    # Keep only numeric
    df = df.select_dtypes(include=[np.number])

    if feature_cols is not None:
        missing  = [c for c in feature_cols if c not in df.columns]
        present  = [c for c in feature_cols if c in df.columns]
        if missing:
            warnings.warn(
                f"{os.path.basename(filepath)}: {len(missing)} requested columns "
                f"not found and will be skipped: {missing[:5]}{'...' if len(missing)>5 else ''}"
            )
        if not present:
            raise ValueError(
                f"None of the requested feature columns exist in {filepath}."
            )
        df = df[present]

    return df.reset_index(drop=True)


# ---------------------------------------------------------------------------
# Preprocessing helpers
# ---------------------------------------------------------------------------

def apply_filter(signal_arr, filter_type, fs,
                 bw_order=4, bw_lo=6.0, bw_hi=0.5,
                 bw_bp_lo=0.5, bw_bp_hi=10.0,
                 ma_win=5, sg_win=11, sg_poly=3, med_kern=5):
    """
    Apply a temporal filter to a 1-D NumPy array.

    filter_type options
    -------------------
    'none'           – pass through
    'butterworth_lp' – low-pass  (cutoff bw_lo Hz)
    'butterworth_hp' – high-pass (cutoff bw_hi Hz)
    'butterworth_bp' – band-pass (bw_bp_lo – bw_bp_hi Hz)
    'moving_average' – causal moving average
    'savgol'         – Savitzky-Golay
    'median'         – Median filter
    """
    sig = np.asarray(signal_arr, dtype=float)
    nyq = fs / 2.0
    ft  = filter_type.lower()

    if ft == "none":
        return sig
    if ft == "butterworth_lp":
        b, a = sp_signal.butter(bw_order, min(bw_lo / nyq, 0.999), btype="low")
        return sp_signal.filtfilt(b, a, sig)
    if ft == "butterworth_hp":
        b, a = sp_signal.butter(bw_order, min(bw_hi / nyq, 0.999), btype="high")
        return sp_signal.filtfilt(b, a, sig)
    if ft == "butterworth_bp":
        lo = min(bw_bp_lo / nyq, 0.499)
        hi = min(bw_bp_hi / nyq, 0.999)
        b, a = sp_signal.butter(bw_order, [lo, hi], btype="band")
        return sp_signal.filtfilt(b, a, sig)
    if ft == "moving_average":
        k = np.ones(ma_win) / ma_win
        return np.convolve(sig, k, mode="same")
    if ft == "savgol":
        win = sg_win if sg_win % 2 == 1 else sg_win + 1
        return sp_signal.savgol_filter(sig, win, sg_poly)
    if ft == "median":
        kern = med_kern if med_kern % 2 == 1 else med_kern + 1
        return sp_signal.medfilt(sig, kern)

    raise ValueError(
        f"Unknown filter_type: '{filter_type}'. "
        "Choose: none | butterworth_lp | butterworth_hp | butterworth_bp | "
        "moving_average | savgol | median"
    )


def preprocess_dataframe(df, filter_type="none", fs=DEFAULT_FS_HZ):
    """Apply filter to every column of df. Returns a new DataFrame."""
    out = df.copy()
    for col in out.columns:
        out[col] = apply_filter(out[col].values, filter_type, fs)
    return out


def resample_to_length(arr, target_len):
    """
    Resample a 2-D array (T × C) to target_len timesteps using linear
    interpolation along the time axis. Returns shape (target_len, C).
    """
    T, C = arr.shape
    if T == target_len:
        return arr
    x_old = np.linspace(0, 1, T)
    x_new = np.linspace(0, 1, target_len)
    out   = np.zeros((target_len, C), dtype=arr.dtype)
    for c in range(C):
        f      = interp1d(x_old, arr[:, c], kind="linear",
                          fill_value="extrapolate")
        out[:, c] = f(x_new)
    return out


def normalize(X, method="minmax", per_sample=True):
    """
    Normalize a 3-D array (N, T, C).

    method     : 'minmax' → [0,1]  |  'zscore' → zero-mean unit-variance
    per_sample : if True, fit scaler independently per trial (recommended)
    """
    N, T, C = X.shape
    out = np.zeros_like(X, dtype=np.float32)

    if per_sample:
        for i in range(N):
            sc = MinMaxScaler() if method == "minmax" else StandardScaler()
            out[i] = sc.fit_transform(X[i])          # X[i] is (T, C)
    else:
        flat      = X.reshape(-1, C)
        sc        = MinMaxScaler() if method == "minmax" else StandardScaler()
        flat_norm = sc.fit_transform(flat)
        out       = flat_norm.reshape(N, T, C).astype(np.float32)

    return out


# ---------------------------------------------------------------------------
# Full dataset builder
# ---------------------------------------------------------------------------

def build_dataset(
    data_root,
    feature_cols=None,
    filter_type="none",
    fs=DEFAULT_FS_HZ,
    target_len=DEFAULT_TARGET_LEN,
    normalization="minmax",
    per_sample_norm=True,
):
    """
    Load every CSV under data_root, apply preprocessing, and return arrays.

    Parameters
    ----------
    data_root       : path to root folder containing gesture subdirectories
    feature_cols    : explicit list of column names (None = all valid features)
    filter_type     : 'none' | 'butterworth_lp' | 'butterworth_hp' |
                      'butterworth_bp' | 'moving_average' | 'savgol' | 'median'
    fs              : sampling rate in Hz (default ~31 Hz)
    target_len      : resample all trials to this many timesteps (default 156)
    normalization   : 'minmax' | 'zscore' | 'none'
    per_sample_norm : if True, normalize each trial independently

    Returns
    -------
    X                : np.float32  (N, target_len, C)
    y                : np.int64    (N,)
    labels           : list[str]   — gesture class name for each integer label
    feature_cols_used: list[str]   — columns actually loaded
    """
    manifest         = discover_dataset(data_root)
    label_map        = get_label_map(data_root)
    labels           = [k for k, v in sorted(label_map.items(), key=lambda x: x[1])]
    X_list, y_list   = [], []
    feature_cols_used = None

    for _, row in manifest.iterrows():
        try:
            df = load_csv(row["filepath"], feature_cols=feature_cols)
        except Exception as e:
            warnings.warn(f"Skipping {row['filepath']}: {e}")
            continue

        if feature_cols_used is None:
            feature_cols_used = df.columns.tolist()

        if filter_type != "none":
            df = preprocess_dataframe(df, filter_type=filter_type, fs=fs)

        arr = df.values.astype(np.float32)          # (T, C)
        arr = arr[~np.isnan(arr).any(axis=1)]       # drop NaN rows

        if len(arr) < 4:
            warnings.warn(f"Too few valid rows in {row['filepath']}, skipping.")
            continue

        if arr.shape[0] != target_len:
            arr = resample_to_length(arr, target_len)

        X_list.append(arr)
        y_list.append(label_map[row["label"]])

    if not X_list:
        raise RuntimeError(
            "No samples were loaded. Check DATA_ROOT points to the folder "
            "containing the four gesture subfolders."
        )

    X = np.stack(X_list, axis=0).astype(np.float32)   # (N, T, C)
    y = np.array(y_list, dtype=np.int64)

    if normalization != "none":
        X = normalize(X, method=normalization, per_sample=per_sample_norm)

    print(f"Dataset loaded: {X.shape[0]} samples  |  "
          f"{X.shape[1]} timesteps  |  {X.shape[2]} features  |  "
          f"{len(labels)} classes")
    print(f"  Classes ({len(labels)}): {labels}")
    print(f"  Class distribution: "
          + "  ".join(f"{lbl}={int((y==i).sum())}"
                      for i, lbl in enumerate(labels)))
    return X, y, labels, feature_cols_used


# ---------------------------------------------------------------------------
# Train / val / test split
# ---------------------------------------------------------------------------

def split_dataset(X, y, train_ratio=0.70, val_ratio=0.10, seed=42):
    """
    Stratified split → (X_train, y_train), (X_val, y_val), (X_test, y_test).
    Default: 70 / 10 / 20.
    """
    from sklearn.model_selection import train_test_split

    test_ratio = 1.0 - train_ratio - val_ratio
    assert test_ratio > 0.0, "train_ratio + val_ratio must be < 1.0"

    X_tv, X_test, y_tv, y_test = train_test_split(
        X, y, test_size=test_ratio, stratify=y, random_state=seed
    )
    val_frac = val_ratio / (train_ratio + val_ratio)
    X_train, X_val, y_train, y_val = train_test_split(
        X_tv, y_tv, test_size=val_frac, stratify=y_tv, random_state=seed
    )
    print(f"Split: train={len(X_train)}  val={len(X_val)}  test={len(X_test)}")
    return (X_train, y_train), (X_val, y_val), (X_test, y_test)
