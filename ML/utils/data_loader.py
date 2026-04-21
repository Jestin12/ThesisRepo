"""
data_loader.py
==============
Shared data-loading and preprocessing utilities for the ThesisA gesture
recognition ML notebooks.

Column naming convention (from glove_visualiser):
    {hand}_{segment}_{loc}_{channel}
    e.g.  left_thumb_mid_ax   |  right_index_prox_pitch  |  left_thumb_flex_mcp

Sensor locations per finger:
    * mid   — distal phalanx IMU   (ax, ay, az, pitch, roll, yaw)
    * prox  — proximal phalanx IMU (ax, ay, az, pitch, roll, yaw)

Additional IMUs:
    * palm  — back of palm         (ax, ay, az, pitch, roll, yaw)
    * wrist — forearm near wrist   (ax, ay, az, pitch, roll, yaw)

Flex sensors (one pair per finger, no loc suffix):
    * flex_mcp  — MCP joint
    * flex_pip  — PIP joint

NOTE: TwoHand_L_Flat_R_Flat samples include quaternion columns (qw, qx, qy, qz)
      which should be EXCLUDED (pitch/roll/yaw are derived from them and can be
      used instead, but are also skipped by default per project guidance).
"""

import os
import glob
import re
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import signal as sp_signal
from scipy.interpolate import interp1d
from sklearn.preprocessing import MinMaxScaler, StandardScaler

warnings.filterwarnings("ignore")

# ---------------------------------------------------------------------------
# Known sensor configuration
# ---------------------------------------------------------------------------
FINGERS = ["thumb", "index", "middle", "ring", "pinky"]
LOCS    = ["mid", "prox"]          # per-finger IMU locations
HANDS   = ["left", "right"]

IMU_CHANNELS   = ["ax", "ay", "az", "pitch", "roll", "yaw"]
FLEX_CHANNELS  = ["flex_mcp", "flex_pip"]
PALM_CHANNELS  = ["palm_ax", "palm_ay", "palm_az",
                  "palm_pitch", "palm_roll", "palm_yaw"]
WRIST_CHANNELS = ["wrist_ax", "wrist_ay", "wrist_az",
                  "wrist_pitch", "wrist_roll", "wrist_yaw"]

# Columns to ALWAYS exclude (quaternion raw values)
EXCLUDE_PATTERNS = [r".*_qw$", r".*_qx$", r".*_qy$", r".*_qz$"]


# ---------------------------------------------------------------------------
# Column discovery helpers
# ---------------------------------------------------------------------------

def build_column_groups(hands=None, fingers=None, locs=None,
                        include_flex=True, include_palm=True,
                        include_wrist=True, include_imu=True):
    """
    Build the canonical list of feature column name groups.

    Returns
    -------
    dict  {group_name: [col_name, ...]}
    """
    hands   = hands   or HANDS
    fingers = fingers or FINGERS
    locs    = locs    or LOCS

    groups = {}
    for hand in hands:
        for finger in fingers:
            for loc in locs:
                key = f"{hand}_{finger}_{loc}"
                if include_imu:
                    groups[f"{key}_imu"] = [f"{key}_{ch}" for ch in IMU_CHANNELS]
            if include_flex:
                groups[f"{hand}_{finger}_flex"] = [
                    f"{hand}_{finger}_{ch}" for ch in FLEX_CHANNELS
                ]
        if include_palm:
            groups[f"{hand}_palm"] = [f"{hand}_{ch}" for ch in PALM_CHANNELS]
        if include_wrist:
            groups[f"{hand}_wrist"] = [f"{hand}_{ch}" for ch in WRIST_CHANNELS]

    return groups


def filter_existing_columns(df, wanted_cols):
    """Return only those columns from wanted_cols that exist in df."""
    return [c for c in wanted_cols if c in df.columns]


def exclude_quaternion_cols(columns):
    """Strip any raw quaternion columns from a list."""
    patterns = [re.compile(p) for p in EXCLUDE_PATTERNS]
    return [c for c in columns if not any(p.match(c) for p in patterns)]


# ---------------------------------------------------------------------------
# Dataset discovery
# ---------------------------------------------------------------------------

def discover_dataset(data_root):
    """
    Walk data_root and return a DataFrame with columns:
        label, filepath, filename

    The label is inferred from the immediate parent folder name.
    The TwoHand_L_Flat_R_Flat folder is included but the label is corrected
    to 'TwoHand_L_Flat_R_Flat' (files inside have 'Fist' in their name — this
    is a known data collection error noted in the project documentation).
    """
    rows = []
    data_root = Path(data_root)
    for csv_path in sorted(data_root.rglob("*.csv")):
        label = csv_path.parent.name
        rows.append({"label": label,
                     "filepath": str(csv_path),
                     "filename": csv_path.name})
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

def load_csv(filepath, feature_cols=None, exclude_quat=True):
    """
    Load one gesture CSV.

    Parameters
    ----------
    filepath     : path to CSV
    feature_cols : list of column names to keep (None = all numeric)
    exclude_quat : drop raw quaternion columns

    Returns
    -------
    df  : pandas DataFrame of numeric sensor data
    """
    df = pd.read_csv(filepath)
    # Drop timestamp/metadata columns
    numeric_cols = df.select_dtypes(include=[np.number]).columns.tolist()
    time_like    = [c for c in numeric_cols
                    if "time" in c.lower() or "timestamp" in c.lower()]
    numeric_cols = [c for c in numeric_cols if c not in time_like]

    if exclude_quat:
        numeric_cols = exclude_quaternion_cols(numeric_cols)

    if feature_cols is not None:
        wanted = [c for c in feature_cols if c in numeric_cols]
        if not wanted:
            raise ValueError(
                f"None of the requested feature columns exist in {filepath}.\n"
                f"Available: {numeric_cols[:20]}"
            )
        numeric_cols = wanted

    return df[numeric_cols].copy()


# ---------------------------------------------------------------------------
# Preprocessing
# ---------------------------------------------------------------------------

def apply_filter(signal_arr, filter_type, fs):
    """
    Apply a temporal filter to a 1-D NumPy array.

    filter_type options
    -------------------
    'none'           – pass through
    'butterworth_lp' – Butterworth low-pass  (default cutoff 6 Hz, order 4)
    'butterworth_hp' – Butterworth high-pass (default cutoff 0.5 Hz, order 4)
    'butterworth_bp' – Butterworth band-pass (default 0.5–6 Hz, order 4)
    'moving_average' – causal moving average (window 5)
    'savgol'         – Savitzky-Golay (window 11, poly 3)
    'median'         – Median filter (kernel 5)

    All parameters can be tuned by modifying the constants below.
    """
    BW_ORDER    = 4
    BW_LO_HZ   = 6.0
    BW_HI_HZ   = 0.5
    BW_BP_LO   = 0.5
    BW_BP_HI   = 6.0
    MA_WIN      = 5
    SG_WIN      = 11
    SG_POLY     = 3
    MED_KERN    = 5

    sig = np.asarray(signal_arr, dtype=float)
    nyq = fs / 2.0
    ft  = filter_type.lower()

    if ft == "none":
        return sig

    if ft == "butterworth_lp":
        b, a = sp_signal.butter(BW_ORDER, min(BW_LO_HZ / nyq, 0.999), btype="low")
        return sp_signal.filtfilt(b, a, sig)

    if ft == "butterworth_hp":
        b, a = sp_signal.butter(BW_ORDER, min(BW_HI_HZ / nyq, 0.999), btype="high")
        return sp_signal.filtfilt(b, a, sig)

    if ft == "butterworth_bp":
        lo = min(BW_BP_LO / nyq, 0.499)
        hi = min(BW_BP_HI / nyq, 0.999)
        b, a = sp_signal.butter(BW_ORDER, [lo, hi], btype="band")
        return sp_signal.filtfilt(b, a, sig)

    if ft == "moving_average":
        k = np.ones(MA_WIN) / MA_WIN
        return np.convolve(sig, k, mode="same")

    if ft == "savgol":
        win = SG_WIN if SG_WIN % 2 == 1 else SG_WIN + 1
        return sp_signal.savgol_filter(sig, win, SG_POLY)

    if ft == "median":
        kern = MED_KERN if MED_KERN % 2 == 1 else MED_KERN + 1
        return sp_signal.medfilt(sig, kern)

    raise ValueError(f"Unknown filter_type: '{filter_type}'")


def preprocess_dataframe(df, filter_type="none", fs=22.0):
    """Apply filter to every column of df. Returns a new DataFrame."""
    out = df.copy()
    for col in out.columns:
        out[col] = apply_filter(out[col].values, filter_type, fs)
    return out


def resample_to_length(arr, target_len):
    """
    Resample a 2-D array (T × C) to target_len time steps using linear
    interpolation.  Returns shape (target_len, C).
    """
    T, C = arr.shape
    if T == target_len:
        return arr
    x_old = np.linspace(0, 1, T)
    x_new = np.linspace(0, 1, target_len)
    out   = np.zeros((target_len, C), dtype=arr.dtype)
    for c in range(C):
        f = interp1d(x_old, arr[:, c], kind="linear", fill_value="extrapolate")
        out[:, c] = f(x_new)
    return out


def normalize(X, method="minmax", per_sample=True):
    """
    Normalize a 3-D array of shape (N, T, C).

    method     : 'minmax'  → [0, 1]  |  'zscore' → zero-mean unit-variance
    per_sample : if True, fit scaler per sample; if False, fit globally
    """
    N, T, C = X.shape
    out = np.zeros_like(X, dtype=np.float32)

    if per_sample:
        for i in range(N):
            sample = X[i]  # (T, C)
            if method == "minmax":
                sc = MinMaxScaler()
            else:
                sc = StandardScaler()
            out[i] = sc.fit_transform(sample)
    else:
        flat = X.reshape(-1, C)
        if method == "minmax":
            sc = MinMaxScaler()
        else:
            sc = StandardScaler()
        flat_norm = sc.fit_transform(flat)
        out = flat_norm.reshape(N, T, C).astype(np.float32)

    return out


# ---------------------------------------------------------------------------
# Full dataset builder
# ---------------------------------------------------------------------------

def build_dataset(data_root, feature_cols=None, filter_type="none",
                  fs=22.0, target_len=110, normalization="minmax",
                  per_sample_norm=True, exclude_quat=True):
    """
    Load every CSV under data_root, apply preprocessing, and return X, y arrays.

    Parameters
    ----------
    data_root       : path to the root folder containing gesture subdirectories
    feature_cols    : list of column names to include (None = all numeric)
    filter_type     : temporal filter to apply (see apply_filter)
    fs              : sampling rate in Hz (default 22 Hz per glove_visualiser)
    target_len      : resample all trials to this many time steps
    normalization   : 'minmax' | 'zscore' | 'none'
    per_sample_norm : normalize each sample independently
    exclude_quat    : drop raw quaternion columns

    Returns
    -------
    X         : np.float32 array  (N, target_len, C)
    y         : np.int64   array  (N,)
    labels    : list[str]  — gesture name for each integer label
    feature_cols_used : list[str]
    """
    manifest  = discover_dataset(data_root)
    label_map = get_label_map(data_root)
    labels    = [k for k, v in sorted(label_map.items(), key=lambda x: x[1])]

    X_list, y_list = [], []
    feature_cols_used = None

    for _, row in manifest.iterrows():
        try:
            df = load_csv(row["filepath"], feature_cols=feature_cols,
                          exclude_quat=exclude_quat)
        except Exception as e:
            warnings.warn(f"Skipping {row['filepath']}: {e}")
            continue

        if feature_cols_used is None:
            feature_cols_used = df.columns.tolist()

        # Apply temporal filter
        if filter_type != "none":
            df = preprocess_dataframe(df, filter_type=filter_type, fs=fs)

        arr = df.values.astype(np.float32)  # (T, C)

        # Drop rows with NaN
        arr = arr[~np.isnan(arr).any(axis=1)]
        if len(arr) < 4:
            warnings.warn(f"Too few valid rows in {row['filepath']}, skipping.")
            continue

        arr = resample_to_length(arr, target_len)
        X_list.append(arr)
        y_list.append(label_map[row["label"]])

    if not X_list:
        raise RuntimeError("No samples were loaded. Check data_root and feature_cols.")

    X = np.stack(X_list, axis=0).astype(np.float32)   # (N, T, C)
    y = np.array(y_list, dtype=np.int64)

    if normalization != "none":
        X = normalize(X, method=normalization, per_sample=per_sample_norm)

    print(f"Dataset built: {X.shape[0]} samples, "
          f"{X.shape[1]} timesteps, {X.shape[2]} features, "
          f"{len(labels)} classes")
    print(f"Classes: {labels}")
    return X, y, labels, feature_cols_used


# ---------------------------------------------------------------------------
# Train / val / test split
# ---------------------------------------------------------------------------

def split_dataset(X, y, train_ratio=0.7, val_ratio=0.1, seed=42):
    """
    Stratified split into train / val / test.

    Returns (X_train, y_train), (X_val, y_val), (X_test, y_test)
    """
    from sklearn.model_selection import train_test_split

    test_ratio = 1.0 - train_ratio - val_ratio
    assert test_ratio > 0, "train_ratio + val_ratio must be < 1.0"

    X_tv, X_test, y_tv, y_test = train_test_split(
        X, y, test_size=test_ratio, stratify=y, random_state=seed
    )
    val_frac_of_tv = val_ratio / (train_ratio + val_ratio)
    X_train, X_val, y_train, y_val = train_test_split(
        X_tv, y_tv, test_size=val_frac_of_tv, stratify=y_tv, random_state=seed
    )

    print(f"Split → train: {len(X_train)}, val: {len(X_val)}, test: {len(X_test)}")
    return (X_train, y_train), (X_val, y_val), (X_test, y_test)
