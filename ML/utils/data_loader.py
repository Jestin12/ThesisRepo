"""
data_loader.py
==============
Shared data-loading and preprocessing utilities used by every ML notebook
in this project.

HOW THIS FILE FITS INTO THE PIPELINE
--------------------------------------
Your COMP3308 lectures describe supervised learning as:
    "Given a set of pre-labelled examples {x, y}, learn a function
     (classifier) that can predict y for new, unseen x."

Here:
  x  = one 5-second glove trial → a 2D array of shape (timesteps, features)
  y  = the gesture label (Fist / Flat / Okay / Rock) stored as an integer

This file handles everything BEFORE the model sees the data:
    1. Discovering which CSV files exist and what label each belongs to
    2. Loading a single CSV and selecting the requested sensor columns
    3. Applying an optional temporal filter (smoothing)
    4. Resampling all trials to the same number of timesteps
    5. Normalising the values into a consistent numeric range
    6. Stacking all trials into a 3D numpy array ready for Keras
    7. Splitting into train / val / test subsets

CONFIRMED COLUMN SCHEMA (from real CSV inspection)
---------------------------------------------------
191 total columns per trial. After removing metadata, 180 are features (90/hand).

  Metadata columns (EXCLUDED — not sensor readings):
      run_index, request_id, request_ts,
      right_recv_time_ms, right_glove_time_ms, right_time,
      left_recv_time_ms,  left_glove_time_ms,  left_time,
      right_hand, left_hand

  Per hand (right_ / left_) — 90 feature columns each:
  ─────────────────────────────────────────────────────
  Palm IMUs:
      {hand}_palm_mid_{yaw/pitch/roll/ax/ay/az}   ← ALWAYS ZERO, auto-excluded
      {hand}_palm_prox_{yaw/pitch/roll/ax/ay/az}  ← real back-of-hand IMU

  Palm flex:
      {hand}_palm_mcp_flex, {hand}_palm_pip_flex  ← ALWAYS -1, auto-excluded

  Per finger (thumb / index / middle / ring / pinky):
      {hand}_{finger}_mcp_flex                    ← MCP joint flex sensor
      {hand}_{finger}_pip_flex                    ← PIP joint flex sensor
      {hand}_{finger}_mid_{yaw/pitch/roll/ax/ay/az}   ← distal phalanx IMU
      {hand}_{finger}_prox_{yaw/pitch/roll/ax/ay/az}  ← proximal phalanx IMU

  Wrist IMU (note: 'heading' instead of 'yaw' here):
      {hand}_wrist_{ax/ay/az/heading/pitch/roll}

NOTE: No raw quaternion columns exist — yaw/pitch/roll are already Euler angles.

DATASET LAYOUT
--------------
    data_root/
        TwoHand_L_Fist_R_Fist_filtered_butterworth_lp/   ← label = folder name
            glove_data_L_Fist_R_Fist_5s_1_...csv
            ...  (100 files)
        TwoHand_L_Flat_R_Flat_filtered_butterworth_lp/
        TwoHand_L_Okay_R_Okay_filtered_butterworth_lp/
        TwoHand_L_Rock_R_Rock_filtered_butterworth_lp/

    4 classes × 100 samples = 400 trials total
    ~156 timesteps per trial at ~31.2 Hz  (5-second recordings)
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

# =============================================================================
# SENSOR LAYOUT CONSTANTS
# =============================================================================
# These define which sensors exist on the glove and how they are named in the CSV.
# Think of these as the "vocabulary" used when building the feature column list.

HANDS   = ["right", "left"]
FINGERS = ["thumb", "index", "middle", "ring", "pinky"]

# Each finger has TWO IMU positions:
#   "mid"  = distal phalanx  (the fingertip segment)
#   "prox" = proximal phalanx (the base segment, closest to the palm)
LOCS    = ["mid", "prox"]

# IMU channels — what each IMU reports:
#   Orientation: yaw (rotation around vertical axis), pitch (forward/back tilt),
#                roll (side-to-side tilt)
#   Accelerometer: ax, ay, az (linear acceleration in 3D space, in g)
IMU_ORIENT_CHANNELS = ["yaw", "pitch", "roll"]
IMU_ACCEL_CHANNELS  = ["ax", "ay", "az"]
IMU_ALL_CHANNELS    = IMU_ORIENT_CHANNELS + IMU_ACCEL_CHANNELS

# Flex sensors measure how bent each finger joint is.
# Each finger has MCP (knuckle) and PIP (middle joint) sensors.
FLEX_CHANNELS       = ["mcp_flex", "pip_flex"]

# Wrist uses "heading" rather than "yaw" for its yaw-equivalent channel.
WRIST_ORIENT_CHANNELS = ["heading", "pitch", "roll"]
WRIST_ACCEL_CHANNELS  = ["ax", "ay", "az"]
WRIST_ALL_CHANNELS    = WRIST_ACCEL_CHANNELS + WRIST_ORIENT_CHANNELS

# =============================================================================
# COLUMNS THAT ARE ALWAYS EXCLUDED
# =============================================================================
# These were confirmed to be constant/invalid by inspecting a real CSV:
#   palm_mid_*  is always 0.0  (no sensor was physically fitted there)
#   palm_flex   is always -1   (no sensor was fitted at the palm joints)
# Including constant columns in your feature set would waste capacity and
# potentially confuse the model — a channel that never changes tells the
# model nothing about which gesture is being performed.

_ALWAYS_EXCLUDE = []
for _h in HANDS:
    for _ch in IMU_ALL_CHANNELS:
        _ALWAYS_EXCLUDE.append(f"{_h}_palm_mid_{_ch}")   # always zero
    for _flex in FLEX_CHANNELS:
        _ALWAYS_EXCLUDE.append(f"{_h}_palm_{_flex}")     # always -1

# Metadata / timing columns that are not sensor readings
_META_COLS = {
    "run_index", "request_id", "request_ts",
    "right_recv_time_ms", "right_glove_time_ms", "right_time",
    "left_recv_time_ms",  "left_glove_time_ms",  "left_time",
    "right_hand", "left_hand",
}

# Default values confirmed from real data
DEFAULT_FS_HZ     = 31.0   # sampling rate: ~156 rows / 5 s ≈ 31.2 Hz
DEFAULT_TARGET_LEN = 156   # native trial length; no resampling needed at this value


# =============================================================================
# COLUMN-GROUP BUILDER
# =============================================================================

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
    Build a flat, ordered list of feature column names based on which
    sensors you want to include.

    This is the bridge between the True/False CONFIG flags in the notebooks
    and the actual column names that appear in the CSV files.

    For example, with include_flex=True and hands=['right'], this adds:
        right_thumb_mcp_flex, right_thumb_pip_flex,
        right_index_mcp_flex, right_index_pip_flex, ...

    Parameters
    ----------
    hands             : list of hands, e.g. ['right'] or ['right','left']
    fingers           : list of finger names (default: all 5)
    locs              : IMU positions per finger: ['mid'], ['prox'], or both
    include_flex      : include MCP/PIP flex sensor columns
    include_palm_prox : include the back-of-hand IMU (palm_prox only;
                        palm_mid is excluded automatically — it is always zero)
    include_wrist     : include the wrist IMU
    include_accel     : include ax/ay/az channels from each IMU
    include_orient    : include yaw/pitch/roll (and 'heading' for the wrist)

    Returns
    -------
    list[str]  — ordered list of column names matching the CSV headers
    """
    hands   = hands   or HANDS
    fingers = fingers or FINGERS
    locs    = locs    or LOCS

    # Decide which IMU channel types to include based on the config flags
    imu_channels = []
    if include_orient:
        imu_channels += IMU_ORIENT_CHANNELS   # yaw, pitch, roll
    if include_accel:
        imu_channels += IMU_ACCEL_CHANNELS    # ax, ay, az

    # Wrist orientation uses 'heading' instead of 'yaw'
    wrist_channels = []
    if include_accel:
        wrist_channels += WRIST_ACCEL_CHANNELS    # ax, ay, az
    if include_orient:
        wrist_channels += WRIST_ORIENT_CHANNELS   # heading, pitch, roll

    cols = []
    for hand in hands:

        # ── Palm back-of-hand IMU ─────────────────────────────────────────────
        # Only "palm_prox" is included; "palm_mid" is always zero and is
        # handled in _ALWAYS_EXCLUDE automatically.
        if include_palm_prox:
            for ch in imu_channels:
                cols.append(f"{hand}_palm_prox_{ch}")

        # ── Per-finger sensors ────────────────────────────────────────────────
        for finger in fingers:

            # Flex sensors (one pair per finger, no location suffix)
            if include_flex:
                for flex in FLEX_CHANNELS:     # mcp_flex, pip_flex
                    cols.append(f"{hand}_{finger}_{flex}")

            # IMU at each finger location (mid = distal tip, prox = base)
            for loc in locs:
                for ch in imu_channels:
                    cols.append(f"{hand}_{finger}_{loc}_{ch}")

        # ── Wrist IMU ─────────────────────────────────────────────────────────
        if include_wrist:
            for ch in wrist_channels:
                cols.append(f"{hand}_wrist_{ch}")

    return cols


# =============================================================================
# DATASET DISCOVERY
# =============================================================================

def discover_dataset(data_root):
    """
    Walk data_root and return a DataFrame listing every CSV trial.

    The gesture label is inferred from the FOLDER NAME — this is the
    standard "folder-per-class" layout used in many ML datasets.

    Example return value:
        label                                      | filepath         | filename
        TwoHand_L_Fist_R_Fist_filtered_butterworth | .../trial_1.csv  | trial_1.csv
        TwoHand_L_Flat_R_Flat_filtered_butterworth  | .../trial_1.csv  | trial_1.csv
        ...

    Returns
    -------
    pd.DataFrame with columns: label, filepath, filename
    """
    rows = []
    for csv_path in sorted(Path(data_root).rglob("*.csv")):
        rows.append({
            "label":    csv_path.parent.name,   # folder name = gesture class
            "filepath": str(csv_path),
            "filename": csv_path.name,
        })
    df = pd.DataFrame(rows)
    if df.empty:
        raise FileNotFoundError(
            f"No CSV files found under: {data_root}\n"
            "Check that DATA_ROOT points to the folder containing the 4 gesture subfolders."
        )
    return df


def get_label_map(data_root):
    """
    Return a dictionary mapping each gesture folder name to an integer.

    In supervised learning (your lectures: week 5), the class labels must
    be numbers so the model can compute a loss. We assign them alphabetically:
        TwoHand_L_Fist_R_Fist...  →  0
        TwoHand_L_Flat_R_Flat...  →  1
        TwoHand_L_Okay_R_Okay...  →  2
        TwoHand_L_Rock_R_Rock...  →  3

    Returns
    -------
    dict  {label_string: integer_index}
    """
    manifest = discover_dataset(data_root)
    labels   = sorted(manifest["label"].unique())
    return {lbl: idx for idx, lbl in enumerate(labels)}


# =============================================================================
# SINGLE-FILE LOADING
# =============================================================================

def load_csv(filepath, feature_cols=None):
    """
    Load one gesture trial CSV and return only the requested feature columns.

    Steps:
    1. Read the CSV into a pandas DataFrame.
    2. Drop metadata/timing columns (run_index, timestamps, etc.) — these
       are not sensor readings and would confuse the model.
    3. Drop the always-invalid columns (palm_mid_*, palm_flex).
    4. Keep only numeric columns (drop any string identifier columns).
    5. If feature_cols is given, keep only those specific columns.

    Parameters
    ----------
    filepath     : path to the CSV file
    feature_cols : list of column names to keep (None = keep all valid features)

    Returns
    -------
    df : pandas DataFrame of shape (timesteps, n_features)
         Each row is one timestep; each column is one sensor channel.
    """
    df = pd.read_csv(filepath)

    # Remove metadata and always-invalid columns
    drop = [c for c in df.columns if c in _META_COLS or c in _ALWAYS_EXCLUDE]
    df   = df.drop(columns=drop, errors="ignore")

    # Keep only numeric data (sensor values are all floats/ints)
    df = df.select_dtypes(include=[np.number])

    if feature_cols is not None:
        missing = [c for c in feature_cols if c not in df.columns]
        present = [c for c in feature_cols if c in df.columns]
        if missing:
            warnings.warn(
                f"{os.path.basename(filepath)}: {len(missing)} requested columns "
                f"not found (will be skipped): {missing[:5]}"
                f"{'...' if len(missing) > 5 else ''}"
            )
        if not present:
            raise ValueError(
                f"None of the requested feature columns exist in {filepath}."
            )
        df = df[present]

    return df.reset_index(drop=True)


# =============================================================================
# FILTERING
# =============================================================================

def apply_filter(signal_arr, filter_type, fs,
                 bw_order=4, bw_lo=6.0, bw_hi=0.5,
                 bw_bp_lo=0.5, bw_bp_hi=10.0,
                 ma_win=5, sg_win=11, sg_poly=3, med_kern=5):
    """
    Apply a temporal filter to a 1-D numpy array of sensor values.

    WHY FILTER?
    -----------
    Raw sensor signals contain noise — small random fluctuations that don't
    represent real hand movement. Filtering smooths these out so the model
    learns from the underlying gesture pattern, not from noise.

    Your glove's firmware already applied a Butterworth low-pass filter at
    5 Hz (the '_bw_lp_5.0hz' in the filename), so the data is already quite
    smooth. FILTER_TYPE = 'none' is therefore a reasonable starting point.

    HOW FILTERS WORK (simplified)
    ------------------------------
    - Low-pass  filter: keeps slow changes (smooth gesture motion), removes
                        fast noise. Think of it as averaging nearby samples.
    - High-pass filter: keeps fast changes (sudden movements), removes slow
                        drift (e.g. the hand gradually moving position over time).
    - Band-pass filter: keeps a specific frequency range — combines both.
    - Moving average:   replace each point with the mean of its neighbours.
                        Simple but introduces lag (the filtered signal lags behind
                        the true signal slightly).
    - Savitzky-Golay:   fits a small polynomial to a window of points, then
                        evaluates it at the centre. Smoother than moving average
                        and preserves peak shapes better.
    - Median filter:    replace each point with the median of its window.
                        Excellent at removing spike noise (outliers).

    Parameters
    ----------
    signal_arr  : 1-D numpy array  (one sensor channel over time)
    filter_type : string — which filter to apply (see options above)
    fs          : sampling rate in Hz (needed to convert Hz cutoffs to
                  normalised frequencies for the Butterworth design)

    Returns
    -------
    1-D numpy array of the same length, filtered
    """
    sig = np.asarray(signal_arr, dtype=float)
    nyq = fs / 2.0      # Nyquist frequency: the highest frequency representable
                        # at this sampling rate. Used to normalise cutoff frequencies.
    ft  = filter_type.lower()

    if ft == "none":
        return sig      # pass through unchanged

    if ft == "butterworth_lp":
        # Low-pass: attenuate anything above bw_lo Hz.
        # 'filtfilt' applies the filter forward then backward to eliminate
        # phase lag (so the output is centred on the input, not shifted in time).
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
        k = np.ones(ma_win) / ma_win    # kernel: uniform weights summing to 1
        return np.convolve(sig, k, mode="same")

    if ft == "savgol":
        win = sg_win if sg_win % 2 == 1 else sg_win + 1   # window must be odd
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
    """
    Apply a filter to every sensor channel (column) in a trial DataFrame.

    Each column is filtered independently — the filter is applied along
    the time axis (rows), treating each sensor as a separate 1-D signal.

    Returns a new DataFrame (does not modify the original).
    """
    out = df.copy()
    for col in out.columns:
        out[col] = apply_filter(out[col].values, filter_type, fs)
    return out


# =============================================================================
# RESAMPLING
# =============================================================================

def resample_to_length(arr, target_len):
    """
    Resample a 2-D trial array from its original length to target_len timesteps.

    WHY RESAMPLE?
    -------------
    Neural networks require all input examples to have the same shape.
    If different trials have slightly different lengths (e.g. 154 vs 158 rows
    because the glove recording didn't always stop at exactly 5.0 s), we
    resample them all to the same TARGET_LEN.

    HOW IT WORKS
    ------------
    For each sensor channel (column), we use linear interpolation:
        - Imagine spreading the original T timesteps evenly across a unit
          interval [0, 1].
        - Then create target_len evenly-spaced points on [0, 1].
        - Read off the interpolated value at each new point.
    This is equivalent to stretching or compressing the signal in time.

    Parameters
    ----------
    arr        : numpy array of shape (T, C)  — T timesteps, C channels
    target_len : int — desired number of timesteps after resampling

    Returns
    -------
    numpy array of shape (target_len, C)
    """
    T, C = arr.shape
    if T == target_len:
        return arr          # already the right length — nothing to do

    x_old = np.linspace(0, 1, T)           # original time axis normalised to [0,1]
    x_new = np.linspace(0, 1, target_len)  # new time axis
    out   = np.zeros((target_len, C), dtype=arr.dtype)

    for c in range(C):
        # Build an interpolation function for this channel and evaluate at new points
        f         = interp1d(x_old, arr[:, c], kind="linear",
                             fill_value="extrapolate")
        out[:, c] = f(x_new)

    return out


# =============================================================================
# NORMALISATION
# =============================================================================

def normalize(X, method="minmax", per_sample=True):
    """
    Normalise a 3-D dataset array.

    WHY NORMALISE? (from your COMP3308 lectures, week 5 — KNN section)
    ------------------------------------------------------------------
    Your lectures describe this exactly: "Different attributes are measured
    on different scales. When calculating the distance between examples,
    attributes with a larger scale dominate."

    The same principle applies to neural networks. If flex sensor values range
    from 0–100 and accelerometer values range from -2 to +2, the network's
    weights will be pulled disproportionately towards the larger-scale channels
    during gradient descent. Normalising puts all channels on the same footing.

    TWO METHODS
    -----------
    'minmax' (Min-Max normalisation — from your lectures):
        a_normalised = (v - min) / (max - min)
        → rescales each channel to [0, 1]
        → preserves the relative shape of the signal

    'zscore' (Z-score / standardisation — also called "standardisation"):
        a_normalised = (v - mean) / std_dev
        → centres each channel around 0 with unit variance
        → useful when the distribution of values matters, not just the range

    PER-SAMPLE vs GLOBAL
    --------------------
    per_sample=True  (recommended):
        Fit the scaler on each individual trial independently.
        Advantage: the model sees consistent [0,1] values regardless of
        how far the hand moved from the "neutral" position at trial start.
        Disadvantage: information about absolute position is lost.

    per_sample=False:
        Fit the scaler once across all trials.
        Advantage: preserves absolute differences between trials.
        Disadvantage: one outlier trial can compress everyone else's range.

    Parameters
    ----------
    X          : numpy array of shape (N, T, C)
                 N = number of trials, T = timesteps, C = sensor channels
    method     : 'minmax' or 'zscore'
    per_sample : if True, normalise each trial independently

    Returns
    -------
    numpy float32 array of the same shape (N, T, C), normalised
    """
    N, T, C = X.shape
    out = np.zeros_like(X, dtype=np.float32)

    if per_sample:
        # Normalise each of the N trials independently
        for i in range(N):
            sc = MinMaxScaler() if method == "minmax" else StandardScaler()
            # X[i] is (T, C) — the scaler treats each column (channel) independently
            out[i] = sc.fit_transform(X[i])
    else:
        # Reshape to (N*T, C) so the scaler sees all trials together,
        # then reshape back to (N, T, C)
        flat      = X.reshape(-1, C)
        sc        = MinMaxScaler() if method == "minmax" else StandardScaler()
        flat_norm = sc.fit_transform(flat)
        out       = flat_norm.reshape(N, T, C).astype(np.float32)

    return out


# =============================================================================
# FULL DATASET BUILDER
# =============================================================================

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
    Load EVERY CSV trial under data_root, apply the full preprocessing
    pipeline, and return arrays ready to pass to a Keras model.

    This is the main entry point called by every notebook. It orchestrates
    discover → load → filter → resample → stack → normalise.

    WHAT COMES OUT
    --------------
    X : shape (N, target_len, C)  — the input to the neural network
        N = total number of trials (e.g. 400)
        target_len = timesteps per trial (e.g. 156)
        C = number of sensor channels selected (depends on CONFIG flags)

    y : shape (N,)  — integer class label for each trial (0, 1, 2, or 3)

    Relating to your lectures:
        X is the "input vector" (or in this case, input MATRIX per example)
        y is the "target output" / class label
        Together {X, y} is your labelled training set

    Parameters
    ----------
    data_root       : path to root folder with gesture label subfolders
    feature_cols    : list of column names to include (None = all valid features)
    filter_type     : temporal filter to apply (see apply_filter for options)
    fs              : sampling rate in Hz
    target_len      : resample all trials to this many timesteps
    normalization   : 'minmax' | 'zscore' | 'none'
    per_sample_norm : if True, normalise each trial independently

    Returns
    -------
    X                 : np.float32 array (N, target_len, C)
    y                 : np.int64 array   (N,)
    labels            : list[str]  — gesture name for index 0, 1, 2, 3
    feature_cols_used : list[str]  — the actual column names loaded
    """
    manifest          = discover_dataset(data_root)
    label_map         = get_label_map(data_root)
    labels            = [k for k, v in sorted(label_map.items(), key=lambda x: x[1])]
    X_list, y_list    = [], []
    feature_cols_used = None

    for _, row in manifest.iterrows():
        # Load and select columns
        try:
            df = load_csv(row["filepath"], feature_cols=feature_cols)
        except Exception as e:
            warnings.warn(f"Skipping {row['filepath']}: {e}")
            continue

        # Record which columns were actually loaded (from the first valid file)
        if feature_cols_used is None:
            feature_cols_used = df.columns.tolist()

        # Optionally apply temporal filtering
        if filter_type != "none":
            df = preprocess_dataframe(df, filter_type=filter_type, fs=fs)

        arr = df.values.astype(np.float32)          # convert to numpy: shape (T, C)

        # Drop any rows containing NaN values (e.g. from filtering artefacts)
        arr = arr[~np.isnan(arr).any(axis=1)]
        if len(arr) < 4:
            warnings.warn(f"Too few valid rows in {row['filepath']}, skipping.")
            continue

        # Resample to the target length if needed
        if arr.shape[0] != target_len:
            arr = resample_to_length(arr, target_len)

        X_list.append(arr)                          # (target_len, C)
        y_list.append(label_map[row["label"]])      # integer class label

    if not X_list:
        raise RuntimeError(
            "No samples were loaded. Check that DATA_ROOT points to the folder\n"
            "containing the four gesture label subfolders."
        )

    # Stack all trials into a single 3D array: (N, target_len, C)
    X = np.stack(X_list, axis=0).astype(np.float32)
    y = np.array(y_list, dtype=np.int64)

    # Normalise the full dataset
    if normalization != "none":
        X = normalize(X, method=normalization, per_sample=per_sample_norm)

    print(f"Dataset loaded: {X.shape[0]} samples  |  "
          f"{X.shape[1]} timesteps  |  {X.shape[2]} features  |  "
          f"{len(labels)} classes")
    print(f"  Classes ({len(labels)}): {labels}")
    print(f"  Class distribution: "
          + "  ".join(f"{lbl}={int((y == i).sum())}"
                      for i, lbl in enumerate(labels)))
    return X, y, labels, feature_cols_used


# =============================================================================
# TRAIN / VAL / TEST SPLIT
# =============================================================================

def split_dataset(X, y, train_ratio=0.70, val_ratio=0.10, seed=42):
    """
    Split the dataset into three non-overlapping subsets: train, val, test.

    WHY THREE SUBSETS? (from your COMP3308 lectures, week 6 — Evaluation)
    ----------------------------------------------------------------------
    Your lectures explain the holdout procedure and the need for a validation set:

    Training set   (70%): Used to update the model's weights during training.
                          The model SEES these examples and learns from them.

    Validation set (10%): Used to monitor performance during training — after
                          each epoch we check "is the model still improving on
                          data it hasn't trained on?".
                          This is what EarlyStopping monitors.
                          The model does NOT train on these but they influence
                          when training stops, so they can't be used for the
                          final accuracy report either.

    Test set       (20%): Held back entirely until training is finished.
                          Gives an unbiased estimate of how well the model
                          generalises to truly new data — the number you report
                          in your thesis.
                          NEVER use the test set to make decisions during training.

    STRATIFICATION
    --------------
    Your lectures mention stratification as "an improvement of the holdout method."
    Stratified split means each subset has the same class proportions as the
    full dataset. With 100 samples per class, stratification ensures each class
    appears ~70/10/20 samples in train/val/test — rather than randomly ending up
    unequal, which would give a biased accuracy estimate.

    Parameters
    ----------
    X           : (N, T, C) input array
    y           : (N,) integer labels
    train_ratio : fraction of data for training (default 0.70)
    val_ratio   : fraction for validation (default 0.10)
    seed        : random seed for reproducibility

    Returns
    -------
    (X_train, y_train), (X_val, y_val), (X_test, y_test)
    """
    from sklearn.model_selection import train_test_split

    test_ratio = 1.0 - train_ratio - val_ratio
    assert test_ratio > 0.0, "train_ratio + val_ratio must be < 1.0"

    # First split: separate out the test set
    X_tv, X_test, y_tv, y_test = train_test_split(
        X, y,
        test_size = test_ratio,
        stratify  = y,              # ensure balanced classes in test set
        random_state = seed,
    )

    # Second split: divide the remainder into train and val
    val_frac = val_ratio / (train_ratio + val_ratio)
    X_train, X_val, y_train, y_val = train_test_split(
        X_tv, y_tv,
        test_size    = val_frac,
        stratify     = y_tv,        # ensure balanced classes in val set
        random_state = seed,
    )

    print(f"Split: train={len(X_train)}  val={len(X_val)}  test={len(X_test)}")
    return (X_train, y_train), (X_val, y_val), (X_test, y_test)
