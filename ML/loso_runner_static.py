#!/usr/bin/env python3
"""
LOSO (Leave-One-Subject-Out) runner for the *glove_ml_explorer* notebook.

Sklearn analogue of loso_runner.py — runs the same LOSO protocol but with the
classical classifiers from glove_ml_explorer.ipynb (SVM RBF, SVM Linear, RF,
KNN, Gradient Boosting, Logistic Regression, LDA, Voting, Stacking) on
feature vectors extracted from the resampled trials (stats+fft by default).

Each experiment:
  - For each subject: train on the OTHER N-1 subjects with inner k-fold CV
                       + final retrain + held-out subject batch test
                       across all selected classifiers
  - Aggregates per-fold (per-held-out-subject) accuracies
  - Produces one PDF report + JSON

Usage:
  python3 loso_runner_glove.py --exp_id S_A1 [--include_nothing]
                               [--use_flex] [--use_ypr] [--use_quat] [--use_accel]
                               [--hands left,right]
                               [--segments thumb,index,middle,ring,pinky,wrist]
                               [--feature_mode stats+fft] [--normalisation minmax]
                               [--classifiers "SVM (RBF),Random Forest,..."]

Data layout (must match the notebook):
  <data_root>/<subject>/Static/<gesture>/*.csv
Subjects are auto-discovered.
"""
import os, glob, json, argparse, time, warnings
from pathlib import Path
from datetime import datetime
from collections import Counter

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from scipy import signal as scipy_signal
from scipy.stats import skew, kurtosis

from sklearn.model_selection import StratifiedKFold
from sklearn.metrics import classification_report, accuracy_score
from sklearn.preprocessing import LabelEncoder, StandardScaler, MinMaxScaler

# Classifiers (same set as glove_ml_explorer.ipynb)
from sklearn.svm import SVC
from sklearn.ensemble import (
    RandomForestClassifier, GradientBoostingClassifier,
    VotingClassifier, StackingClassifier,
)
from sklearn.neighbors import KNeighborsClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis

warnings.filterwarnings('ignore')
np.random.seed(42)

# ── CLI ──────────────────────────────────────────────────────────────────────
ap = argparse.ArgumentParser()
ap.add_argument('--exp_id', required=True)
ap.add_argument('--exp_label', default='')
ap.add_argument('--data_root', default='./NewTestData',
                help='Directory that contains one folder per subject. '
                     'Each subject folder must contain a Static/ subfolder with one folder per gesture.')
ap.add_argument('--out_dir', default='./loso_results_glove',
                help='Directory to write PDFs, plots, JSON results, and per-fold state files into.')

# Class set
ap.add_argument('--include_nothing', action='store_true',
                help='Include Double_Nothing class (excluded by default).')

# Sensor modalities (default: nothing, must be opted in — matches loso_runner.py style)
ap.add_argument('--use_ypr',   action='store_true', help='Yaw/Pitch/Roll (Euler) IMU channels')
ap.add_argument('--use_quat',  action='store_true', help='Raw quaternion IMU channels')
ap.add_argument('--use_accel', action='store_true', help='Linear accelerometer channels')
ap.add_argument('--use_flex',  action='store_true', help='Per-finger flex sensor channels')

# Hands & segments
ap.add_argument('--hands', default='left',
                help="Comma list, any of: left,right (default: left, matching notebook).")
ap.add_argument('--segments',
                default='thumb,index,middle,ring,pinky,wrist',
                help="Comma list, any of: palm_prox,thumb,index,middle,ring,pinky,wrist "
                     "(palm_mid is intentionally not exposed — no sensor data).")

# Preprocessing knobs
ap.add_argument('--resample_n', type=int, default=90)
ap.add_argument('--apply_butterworth', action='store_true')
ap.add_argument('--butter_cutoff', type=float, default=10.0)
ap.add_argument('--butter_order',  type=int,   default=4)
ap.add_argument('--sampling_rate', type=float, default=30.0)
ap.add_argument('--normalisation', default='minmax', choices=['minmax', 'standard', 'none'])
ap.add_argument('--feature_mode',  default='stats+fft',
                choices=['flatten', 'stats', 'fft', 'stats+fft'])

# Augmentation (train-only, per inner-CV fold — matches notebook's train-only augment)
ap.add_argument('--apply_augmentation', action='store_true')
ap.add_argument('--aug_copies', type=int, default=1, help='N_AUG_PER_TRAIN_TRIAL')
ap.add_argument('--aug_jitter', action='store_true')
ap.add_argument('--aug_jitter_sigma', type=float, default=0.01)
ap.add_argument('--aug_scale', action='store_true')
ap.add_argument('--aug_scale_low',  type=float, default=0.8)
ap.add_argument('--aug_scale_high', type=float, default=1.2)
ap.add_argument('--aug_time_warp', action='store_true')
ap.add_argument('--aug_warp_low',  type=float, default=0.9)
ap.add_argument('--aug_warp_high', type=float, default=1.1)

# LOSO / CV
ap.add_argument('--cv_folds', type=int, default=5,
                help='Inner CV folds inside each LOSO fold. Notebook uses 10; '
                     'we default to 5 to keep the grid affordable.')
ap.add_argument('--max_subjects', type=int, default=0)  # 0 = all
ap.add_argument('--fold_start', type=int, default=0, help='0-based fold index to start (inclusive)')
ap.add_argument('--fold_end',   type=int, default=-1, help='0-based fold index to end (exclusive). -1 = all')

# Model selection
DEFAULT_CLASSIFIERS = ('SVM (RBF),SVM (Linear),Random Forest,KNN,'
                       'Logistic Regression,LDA,'
                       'Voting Soft (SVM+RF+KNN+LogReg),Stacking (SVM+RF+KNN+LogReg)')
ap.add_argument('--classifiers', default=DEFAULT_CLASSIFIERS,
                help='Comma-separated classifier names. Available: '
                     '"SVM (RBF)", "SVM (Linear)", "Random Forest", "KNN", '
                     '"Gradient Boosting", "Logistic Regression", "LDA", '
                     '"Voting Soft (SVM+RF+KNN+LogReg)", '
                     '"Stacking (SVM+RF+KNN+LogReg)". '
                     'Gradient Boosting is OFF by default (slow).')

ap.add_argument('--resume_from', default='', help='Path to JSON of partial results to merge with')
ap.add_argument('--skip_pdf', action='store_true', help='Just save partial JSON, do not render PDF')
ap.add_argument('--state_file', default='', help='Path to per-fold state JSON. If exists, completed folds are skipped. Updated after each fold.')
args = ap.parse_args()

OUT_DIR = Path(args.out_dir)
OUT_DIR.mkdir(parents=True, exist_ok=True)

RANDOM_STATE = 42

# ── Constants ────────────────────────────────────────────────────────────────
# Static-gesture class set (mirrors INCLUDE_LABELS in the notebook).
GESTURE_LABELS_ALL = [
    'Double_Closed_Fist',
    'Double_Nothing',
    'Double_Okay',
    'Double_Open_Palm',
    'Double_Phone',
    'Double_Pistol',
    'Double_Spiderman',
]
GESTURE_LABELS = GESTURE_LABELS_ALL if args.include_nothing else \
                 [g for g in GESTURE_LABELS_ALL if g != 'Double_Nothing']

SEGMENTS_WITH_FLEX = ['thumb', 'index', 'middle', 'ring', 'pinky']  # palm has no flex

# ── Sensor columns (mirrors build_sensor_columns + seg_map in the notebook) ──
HANDS = [h.strip() for h in args.hands.split(',') if h.strip()]
for h in HANDS:
    if h not in ('left', 'right'):
        raise SystemExit(f'Bad hand: {h!r}. Use "left" and/or "right".')

# Resolve segments, deduplicating palm if both palm_mid / palm_prox were given.
SEG_MAP = {
    'palm_mid':  'palm',
    'palm_prox': 'palm',
    'thumb':     'thumb',
    'index':     'index',
    'middle':    'middle',
    'ring':      'ring',
    'pinky':     'pinky',
    'wrist':     'wrist',
}
raw_segs = [s.strip() for s in args.segments.split(',') if s.strip()]
for s in raw_segs:
    if s not in SEG_MAP:
        raise SystemExit(f'Bad segment: {s!r}. Allowed: {sorted(SEG_MAP)}')
SEGMENTS = list(dict.fromkeys(SEG_MAP[s] for s in raw_segs))

def build_sensor_columns(hands, segments, use_ypr, use_quat, use_accel, use_flex):
    cols = []
    for hand in hands:
        for seg in segments:
            if seg == 'wrist':
                p = f'{hand}_wrist'
                if use_ypr:   cols += [f'{p}_heading', f'{p}_pitch', f'{p}_roll']
                if use_quat:  cols += [f'{p}_quat_w', f'{p}_quat_x', f'{p}_quat_y', f'{p}_quat_z']
                if use_accel: cols += [f'{p}_ax', f'{p}_ay', f'{p}_az']
            else:
                for loc in ['mid', 'prox']:
                    p = f'{hand}_{seg}_{loc}'
                    if use_ypr:   cols += [f'{p}_yaw', f'{p}_pitch', f'{p}_roll']
                    if use_quat:  cols += [f'{p}_quat_w', f'{p}_quat_x', f'{p}_quat_y', f'{p}_quat_z']
                    if use_accel: cols += [f'{p}_ax', f'{p}_ay', f'{p}_az']
                if use_flex and seg in SEGMENTS_WITH_FLEX:
                    cols += [f'{hand}_{seg}_mcp_flex', f'{hand}_{seg}_pip_flex']
    return cols

SENSOR_COLS = build_sensor_columns(
    HANDS, SEGMENTS,
    args.use_ypr, args.use_quat, args.use_accel, args.use_flex,
)
if not SENSOR_COLS:
    raise SystemExit('No sensor columns selected — enable at least one modality.')

print(f'[{args.exp_id}] {args.exp_label}')
print(f'  classes:  {len(GESTURE_LABELS)} ({"+Nothing" if args.include_nothing else "no Nothing"})')
print(f'  sensors:  ypr={args.use_ypr} quat={args.use_quat} accel={args.use_accel} flex={args.use_flex}')
print(f'  hands:    {HANDS}')
print(f'  segments: {SEGMENTS}')
print(f'  channels: {len(SENSOR_COLS)}')
print(f'  features: {args.feature_mode}  norm: {args.normalisation}')
print(f'  CV inner folds: {args.cv_folds}')

# ── Subject discovery ────────────────────────────────────────────────────────
def discover_subjects(data_root, gesture_labels):
    out = []
    for sub in sorted(os.listdir(data_root)):
        if sub in ('Combined', 'Copy', 'tree.txt'):
            continue
        stat = os.path.join(data_root, sub, 'Static')
        if not os.path.isdir(stat):
            continue
        ok = True
        for g in gesture_labels:
            if not glob.glob(os.path.join(stat, g, '*.csv')):
                ok = False; break
        if ok:
            out.append(sub)
    return out

SUBJECTS = discover_subjects(args.data_root, GESTURE_LABELS)
if args.max_subjects and args.max_subjects < len(SUBJECTS):
    SUBJECTS = SUBJECTS[:args.max_subjects]
print(f'  subjects ({len(SUBJECTS)}): {", ".join(SUBJECTS)}')
if len(SUBJECTS) < 3:
    raise SystemExit('Need at least 3 subjects for LOSO.')

# ── Load / preprocess data, indexed by subject ───────────────────────────────
def load_subject(subject_dir, sensor_cols, gesture_labels):
    """Returns list of (T, C) float32 arrays plus parallel list of labels."""
    trials, labels = [], []
    for g in gesture_labels:
        for fpath in sorted(glob.glob(os.path.join(subject_dir, 'Static', g, '*.csv'))):
            try:
                df = pd.read_csv(fpath)
                avail = [c for c in sensor_cols if c in df.columns]
                if not avail:
                    continue
                # If any expected column is missing, zero-pad the rest (rare but safe).
                if len(avail) != len(sensor_cols):
                    arr = np.zeros((len(df), len(sensor_cols)), dtype=np.float32)
                    idx = {c: i for i, c in enumerate(sensor_cols)}
                    for c in avail:
                        arr[:, idx[c]] = df[c].values.astype(np.float32)
                else:
                    arr = df[sensor_cols].values.astype(np.float32)
                trials.append(arr); labels.append(g)
            except Exception as e:
                print(f'    err {os.path.basename(fpath)}: {e}')
    return trials, labels

def resample_trial(trial, n):
    T, C = trial.shape
    if T == n:
        return trial.astype(np.float32, copy=False)
    old = np.linspace(0, 1, T); new = np.linspace(0, 1, n)
    out = np.zeros((n, C), dtype=np.float32)
    for c in range(C):
        out[:, c] = np.interp(new, old, trial[:, c])
    return out

def butter_filter(trials, cutoff, order, fs):
    nyq = fs / 2.0; norm = cutoff / nyq
    if norm >= 1.0:
        return trials
    b, a = scipy_signal.butter(order, norm, btype='low')
    return [scipy_signal.filtfilt(b, a, t, axis=0).astype(np.float32) for t in trials]

print('  loading data...', flush=True)
subject_data = {}
for sub in SUBJECTS:
    raw_trials, raw_labels = load_subject(
        os.path.join(args.data_root, sub), SENSOR_COLS, GESTURE_LABELS)
    raw_trials = [resample_trial(t, args.resample_n) for t in raw_trials]
    if args.apply_butterworth:
        raw_trials = butter_filter(raw_trials, args.butter_cutoff,
                                    args.butter_order, args.sampling_rate)
    X = np.stack(raw_trials, axis=0).astype(np.float32)
    X = np.nan_to_num(X, nan=0.0, posinf=0.0, neginf=0.0)
    subject_data[sub] = (X, np.array(raw_labels))
    print(f'    {sub}: {X.shape}, {Counter(raw_labels)}', flush=True)

le = LabelEncoder().fit(GESTURE_LABELS)
n_classes = len(le.classes_)
sequence_length = args.resample_n
n_channels = len(SENSOR_COLS)

# ── Augmentation (matches augment_trials() in the notebook) ──────────────────
def jitter_trial(trial, sigma, rng):
    return (trial + rng.normal(0, sigma, size=trial.shape).astype(np.float32)).astype(np.float32)

def scale_trial(trial, lo, hi, rng):
    return (trial * np.float32(rng.uniform(lo, hi))).astype(np.float32)

def time_warp_trial(trial, lo, hi, n_steps, rng):
    factor = rng.uniform(lo, hi)
    T, C = trial.shape
    warped_len = max(5, int(T * factor))
    old = np.linspace(0, 1, T); mid = np.linspace(0, 1, warped_len)
    warped = np.zeros((warped_len, C), dtype=np.float32)
    for c in range(C):
        warped[:, c] = np.interp(mid, old, trial[:, c])
    if n_steps is not None and warped_len != n_steps:
        return resample_trial(warped, n_steps)
    return warped

def augment_trials(trials, labels, copies, seed):
    """Returns (trials, labels) including the originals plus `copies` augmented
    copies per original. Mirrors augment_trials() in the notebook."""
    if not args.apply_augmentation or copies <= 0:
        return list(trials), list(labels)
    rng = np.random.default_rng(seed)
    out_t, out_l = [], []
    for trial, label in zip(trials, labels):
        out_t.append(trial.astype(np.float32)); out_l.append(label)
        for _ in range(copies):
            t = trial.astype(np.float32)
            if args.aug_jitter:
                t = jitter_trial(t, args.aug_jitter_sigma, rng)
            if args.aug_scale:
                t = scale_trial(t, args.aug_scale_low, args.aug_scale_high, rng)
            if args.aug_time_warp:
                t = time_warp_trial(t, args.aug_warp_low, args.aug_warp_high,
                                     args.resample_n, rng)
            out_t.append(t); out_l.append(label)
    return out_t, out_l

# ── Feature extraction (matches extract_features() in the notebook) ──────────
def extract_stats_one(trial):
    return np.concatenate([
        trial.mean(axis=0),
        trial.std(axis=0),
        trial.min(axis=0),
        trial.max(axis=0),
        trial.max(axis=0) - trial.min(axis=0),
        np.sqrt((trial ** 2).mean(axis=0)),
        skew(trial, axis=0).astype(np.float32),
        kurtosis(trial, axis=0).astype(np.float32),
    ])

def extract_fft_one(trial):
    return np.abs(np.fft.rfft(trial, axis=0)).flatten().astype(np.float32)

def extract_features(trials, mode):
    X = []
    for t in trials:
        if   mode == 'flatten':   feats = t.flatten()
        elif mode == 'stats':     feats = extract_stats_one(t)
        elif mode == 'fft':       feats = extract_fft_one(t)
        elif mode == 'stats+fft': feats = np.concatenate([extract_stats_one(t),
                                                          extract_fft_one(t)])
        else: raise ValueError(f'Unknown feature_mode {mode}')
        X.append(feats)
    X = np.array(X, dtype=np.float32)
    return np.nan_to_num(X, nan=0.0, posinf=0.0, neginf=0.0)

def make_scaler(kind):
    if kind == 'standard': return StandardScaler()
    if kind == 'minmax':   return MinMaxScaler()
    return None

# ── Classifier factories (fresh estimator per fit to avoid state leakage) ────
def _base_estimators():
    """Re-instantiate the base set used by Voting/Stacking. Defining inline so
    every call produces brand-new estimators (sklearn ensembles will clone, but
    keeping this explicit avoids surprises)."""
    return [
        ('svm_rbf', SVC(kernel='rbf', C=10, gamma='scale',
                        random_state=RANDOM_STATE, probability=True)),
        ('rf',     RandomForestClassifier(n_estimators=200,
                                           random_state=RANDOM_STATE, n_jobs=-1)),
        ('knn',    KNeighborsClassifier(n_neighbors=5, metric='euclidean')),
        ('logreg', LogisticRegression(max_iter=1000, random_state=RANDOM_STATE)),
    ]

CLASSIFIER_FACTORIES = {
    'SVM (RBF)':           lambda: SVC(kernel='rbf', C=10, gamma='scale',
                                        random_state=RANDOM_STATE, probability=True),
    'SVM (Linear)':        lambda: SVC(kernel='linear', C=1,
                                        random_state=RANDOM_STATE, probability=True),
    'Random Forest':       lambda: RandomForestClassifier(n_estimators=200,
                                                          random_state=RANDOM_STATE, n_jobs=-1),
    'KNN':                 lambda: KNeighborsClassifier(n_neighbors=5, metric='euclidean'),
    'Gradient Boosting':   lambda: GradientBoostingClassifier(n_estimators=100,
                                                              random_state=RANDOM_STATE),
    'Logistic Regression': lambda: LogisticRegression(max_iter=1000,
                                                      random_state=RANDOM_STATE),
    'LDA':                 lambda: LinearDiscriminantAnalysis(),
    'Voting Soft (SVM+RF+KNN+LogReg)': lambda: VotingClassifier(
        estimators=_base_estimators(), voting='soft', n_jobs=-1),
    'Stacking (SVM+RF+KNN+LogReg)':   lambda: StackingClassifier(
        estimators=_base_estimators(),
        final_estimator=LogisticRegression(max_iter=1000, random_state=RANDOM_STATE),
        passthrough=False, n_jobs=-1),
}

WANT = [c.strip() for c in args.classifiers.split(',') if c.strip()]
unknown = [c for c in WANT if c not in CLASSIFIER_FACTORIES]
if unknown:
    raise SystemExit(f'Unknown classifiers: {unknown}\n'
                     f'Available: {list(CLASSIFIER_FACTORIES.keys())}')
VARIANTS = [(c, CLASSIFIER_FACTORIES[c]) for c in WANT]
print(f'  classifiers ({len(VARIANTS)}): {[v[0] for v in VARIANTS]}', flush=True)

# ── LOSO loop ────────────────────────────────────────────────────────────────
loso_results = []  # list of dicts, one per held-out subject
total_t0 = time.time()

# Resume from state file if present
state_path = Path(args.state_file) if args.state_file else None
completed_held_out = set()
if state_path and state_path.exists():
    try:
        with open(state_path) as fh:
            state = json.load(fh)
        loso_results = state.get('loso_results', [])
        completed_held_out = {r['held_out'] for r in loso_results}
        print(f'  resumed from state: {len(completed_held_out)} completed folds: {sorted(completed_held_out)}', flush=True)
    except Exception as e:
        print(f'  state file unreadable ({e}), starting fresh', flush=True)

subjects_to_run = SUBJECTS[args.fold_start : (args.fold_end if args.fold_end >= 0 else len(SUBJECTS))]
subjects_to_run = [s for s in subjects_to_run if s not in completed_held_out]
print(f'  running folds: {args.fold_start}..{args.fold_start + len(subjects_to_run) - 1} '
      f'({len(subjects_to_run)} subjects to do: {subjects_to_run})', flush=True)

def fit_predict(name, factory, X_tr, y_tr, X_te):
    clf = factory()
    clf.fit(X_tr, y_tr)
    return clf, clf.predict(X_te)

for i_fold, held_out in enumerate(subjects_to_run, start=args.fold_start + 1):
    print(f'\n  ── LOSO fold {i_fold}/{len(SUBJECTS)}: held-out={held_out} ──', flush=True)
    fold_t0 = time.time()

    train_subjects = [s for s in SUBJECTS if s != held_out]
    # Stack the *resampled+filtered* trials (still in (T, C) form) so we can
    # apply train-only augmentation per inner-CV fold before feature extraction.
    Xtr_seq = np.concatenate([subject_data[s][0] for s in train_subjects], axis=0)
    ytr_str = np.concatenate([subject_data[s][1] for s in train_subjects], axis=0)
    ytr_all = le.transform(ytr_str)
    Xte_seq = subject_data[held_out][0]
    yte     = le.transform(subject_data[held_out][1])

    # Held-out test features (no augmentation, fit-on-train scaler applied later)
    X_te_feat_raw = extract_features(list(Xte_seq), args.feature_mode)

    print(f'    train: {Xtr_seq.shape[0]} trials from {len(train_subjects)} subjects, '
          f'test: {Xte_seq.shape[0]} trials', flush=True)

    fold_variant_records = []
    for v_name, v_factory in VARIANTS:
        v_t0 = time.time()

        # ── Inner CV: aug + feature-extract + scale INSIDE each fold ────────
        cv_accs = []
        if args.cv_folds > 1:
            skf = StratifiedKFold(n_splits=args.cv_folds, shuffle=True,
                                   random_state=RANDOM_STATE)
            for inner_idx, (tr_idx, va_idx) in enumerate(skf.split(Xtr_seq, ytr_all), start=1):
                seq_tr = [Xtr_seq[i] for i in tr_idx]
                lab_tr = ytr_all[tr_idx]
                seq_va = [Xtr_seq[i] for i in va_idx]
                lab_va = ytr_all[va_idx]

                seq_tr_aug, lab_tr_aug = augment_trials(
                    seq_tr, list(lab_tr), args.aug_copies, RANDOM_STATE + inner_idx)
                lab_tr_aug = np.asarray(lab_tr_aug)

                X_tr_f = extract_features(seq_tr_aug, args.feature_mode)
                X_va_f = extract_features(seq_va,     args.feature_mode)

                sc = make_scaler(args.normalisation)
                if sc is not None:
                    X_tr_f = sc.fit_transform(X_tr_f)
                    X_va_f = sc.transform(X_va_f)

                _, pred = fit_predict(v_name, v_factory, X_tr_f, lab_tr_aug, X_va_f)
                cv_accs.append(float(accuracy_score(lab_va, pred)))
        cv_mean = float(np.mean(cv_accs)) if cv_accs else None
        cv_std  = float(np.std(cv_accs))  if cv_accs else None

        # ── Final retrain on full train pool, evaluate on held-out subject ──
        seq_full_aug, lab_full_aug = augment_trials(
            list(Xtr_seq), list(ytr_all), args.aug_copies, RANDOM_STATE)
        lab_full_aug = np.asarray(lab_full_aug)

        X_tr_f = extract_features(seq_full_aug, args.feature_mode)
        X_te_f = X_te_feat_raw.copy()
        sc = make_scaler(args.normalisation)
        if sc is not None:
            X_tr_f = sc.fit_transform(X_tr_f)
            X_te_f = sc.transform(X_te_f)

        clf, y_pred = fit_predict(v_name, v_factory, X_tr_f, lab_full_aug, X_te_f)
        test_acc = float(accuracy_score(yte, y_pred))
        rep = classification_report(yte, y_pred, target_names=le.classes_,
                                     output_dict=True, zero_division=0)

        # Per-class held-out accuracy
        per_class = {}
        for c_idx, c_name in enumerate(le.classes_):
            mask = (yte == c_idx); n = int(mask.sum())
            if n > 0:
                per_class[c_name] = {
                    'correct':  int((y_pred[mask] == c_idx).sum()),
                    'total':    n,
                    'accuracy': float((y_pred[mask] == c_idx).mean()),
                }
            else:
                per_class[c_name] = {'correct': 0, 'total': 0, 'accuracy': None}

        # n_params surrogate: count of learned parameters where it makes sense
        # (otherwise 0 — sklearn estimators don't expose a uniform count).
        try:
            if hasattr(clf, 'coef_'):
                n_params = int(np.size(clf.coef_) + (np.size(clf.intercept_)
                                                     if hasattr(clf, 'intercept_') else 0))
            elif hasattr(clf, 'estimators_') and hasattr(clf.estimators_, '__iter__'):
                # tree ensembles: count nodes across all trees
                n_params = int(sum(getattr(e, 'tree_', None).node_count
                                    for e in clf.estimators_
                                    if getattr(e, 'tree_', None) is not None))
            else:
                n_params = 0
        except Exception:
            n_params = 0

        fold_variant_records.append({
            'name': v_name, 'n_params': n_params,
            'cv_mean': cv_mean, 'cv_std': cv_std, 'cv_accs': cv_accs,
            'test_acc': test_acc,
            'classification_report': rep, 'per_class': per_class,
            'fit_seconds': float(time.time() - v_t0),
        })
        cv_str = f'{cv_mean:.3f}\u00b1{cv_std:.3f}' if cv_mean is not None else '   —    '
        print(f'    {v_name:<38} cv={cv_str}  test={test_acc:.3f}  '
              f'({time.time()-v_t0:.1f}s)', flush=True)

    loso_results.append({
        'held_out': held_out,
        'n_train_trials': int(Xtr_seq.shape[0]),
        'n_test_trials':  int(Xte_seq.shape[0]),
        'variants': fold_variant_records,
    })
    print(f'    fold time: {(time.time()-fold_t0)/60:.1f} min', flush=True)

    if state_path:
        with open(state_path, 'w') as fh:
            json.dump({
                'exp_id': args.exp_id, 'exp_label': args.exp_label,
                'config': vars(args), 'subjects': SUBJECTS,
                'loso_results': loso_results,
            }, fh, indent=2, default=str)
        print(f'    state saved: {len(loso_results)} folds total', flush=True)

print(f'\nTotal LOSO time: {(time.time()-total_t0)/60:.1f} min', flush=True)

# Merge with prior partial results if requested
if args.resume_from and os.path.exists(args.resume_from):
    print(f'  merging with {args.resume_from}', flush=True)
    with open(args.resume_from) as fh:
        prior = json.load(fh)
    seen = {r['held_out'] for r in loso_results}
    for r in prior.get('loso_results', []):
        if r['held_out'] not in seen:
            loso_results.append(r)
    loso_results.sort(key=lambda r: SUBJECTS.index(r['held_out']) if r['held_out'] in SUBJECTS else 999)

if args.skip_pdf:
    ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    out_json = OUT_DIR / f'loso_glove_{args.exp_id}_partial_{ts}.json'
    with open(out_json, 'w') as fh:
        json.dump({
            'exp_id': args.exp_id, 'exp_label': args.exp_label,
            'config': vars(args), 'subjects': SUBJECTS,
            'loso_results': loso_results,
        }, fh, indent=2, default=str)
    print(f'PARTIAL JSON: {out_json}', flush=True)
    print('DONE_PARTIAL', flush=True)
    raise SystemExit(0)

# ── Aggregate across folds ───────────────────────────────────────────────────
def _rec(fold, name):
    return fold['variants'][[v['name'] for v in fold['variants']].index(name)]

variant_summary = {}
for v_name, _ in VARIANTS:
    test_accs = [_rec(f, v_name)['test_acc'] for f in loso_results]
    cv_means  = [_rec(f, v_name)['cv_mean']  for f in loso_results]
    cv_means  = [c for c in cv_means if c is not None]
    variant_summary[v_name] = {
        'test_mean': float(np.mean(test_accs)),
        'test_std':  float(np.std(test_accs)),
        'test_min':  float(np.min(test_accs)),
        'test_max':  float(np.max(test_accs)),
        'cv_mean_of_means':    float(np.mean(cv_means)) if cv_means else None,
        'cv_std_across_folds': float(np.std(cv_means)) if cv_means else None,
        'fold_test_accs': test_accs,
        'n_params': _rec(loso_results[0], v_name)['n_params'],
    }

# ── Plots ────────────────────────────────────────────────────────────────────
ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
names = [v[0] for v in VARIANTS]

# 1. Per-subject test accuracy heatmap
fig, ax = plt.subplots(figsize=(max(8, len(SUBJECTS)*0.7), max(4, 0.35*len(names)+1)))
mat = np.array([[_rec(f, n)['test_acc'] for f in loso_results] for n in names])
im = ax.imshow(mat, cmap='RdYlGn', vmin=0.0, vmax=1.0, aspect='auto')
ax.set_xticks(np.arange(len(SUBJECTS))); ax.set_xticklabels(SUBJECTS, rotation=45, ha='right', fontsize=8)
ax.set_yticks(np.arange(len(names)));    ax.set_yticklabels(names, fontsize=8)
for i in range(mat.shape[0]):
    for j in range(mat.shape[1]):
        ax.text(j, i, f'{mat[i,j]:.2f}', ha='center', va='center', fontsize=6.5,
                color='black' if mat[i,j] > 0.5 else 'white')
plt.colorbar(im, ax=ax, label='Held-out test accuracy', shrink=0.8)
ax.set_title(f'{args.exp_id}: per-subject LOSO accuracy by classifier')
plt.tight_layout()
heatmap_path = OUT_DIR / f'{args.exp_id}_heatmap_{ts}.png'
plt.savefig(heatmap_path, dpi=140); plt.close()

# 2. Aggregate bar chart
fig, ax = plt.subplots(figsize=(max(8, 0.9*len(names)+2), 4.5))
means = [variant_summary[n]['test_mean'] for n in names]
stds  = [variant_summary[n]['test_std']  for n in names]
ax.bar(np.arange(len(names)), means, yerr=stds, capsize=4, color='#4c78a8')
ax.set_xticks(np.arange(len(names))); ax.set_xticklabels(names, rotation=30, ha='right', fontsize=8)
ax.set_ylim(0, 1.05); ax.set_ylabel('LOSO held-out accuracy (mean ± std)')
ax.set_title(f'{args.exp_id}: LOSO accuracy by classifier (n={len(SUBJECTS)} folds)')
ax.grid(axis='y', alpha=0.3)
for i, v in enumerate(means):
    ax.text(i, v + max(stds[i], 0.01) + 0.02, f'{v:.3f}', ha='center', fontsize=8)
plt.tight_layout()
bar_path = OUT_DIR / f'{args.exp_id}_bar_{ts}.png'
plt.savefig(bar_path, dpi=140); plt.close()

# 3. Per-variant box plot of fold accuracies
fig, ax = plt.subplots(figsize=(max(8, 0.9*len(names)+2), 4.5))
data = [variant_summary[n]['fold_test_accs'] for n in names]
ax.boxplot(data, labels=names)
ax.set_xticklabels(names, rotation=30, ha='right', fontsize=8)
ax.set_ylabel('Per-fold held-out accuracy')
ax.set_ylim(0, 1.05); ax.grid(axis='y', alpha=0.3)
ax.set_title(f'{args.exp_id}: distribution of LOSO fold accuracies')
plt.tight_layout()
box_path = OUT_DIR / f'{args.exp_id}_box_{ts}.png'
plt.savefig(box_path, dpi=140); plt.close()

# ── PDF ──────────────────────────────────────────────────────────────────────
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import mm
from reportlab.lib import colors
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Table,
                                 TableStyle, Image, PageBreak)
from reportlab.lib.enums import TA_LEFT

def _fmt(v, nd=4):
    if v is None: return '\u2014'
    if isinstance(v, float): return f'{v:.{nd}f}'
    return str(v)

styles = getSampleStyleSheet()
styles.add(ParagraphStyle(name='Section',    parent=styles['Heading2'], fontSize=11,
                          spaceBefore=6, spaceAfter=2, textColor=colors.HexColor('#1a3a6c')))
styles.add(ParagraphStyle(name='SubSection', parent=styles['Heading3'], fontSize=9.5,
                          spaceBefore=4, spaceAfter=1, textColor=colors.HexColor('#2a4a7c')))
styles.add(ParagraphStyle(name='Small',      parent=styles['BodyText'], fontSize=8, leading=10))
styles['Title'].fontSize = 15; styles['Title'].spaceAfter = 4; styles['Title'].alignment = TA_LEFT
KV_K = ParagraphStyle('KV_K', fontName='Helvetica-Bold', fontSize=7.5, leading=9.5)
KV_V = ParagraphStyle('KV_V', fontName='Helvetica', fontSize=7.5, leading=9.5)

def P(text, style):
    s = str(text).replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
    return Paragraph(s, style)

pdf_path = OUT_DIR / f'loso_glove_{args.exp_id}_{ts}.pdf'
doc = SimpleDocTemplate(str(pdf_path), pagesize=A4,
                        leftMargin=12*mm, rightMargin=12*mm,
                        topMargin=10*mm, bottomMargin=10*mm,
                        title=f'LOSO glove {args.exp_id}')
story = []

# Header
story.append(Paragraph(f'LOSO (glove_ml_explorer) Experiment {args.exp_id}', styles['Title']))
story.append(Paragraph(args.exp_label or '', styles['Small']))
story.append(Paragraph(f"Generated {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}", styles['Small']))
story.append(Spacer(1, 4))

# Config
story.append(Paragraph('<b>Experiment configuration</b>', styles['Section']))
sensor_modalities = ', '.join(filter(None, [
    'flex'  if args.use_flex  else None,
    'YPR'   if args.use_ypr   else None,
    'quat'  if args.use_quat  else None,
    'accel' if args.use_accel else None,
]))
aug_bits = []
if args.apply_augmentation:
    if args.aug_jitter:    aug_bits.append(f'jitter \u03c3={args.aug_jitter_sigma}')
    if args.aug_scale:     aug_bits.append(f'scale [{args.aug_scale_low},{args.aug_scale_high}]')
    if args.aug_time_warp: aug_bits.append(f'time_warp [{args.aug_warp_low},{args.aug_warp_high}]')
aug_str = ('off' if not args.apply_augmentation
           else f'{args.aug_copies} cop/trial: ' + ', '.join(aug_bits or ['(no ops enabled)']))
cfg_rows = [
    ('Class set', f'{len(GESTURE_LABELS)} classes ' + ('including' if args.include_nothing else 'excluding') + ' Double_Nothing'),
    ('Classes', ', '.join(GESTURE_LABELS)),
    ('Sensor modalities', sensor_modalities),
    ('Hands', ', '.join(HANDS)),
    ('Segments', ', '.join(SEGMENTS)),
    ('Sensor channels', str(n_channels)),
    ('Sequence length', f'{sequence_length} steps (resampled)'),
    ('Filter', f'Butterworth {args.butter_cutoff}Hz order {args.butter_order}'
                if args.apply_butterworth else 'off'),
    ('Feature mode',   args.feature_mode),
    ('Normalisation',  args.normalisation),
    ('Augmentation',   aug_str),
    ('Inner CV folds', str(args.cv_folds)),
    ('LOSO subjects',  f'{len(SUBJECTS)}: {", ".join(SUBJECTS)}'),
]
ct = Table([[P(k, KV_K), P(v, KV_V)] for k, v in cfg_rows],
           colWidths=(50*mm, 135*mm), hAlign='LEFT')
ct.setStyle(TableStyle([
    ('VALIGN', (0,0),(-1,-1),'TOP'),
    ('BOTTOMPADDING',(0,0),(-1,-1),1), ('TOPPADDING',(0,0),(-1,-1),1),
    ('LINEBELOW',(0,0),(-1,-2), 0.25, colors.HexColor('#dddddd'))]))
story.append(ct)

# Summary table
story.append(Paragraph('<b>LOSO summary across all subjects</b>', styles['Section']))
hdr = ['Classifier', 'Params*', 'LOSO mean', 'LOSO std', 'Min fold', 'Max fold',
       'Inner CV mean']
rows = [hdr]
best = max(VARIANTS, key=lambda v: variant_summary[v[0]]['test_mean'])[0]
for v_name, _ in VARIANTS:
    s = variant_summary[v_name]
    rows.append([v_name, f'{s["n_params"]:,}' if s['n_params'] else '\u2014',
                 _fmt(s['test_mean'], 4), _fmt(s['test_std'], 4),
                 _fmt(s['test_min'], 4), _fmt(s['test_max'], 4),
                 _fmt(s['cv_mean_of_means'], 4)])
t = Table(rows, hAlign='LEFT',
          colWidths=(60*mm, 18*mm, 22*mm, 18*mm, 18*mm, 18*mm, 24*mm))
sty = [('FONT',(0,0),(-1,-1),'Helvetica',7.5),
       ('FONT',(0,0),(-1,0),'Helvetica-Bold',7.5),
       ('BACKGROUND',(0,0),(-1,0),colors.HexColor('#eef2f7')),
       ('GRID',(0,0),(-1,-1),0.25,colors.HexColor('#cccccc')),
       ('VALIGN',(0,0),(-1,-1),'MIDDLE'),
       ('ALIGN',(1,1),(-1,-1),'CENTER'),
       ('BOTTOMPADDING',(0,0),(-1,-1),1), ('TOPPADDING',(0,0),(-1,-1),1)]
for i, (v_name, _) in enumerate(VARIANTS, start=1):
    if v_name == best:
        sty.append(('BACKGROUND',(0,i),(-1,i),colors.HexColor('#fff7e0')))
t.setStyle(TableStyle(sty))
story.append(t)
story.append(Spacer(1, 2))
story.append(Paragraph(
    '*Params: linear models use coef_+intercept size; tree ensembles use total '
    'node count across trees. SVM-RBF / KNN / Voting / Stacking are non-parametric '
    'or composite, so they show as —.',
    styles['Small']))
story.append(Paragraph(f'Best classifier by LOSO mean: <b>{best}</b>', styles['Small']))

# Bar + box plots
img_w = 175*mm
story.append(Image(str(bar_path), width=img_w, height=img_w*0.5))
story.append(Image(str(box_path), width=img_w, height=img_w*0.5))

# Heatmap
story.append(Paragraph('<b>Per-subject LOSO accuracy heatmap</b>', styles['Section']))
story.append(Image(str(heatmap_path), width=img_w, height=img_w*0.42))

# Per-fold detail table
story.append(PageBreak())
story.append(Paragraph('<b>Per-fold detail (held-out subject = each row)</b>', styles['Section']))
hdr = ['Held-out subj', 'Train trials'] + [v[0] for v in VARIANTS]
rows = [hdr]
for f in loso_results:
    row = [f['held_out'], f['n_train_trials']]
    for v_name, _ in VARIANTS:
        row.append(_fmt(_rec(f, v_name)['test_acc'], 3))
    rows.append(row)
cell_style = ParagraphStyle('cell', fontName='Helvetica', fontSize=6.5, leading=8)
rows_p = [[P(c, cell_style) for c in r] for r in rows]
t = Table(rows_p, hAlign='LEFT')
t.setStyle(TableStyle([
    ('FONT',(0,0),(-1,-1),'Helvetica',6.5),
    ('FONT',(0,0),(-1,0),'Helvetica-Bold',6.5),
    ('BACKGROUND',(0,0),(-1,0),colors.HexColor('#eef2f7')),
    ('GRID',(0,0),(-1,-1),0.25,colors.HexColor('#cccccc')),
    ('VALIGN',(0,0),(-1,-1),'MIDDLE'),
    ('ALIGN',(1,1),(-1,-1),'CENTER'),
    ('BOTTOMPADDING',(0,0),(-1,-1),1), ('TOPPADDING',(0,0),(-1,-1),1),
]))
story.append(t)

# Per-classifier detail pages
for v_name, _ in VARIANTS:
    story.append(PageBreak())
    s = variant_summary[v_name]
    story.append(Paragraph(f'Classifier: {v_name}', styles['Title']))
    story.append(Paragraph(f'Params (best-effort): {s["n_params"]:,}' if s['n_params']
                            else 'Params (best-effort): —', styles['Small']))
    story.append(Paragraph(
        f'LOSO mean accuracy: {s["test_mean"]:.4f} ± {s["test_std"]:.4f} '
        f'(min {s["test_min"]:.4f}, max {s["test_max"]:.4f})', styles['Small']))
    story.append(Spacer(1, 4))

    story.append(Paragraph('<b>Per-fold per-class held-out accuracy</b>', styles['SubSection']))
    cls_hdr = ['Held-out'] + GESTURE_LABELS + ['Overall']
    rows = [cls_hdr]
    for f in loso_results:
        rec = _rec(f, v_name)
        row = [f['held_out']]
        for c in GESTURE_LABELS:
            pc = rec['per_class'].get(c)
            row.append('\u2014' if (pc is None or pc['total'] == 0)
                       else f'{pc["correct"]}/{pc["total"]}')
        row.append(_fmt(rec['test_acc'], 3))
        rows.append(row)
    agg_row = ['MEAN']
    for c in GESTURE_LABELS:
        accs = []
        for f in loso_results:
            pc = _rec(f, v_name)['per_class'].get(c)
            if pc and pc['total'] > 0:
                accs.append(pc['accuracy'])
        agg_row.append(_fmt(float(np.mean(accs)), 3) if accs else '\u2014')
    agg_row.append(_fmt(s['test_mean'], 3))
    rows.append(agg_row)

    rows_p = [[P(c, cell_style) for c in r] for r in rows]
    cw = [22*mm] + [(160/len(GESTURE_LABELS))*mm]*len(GESTURE_LABELS) + [16*mm]
    pt = Table(rows_p, hAlign='LEFT', colWidths=cw)
    pt.setStyle(TableStyle([
        ('FONT',(0,0),(-1,-1),'Helvetica',6.5),
        ('FONT',(0,0),(-1,0),'Helvetica-Bold',6.5),
        ('FONT',(0,-1),(-1,-1),'Helvetica-Bold',6.5),
        ('BACKGROUND',(0,0),(-1,0),colors.HexColor('#eef2f7')),
        ('BACKGROUND',(0,-1),(-1,-1),colors.HexColor('#fff7e0')),
        ('GRID',(0,0),(-1,-1),0.25,colors.HexColor('#cccccc')),
        ('VALIGN',(0,0),(-1,-1),'MIDDLE'),
        ('ALIGN',(1,1),(-1,-1),'CENTER'),
        ('BOTTOMPADDING',(0,0),(-1,-1),1), ('TOPPADDING',(0,0),(-1,-1),1),
    ]))
    story.append(pt)

doc.build(story)
print(f'\nPDF: {pdf_path}', flush=True)

out_json = OUT_DIR / f'loso_glove_{args.exp_id}_{ts}.json'
with open(out_json, 'w') as fh:
    json.dump({
        'exp_id': args.exp_id, 'exp_label': args.exp_label,
        'config': vars(args), 'subjects': SUBJECTS,
        'variant_summary': variant_summary,
        'loso_results': loso_results,
    }, fh, indent=2, default=str)
print(f'JSON: {out_json}', flush=True)
print('DONE', flush=True)
