#!/usr/bin/env python3
"""
LOSO (Leave-One-Subject-Out) runner for 1D CNN variants.

Runs one experiment at a time. Each experiment:
  - For each subject: train on the OTHER N-1 subjects with 3-fold inner CV
                       + final retrain + held-out subject batch test
                       across all 5 variants
  - Aggregates per-fold (per-held-out-subject) accuracies
  - Produces one PDF report

Usage:
  python3 loso_runner.py --exp_id A1 [--subjects 11] [--include_nothing]
                         [--use_flex] [--use_ypr] [--use_accel]
                         [--use_left] [--use_right]
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
from sklearn.model_selection import StratifiedKFold
from sklearn.metrics import classification_report, confusion_matrix, accuracy_score
from sklearn.preprocessing import LabelEncoder, StandardScaler, MinMaxScaler

import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import (
    Conv1D, MaxPooling1D, Flatten, Dense, Dropout,
    BatchNormalization, GlobalAveragePooling1D, Activation, Input,
)

warnings.filterwarnings('ignore')
np.random.seed(42)
tf.random.set_seed(42)

# ── CLI ──────────────────────────────────────────────────────────────────────
ap = argparse.ArgumentParser()
ap.add_argument('--exp_id', required=True)
ap.add_argument('--exp_label', default='')
ap.add_argument('--data_root', default='./NewTestData/NewTestData',
                help='Path to the directory that contains one folder per subject. '
                     'Each subject folder must contain a Dynamic/ subfolder with one folder per gesture.')
ap.add_argument('--out_dir', default='./loso_results',
                help='Directory to write PDFs, plots, JSON results, and per-fold state files into. '
                     'Defaults to ./loso_results relative to the current working directory '
                     '(typically ML/loso_results when run from the ML/ folder).')
ap.add_argument('--include_nothing', action='store_true')
ap.add_argument('--use_flex',  action='store_true')
ap.add_argument('--use_ypr',   action='store_true')
ap.add_argument('--use_accel', action='store_true')
ap.add_argument('--use_left',  action='store_true')
ap.add_argument('--use_right', action='store_true')
ap.add_argument('--epochs', type=int, default=8)
ap.add_argument('--cv_folds', type=int, default=3)
ap.add_argument('--max_subjects', type=int, default=0)  # 0 = all
ap.add_argument('--variants', default='Baseline,Shallow,Deep,BN_GAP,WideKernel')
ap.add_argument('--fold_start', type=int, default=0, help='0-based fold index to start (inclusive)')
ap.add_argument('--fold_end',   type=int, default=-1, help='0-based fold index to end (exclusive). -1 = all')
ap.add_argument('--resume_from', default='', help='Path to JSON of partial results to merge with')
ap.add_argument('--skip_pdf', action='store_true', help='Just save partial JSON, do not render PDF')
ap.add_argument('--state_file', default='', help='Path to per-fold state JSON. If exists, completed folds are skipped. Updated after each fold.')
args = ap.parse_args()

OUT_DIR = Path(args.out_dir)
OUT_DIR.mkdir(parents=True, exist_ok=True)

# ── Constants ────────────────────────────────────────────────────────────────
GESTURE_LABELS_ALL = ['Double_Lower', 'Double_Nothing', 'Double_Pistol_Recoil',
                      'Double_Raise', 'Double_Wiggle', 'Double_cmere', 'Drum_Roll']
GESTURE_LABELS = GESTURE_LABELS_ALL if args.include_nothing else \
                 [g for g in GESTURE_LABELS_ALL if g != 'Double_Nothing']

RESAMPLE_TO_N_STEPS  = 90
APPLY_BUTTERWORTH    = True
BUTTERWORTH_CUTOFF   = 10.0
BUTTERWORTH_ORDER    = 4
SAMPLING_RATE_HZ     = 30.0
NORMALISATION        = 'minmax'
RANDOM_STATE         = 42
TRAIN_BATCH_SIZE     = 16
TRAIN_VALIDATION_SPLIT = 0.2
SEGMENTS_WITH_FLEX   = ['thumb', 'index', 'middle', 'ring', 'pinky']
ALL_SEGS             = ['wrist', 'palm', 'thumb', 'index', 'middle', 'ring', 'pinky']

AUGMENT_CONFIG = {
    'amplitude_scale': {'enabled': True,  'apply_prob': 0.5, 'scale_range': (0.95, 1.05)},
    'baseline_offset': {'enabled': True,  'apply_prob': 0.5, 'offset_std_ratio': 0.02},
}
AUGMENT_COPIES = 3

# ── Sensor columns ───────────────────────────────────────────────────────────
def build_sensor_columns(use_left, use_right, use_ypr, use_accel, use_flex):
    hands = []
    if use_left:  hands.append('left')
    if use_right: hands.append('right')
    cols = []
    for hand in hands:
        for seg in ALL_SEGS:
            if seg == 'wrist':
                p = f'{hand}_wrist'
                if use_ypr:   cols += [f'{p}_heading', f'{p}_pitch', f'{p}_roll']
                if use_accel: cols += [f'{p}_ax', f'{p}_ay', f'{p}_az']
            else:
                for loc in ['mid', 'prox']:
                    p = f'{hand}_{seg}_{loc}'
                    if use_ypr:   cols += [f'{p}_yaw', f'{p}_pitch', f'{p}_roll']
                    if use_accel: cols += [f'{p}_ax', f'{p}_ay', f'{p}_az']
                if use_flex and seg in SEGMENTS_WITH_FLEX:
                    cols += [f'{hand}_{seg}_mcp_flex', f'{hand}_{seg}_pip_flex']
    return cols

SENSOR_COLS = build_sensor_columns(args.use_left, args.use_right,
                                    args.use_ypr, args.use_accel, args.use_flex)
if not SENSOR_COLS:
    raise SystemExit('No sensor columns selected — aborting.')

print(f'[{args.exp_id}] {args.exp_label}')
print(f'  classes:  {len(GESTURE_LABELS)} ({"+Nothing" if args.include_nothing else "no Nothing"})')
print(f'  sensors:  flex={args.use_flex} ypr={args.use_ypr} accel={args.use_accel}')
print(f'  hands:    left={args.use_left} right={args.use_right}')
print(f'  channels: {len(SENSOR_COLS)}')

# ── Subject discovery ────────────────────────────────────────────────────────
def discover_subjects(data_root, gesture_labels):
    out = []
    for sub in sorted(os.listdir(data_root)):
        if sub in ('Combined', 'Copy', 'tree.txt'): continue
        dyn = os.path.join(data_root, sub, 'Dynamic')
        if not os.path.isdir(dyn): continue
        ok = True
        for g in gesture_labels:
            if not glob.glob(os.path.join(dyn, g, '*.csv')):
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

# ── Load all data, indexed by subject ────────────────────────────────────────
def load_subject(subject_dir, sensor_cols, gesture_labels):
    trials, labels = [], []
    for g in gesture_labels:
        for fpath in sorted(glob.glob(os.path.join(subject_dir, 'Dynamic', g, '*.csv'))):
            try:
                df = pd.read_csv(fpath)
                avail = [c for c in sensor_cols if c in df.columns]
                if not avail: continue
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
    if T == n: return trial.astype(np.float32, copy=False)
    old = np.linspace(0, 1, T); new = np.linspace(0, 1, n)
    out = np.zeros((n, C), dtype=np.float32)
    for c in range(C):
        out[:, c] = np.interp(new, old, trial[:, c])
    return out

def butter_filter(trials, cutoff, order, fs):
    nyq = fs/2; norm = cutoff/nyq
    if norm >= 1.0: return trials
    b, a = scipy_signal.butter(order, norm, btype='low')
    return [scipy_signal.filtfilt(b, a, t, axis=0).astype(np.float32) for t in trials]

print('  loading data...', flush=True)
subject_data = {}
for sub in SUBJECTS:
    raw_trials, raw_labels = load_subject(os.path.join(args.data_root, sub), SENSOR_COLS, GESTURE_LABELS)
    raw_trials = [resample_trial(t, RESAMPLE_TO_N_STEPS) for t in raw_trials]
    if APPLY_BUTTERWORTH:
        raw_trials = butter_filter(raw_trials, BUTTERWORTH_CUTOFF, BUTTERWORTH_ORDER, SAMPLING_RATE_HZ)
    X = np.stack(raw_trials, axis=0).astype(np.float32)
    X = np.nan_to_num(X, nan=0.0, posinf=0.0, neginf=0.0)
    subject_data[sub] = (X, np.array(raw_labels))
    print(f'    {sub}: {X.shape}, {Counter(raw_labels)}', flush=True)

le = LabelEncoder().fit(GESTURE_LABELS)
n_classes = len(le.classes_)
sequence_length = RESAMPLE_TO_N_STEPS
n_channels = len(SENSOR_COLS)
print(f'  X shape per subject: ({sequence_length}, {n_channels}), classes={n_classes}', flush=True)

# ── Augmentations ────────────────────────────────────────────────────────────
def aug_amp(t, sr, rng):
    s = rng.uniform(sr[0], sr[1], size=(1, t.shape[1]))
    return (t * s).astype(np.float32)
def aug_off(t, ratio, rng):
    sd = np.maximum(np.std(t, axis=0, keepdims=True) * ratio, 1e-6)
    return (t + rng.normal(0, sd, size=(1, t.shape[1]))).astype(np.float32)

def augment(X, y, copies, seed):
    if copies <= 0: return X, y
    rng = np.random.default_rng(seed)
    Xa, ya = [], []
    for trial, lbl in zip(X, y):
        for _ in range(copies):
            t = trial.astype(np.float32, copy=True)
            cfg = AUGMENT_CONFIG['amplitude_scale']
            if cfg['enabled'] and rng.random() < cfg['apply_prob']:
                t = aug_amp(t, cfg['scale_range'], rng)
            cfg = AUGMENT_CONFIG['baseline_offset']
            if cfg['enabled'] and rng.random() < cfg['apply_prob']:
                t = aug_off(t, cfg['offset_std_ratio'], rng)
            Xa.append(np.nan_to_num(t, nan=0.0)); ya.append(lbl)
    return np.concatenate([X, np.stack(Xa, axis=0)], axis=0), np.concatenate([y, np.array(ya, dtype=y.dtype)], axis=0)

def make_scaler(kind):
    return MinMaxScaler() if kind == 'minmax' else StandardScaler() if kind == 'standard' else None

def scale_sets(X_tr, X_va, X_te, kind):
    sc = make_scaler(kind)
    if sc is None: return X_tr, X_va, X_te, None
    Nt, T, C = X_tr.shape
    Xt = sc.fit_transform(X_tr.reshape(Nt, T*C)).reshape(Nt, T, C).astype(np.float32)
    Xv = sc.transform(X_va.reshape(X_va.shape[0], T*C)).reshape(X_va.shape[0], T, C).astype(np.float32) if len(X_va) else X_va
    Xs = sc.transform(X_te.reshape(X_te.shape[0], T*C)).reshape(X_te.shape[0], T, C).astype(np.float32) if len(X_te) else X_te
    return Xt, Xv, Xs, sc

# ── Variants ─────────────────────────────────────────────────────────────────
def build_baseline(s, c, n):
    m = Sequential([Input(shape=(s, c)),
        Conv1D(32, 4, activation='relu'), MaxPooling1D(2),
        Conv1D(64, 4, activation='relu'), MaxPooling1D(2),
        Flatten(), Dense(64, activation='relu'), Dropout(0.3),
        Dense(n, activation='softmax')], name='Baseline')
    m.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy']); return m

def build_shallow(s, c, n):
    m = Sequential([Input(shape=(s, c)),
        Conv1D(32, 5, activation='relu'), MaxPooling1D(2),
        Flatten(), Dense(32, activation='relu'), Dropout(0.3),
        Dense(n, activation='softmax')], name='Shallow')
    m.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy']); return m

def build_deep(s, c, n):
    m = Sequential([Input(shape=(s, c)),
        Conv1D(32, 4, activation='relu'), MaxPooling1D(2),
        Conv1D(64, 4, activation='relu'), MaxPooling1D(2),
        Conv1D(128, 3, activation='relu'), MaxPooling1D(2),
        Flatten(), Dense(128, activation='relu'), Dropout(0.4),
        Dense(n, activation='softmax')], name='Deep')
    m.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy']); return m

def build_bn_gap(s, c, n):
    m = Sequential([Input(shape=(s, c)),
        Conv1D(32, 4, padding='same'), BatchNormalization(), Activation('relu'), MaxPooling1D(2),
        Conv1D(64, 4, padding='same'), BatchNormalization(), Activation('relu'), MaxPooling1D(2),
        Conv1D(128, 3, padding='same'), BatchNormalization(), Activation('relu'),
        GlobalAveragePooling1D(), Dropout(0.3),
        Dense(n, activation='softmax')], name='BN_GAP')
    m.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy']); return m

def build_wide(s, c, n):
    m = Sequential([Input(shape=(s, c)),
        Conv1D(32, 8, activation='relu'), MaxPooling1D(2),
        Conv1D(64, 8, activation='relu'), MaxPooling1D(2),
        Flatten(), Dense(64, activation='relu'), Dropout(0.3),
        Dense(n, activation='softmax')], name='WideKernel')
    m.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy']); return m

ALL_BUILDERS = {'Baseline': build_baseline, 'Shallow': build_shallow,
                'Deep': build_deep, 'BN_GAP': build_bn_gap, 'WideKernel': build_wide}
WANT = [v.strip() for v in args.variants.split(',') if v.strip()]
VARIANTS = [(n, ALL_BUILDERS[n]) for n in WANT if n in ALL_BUILDERS]

# ── LOSO loop ────────────────────────────────────────────────────────────────
loso_results = []  # list of dicts, one per held-out subject
total_t0 = time.time()

# Load state if present
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
for i_fold, held_out in enumerate(subjects_to_run, start=args.fold_start + 1):
    print(f'\n  ── LOSO fold {i_fold}/{len(SUBJECTS)}: held-out={held_out} ──', flush=True)
    fold_t0 = time.time()
    train_subjects = [s for s in SUBJECTS if s != held_out]
    X_tr_all = np.concatenate([subject_data[s][0] for s in train_subjects], axis=0)
    y_tr_all_str = np.concatenate([subject_data[s][1] for s in train_subjects], axis=0)
    y_tr_all = le.transform(y_tr_all_str)
    X_te = subject_data[held_out][0]
    y_te = le.transform(subject_data[held_out][1])
    print(f'    train: {X_tr_all.shape[0]} trials from {len(train_subjects)} subjects, '
          f'test: {X_te.shape[0]} trials', flush=True)

    fold_variant_records = []
    for v_name, v_builder in VARIANTS:
        # Inner CV
        cv_accs = []
        if args.cv_folds > 1:
            skf = StratifiedKFold(n_splits=args.cv_folds, shuffle=True, random_state=RANDOM_STATE)
            for inner_idx, (tr_idx, va_idx) in enumerate(skf.split(X_tr_all, y_tr_all), start=1):
                X_tr, y_tr = X_tr_all[tr_idx].copy(), y_tr_all[tr_idx].copy()
                X_va, y_va = X_tr_all[va_idx].copy(), y_tr_all[va_idx].copy()
                X_tr, y_tr = augment(X_tr, y_tr, AUGMENT_COPIES, RANDOM_STATE + inner_idx)
                X_tr_s, X_va_s, _, _ = scale_sets(X_tr, X_va, X_te, NORMALISATION)
                tf.keras.backend.clear_session()
                tf.random.set_seed(RANDOM_STATE + inner_idx)
                m = v_builder(sequence_length, n_channels, n_classes)
                m.fit(X_tr_s, y_tr, epochs=args.epochs, batch_size=TRAIN_BATCH_SIZE,
                      validation_data=(X_va_s, y_va), verbose=0)
                pred = np.argmax(m.predict(X_va_s, verbose=0), axis=1)
                cv_accs.append(float(accuracy_score(y_va, pred)))
        cv_mean = float(np.mean(cv_accs)) if cv_accs else None
        cv_std  = float(np.std(cv_accs))  if cv_accs else None

        # Final retrain on full pool, evaluate on held-out subject
        X_tr_full, y_tr_full = augment(X_tr_all, y_tr_all, AUGMENT_COPIES, RANDOM_STATE)
        X_tr_s, _, X_te_s, scaler = scale_sets(X_tr_full, X_tr_full[:0], X_te, NORMALISATION)
        tf.keras.backend.clear_session()
        tf.random.set_seed(RANDOM_STATE)
        m = v_builder(sequence_length, n_channels, n_classes)
        n_params = int(m.count_params())
        m.fit(X_tr_s, y_tr_full, epochs=args.epochs, batch_size=TRAIN_BATCH_SIZE,
              validation_split=TRAIN_VALIDATION_SPLIT, verbose=0)
        test_loss, test_acc = m.evaluate(X_te_s, y_te, verbose=0)
        y_pred = np.argmax(m.predict(X_te_s, verbose=0), axis=1)
        rep = classification_report(y_te, y_pred, target_names=le.classes_,
                                    output_dict=True, zero_division=0)

        # per-class held-out accuracy
        per_class = {}
        for c_idx, c_name in enumerate(le.classes_):
            mask = (y_te == c_idx)
            n = int(mask.sum())
            if n > 0:
                per_class[c_name] = {'correct': int((y_pred[mask] == c_idx).sum()),
                                     'total': n,
                                     'accuracy': float((y_pred[mask] == c_idx).mean())}
            else:
                per_class[c_name] = {'correct': 0, 'total': 0, 'accuracy': None}

        fold_variant_records.append({
            'name': v_name, 'n_params': n_params,
            'cv_mean': cv_mean, 'cv_std': cv_std, 'cv_accs': cv_accs,
            'test_acc': float(test_acc), 'test_loss': float(test_loss),
            'classification_report': rep, 'per_class': per_class,
        })
        print(f'    {v_name:<11} cv={cv_mean:.3f}\u00b1{cv_std:.3f}  test={test_acc:.3f} '
              f'params={n_params:,}', flush=True)

    loso_results.append({
        'held_out': held_out,
        'n_train_trials': int(X_tr_all.shape[0]),
        'n_test_trials':  int(X_te.shape[0]),
        'variants': fold_variant_records,
    })
    print(f'    fold time: {(time.time()-fold_t0)/60:.1f} min', flush=True)

    # Persist state after every fold so we can resume
    if state_path:
        with open(state_path, 'w') as fh:
            json.dump({
                'exp_id': args.exp_id, 'exp_label': args.exp_label,
                'config': vars(args), 'subjects': SUBJECTS,
                'loso_results': [{'held_out': f['held_out'],
                                  'n_train_trials': f['n_train_trials'],
                                  'n_test_trials':  f['n_test_trials'],
                                  'variants': f['variants']} for f in loso_results],
            }, fh, indent=2, default=str)
        print(f'    state saved: {len(loso_results)} folds total', flush=True)

print(f'\nTotal LOSO time: {(time.time()-total_t0)/60:.1f} min', flush=True)

# Merge with prior partial results if requested
if args.resume_from and os.path.exists(args.resume_from):
    print(f'  merging with {args.resume_from}', flush=True)
    with open(args.resume_from) as fh:
        prior = json.load(fh)
    prior_results = prior.get('loso_results', [])
    seen = {r['held_out'] for r in loso_results}
    for r in prior_results:
        if r['held_out'] not in seen:
            loso_results.append(r)
    # Reorder to match SUBJECTS
    loso_results.sort(key=lambda r: SUBJECTS.index(r['held_out']) if r['held_out'] in SUBJECTS else 999)

if args.skip_pdf:
    ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    out_json = OUT_DIR / f'loso_{args.exp_id}_partial_{ts}.json'
    with open(out_json, 'w') as fh:
        json.dump({
            'exp_id': args.exp_id, 'exp_label': args.exp_label,
            'config': vars(args), 'subjects': SUBJECTS,
            'loso_results': [{'held_out': f['held_out'],
                              'n_train_trials': f['n_train_trials'],
                              'n_test_trials':  f['n_test_trials'],
                              'variants': f['variants']} for f in loso_results],
        }, fh, indent=2, default=str)
    print(f'PARTIAL JSON: {out_json}', flush=True)
    print('DONE_PARTIAL', flush=True)
    raise SystemExit(0)

# ── Aggregate across folds ───────────────────────────────────────────────────
variant_summary = {}
for v_name, _ in VARIANTS:
    test_accs = [f['variants'][[v['name'] for v in f['variants']].index(v_name)]['test_acc']
                 for f in loso_results]
    cv_means = [f['variants'][[v['name'] for v in f['variants']].index(v_name)]['cv_mean']
                for f in loso_results]
    cv_means = [c for c in cv_means if c is not None]
    variant_summary[v_name] = {
        'test_mean': float(np.mean(test_accs)),
        'test_std':  float(np.std(test_accs)),
        'test_min':  float(np.min(test_accs)),
        'test_max':  float(np.max(test_accs)),
        'cv_mean_of_means':  float(np.mean(cv_means)) if cv_means else None,
        'cv_std_across_folds': float(np.std(cv_means)) if cv_means else None,
        'fold_test_accs': test_accs,
        'n_params': loso_results[0]['variants'][[v['name']
                    for v in loso_results[0]['variants']].index(v_name)]['n_params'],
    }

# ── Plots ────────────────────────────────────────────────────────────────────
ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

# 1. Per-subject test accuracy heatmap (variants × subjects)
fig, ax = plt.subplots(figsize=(max(8, len(SUBJECTS)*0.7), 4))
mat = np.array([[f['variants'][[v['name'] for v in f['variants']].index(v_name)]['test_acc']
                 for f in loso_results] for v_name, _ in VARIANTS])
im = ax.imshow(mat, cmap='RdYlGn', vmin=0.0, vmax=1.0, aspect='auto')
ax.set_xticks(np.arange(len(SUBJECTS))); ax.set_xticklabels(SUBJECTS, rotation=45, ha='right', fontsize=8)
ax.set_yticks(np.arange(len(VARIANTS))); ax.set_yticklabels([v[0] for v in VARIANTS])
for i in range(mat.shape[0]):
    for j in range(mat.shape[1]):
        ax.text(j, i, f'{mat[i,j]:.2f}', ha='center', va='center', fontsize=7,
                color='black' if mat[i,j] > 0.5 else 'white')
plt.colorbar(im, ax=ax, label='Held-out test accuracy', shrink=0.8)
ax.set_title(f'{args.exp_id}: per-subject LOSO accuracy by variant')
plt.tight_layout()
heatmap_path = OUT_DIR / f'{args.exp_id}_heatmap_{ts}.png'
plt.savefig(heatmap_path, dpi=140); plt.close()

# 2. Aggregate bar chart
fig, ax = plt.subplots(figsize=(8, 4.5))
names = [v[0] for v in VARIANTS]
means = [variant_summary[n]['test_mean'] for n in names]
stds  = [variant_summary[n]['test_std'] for n in names]
ax.bar(np.arange(len(names)), means, yerr=stds, capsize=4, color='#4c78a8')
ax.set_xticks(np.arange(len(names))); ax.set_xticklabels(names)
ax.set_ylim(0, 1.05); ax.set_ylabel('LOSO held-out accuracy (mean ± std)')
ax.set_title(f'{args.exp_id}: LOSO accuracy by variant (n={len(SUBJECTS)} folds)')
ax.grid(axis='y', alpha=0.3)
for i, v in enumerate(means):
    ax.text(i, v + max(stds[i], 0.01) + 0.02, f'{v:.3f}', ha='center', fontsize=8)
plt.tight_layout()
bar_path = OUT_DIR / f'{args.exp_id}_bar_{ts}.png'
plt.savefig(bar_path, dpi=140); plt.close()

# 3. Per-variant box plot of fold accuracies
fig, ax = plt.subplots(figsize=(8, 4.5))
data = [variant_summary[n]['fold_test_accs'] for n in names]
ax.boxplot(data, labels=names)
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
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, PageBreak
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

pdf_path = OUT_DIR / f'loso_{args.exp_id}_{ts}.pdf'
doc = SimpleDocTemplate(str(pdf_path), pagesize=A4,
                        leftMargin=12*mm, rightMargin=12*mm,
                        topMargin=10*mm, bottomMargin=10*mm,
                        title=f'LOSO {args.exp_id}')
story = []

# Header
story.append(Paragraph(f'LOSO Experiment {args.exp_id}', styles['Title']))
story.append(Paragraph(args.exp_label or '', styles['Small']))
story.append(Paragraph(f"Generated {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}", styles['Small']))
story.append(Spacer(1, 4))

# Config
story.append(Paragraph('<b>Experiment configuration</b>', styles['Section']))
sensor_modalities = ', '.join(filter(None, [
    'flex' if args.use_flex else None,
    'YPR'  if args.use_ypr else None,
    'accel' if args.use_accel else None,
]))
hands_used = ', '.join(filter(None, [
    'left'  if args.use_left else None,
    'right' if args.use_right else None,
]))
cfg_rows = [
    ('Class set', f'{len(GESTURE_LABELS)} classes ' + ('including' if args.include_nothing else 'excluding') + ' Double_Nothing'),
    ('Classes', ', '.join(GESTURE_LABELS)),
    ('Sensor modalities', sensor_modalities),
    ('Hands', hands_used),
    ('Sensor channels', str(n_channels)),
    ('Sequence length', f'{sequence_length} steps (resampled)'),
    ('Filter', f'Butterworth {BUTTERWORTH_CUTOFF}Hz order {BUTTERWORTH_ORDER}'),
    ('Normalisation', NORMALISATION),
    ('Augmentation', f'amplitude_scale + baseline_offset, {AUGMENT_COPIES} copies'),
    ('Inner CV folds', str(args.cv_folds)),
    ('Epochs · batch size', f'{args.epochs} · {TRAIN_BATCH_SIZE}'),
    ('LOSO subjects', f'{len(SUBJECTS)}: {", ".join(SUBJECTS)}'),
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
hdr = ['Variant', 'Params', 'LOSO mean', 'LOSO std', 'Min fold', 'Max fold',
       'Inner CV mean (avg across folds)']
rows = [hdr]
best = max(VARIANTS, key=lambda v: variant_summary[v[0]]['test_mean'])[0]
for v_name, _ in VARIANTS:
    s = variant_summary[v_name]
    rows.append([v_name, f'{s["n_params"]:,}',
                 _fmt(s['test_mean'], 4), _fmt(s['test_std'], 4),
                 _fmt(s['test_min'], 4), _fmt(s['test_max'], 4),
                 _fmt(s['cv_mean_of_means'], 4)])
t = Table(rows, hAlign='LEFT', colWidths=(22*mm, 22*mm, 22*mm, 18*mm, 18*mm, 18*mm, 38*mm))
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
story.append(Spacer(1, 3))
story.append(Paragraph(f'Best variant by LOSO mean: <b>{best}</b>', styles['Small']))

# Bar + box plots
img_w = 175*mm
story.append(Image(str(bar_path), width=img_w, height=img_w*0.5))
story.append(Image(str(box_path), width=img_w, height=img_w*0.5))

# Heatmap
story.append(Paragraph('<b>Per-subject LOSO accuracy heatmap</b>', styles['Section']))
story.append(Image(str(heatmap_path), width=img_w, height=img_w*0.42))

# Per-fold table
story.append(PageBreak())
story.append(Paragraph('<b>Per-fold detail (held-out subject = each row)</b>', styles['Section']))
hdr = ['Held-out subj', 'Train trials'] + [v[0] for v in VARIANTS]
rows = [hdr]
for f in loso_results:
    row = [f['held_out'], f['n_train_trials']]
    for v_name, _ in VARIANTS:
        rec = f['variants'][[v['name'] for v in f['variants']].index(v_name)]
        row.append(_fmt(rec['test_acc'], 3))
    rows.append(row)
t = Table(rows, hAlign='LEFT')
t.setStyle(TableStyle([
    ('FONT',(0,0),(-1,-1),'Helvetica',7.5),
    ('FONT',(0,0),(-1,0),'Helvetica-Bold',7.5),
    ('BACKGROUND',(0,0),(-1,0),colors.HexColor('#eef2f7')),
    ('GRID',(0,0),(-1,-1),0.25,colors.HexColor('#cccccc')),
    ('VALIGN',(0,0),(-1,-1),'MIDDLE'),
    ('ALIGN',(1,1),(-1,-1),'CENTER'),
    ('BOTTOMPADDING',(0,0),(-1,-1),1), ('TOPPADDING',(0,0),(-1,-1),1),
]))
story.append(t)

# Per-variant detail pages
for v_name, _ in VARIANTS:
    story.append(PageBreak())
    s = variant_summary[v_name]
    story.append(Paragraph(f'Variant: {v_name}', styles['Title']))
    story.append(Paragraph(f'Params: {s["n_params"]:,}', styles['Small']))
    story.append(Paragraph(
        f'LOSO mean accuracy: {s["test_mean"]:.4f} ± {s["test_std"]:.4f} '
        f'(min {s["test_min"]:.4f}, max {s["test_max"]:.4f})', styles['Small']))
    story.append(Spacer(1, 4))

    # Per-fold per-class accuracy table
    story.append(Paragraph('<b>Per-fold per-class held-out accuracy</b>', styles['SubSection']))
    cls_hdr = ['Held-out'] + GESTURE_LABELS + ['Overall']
    rows = [cls_hdr]
    for f in loso_results:
        rec = f['variants'][[v['name'] for v in f['variants']].index(v_name)]
        row = [f['held_out']]
        for c in GESTURE_LABELS:
            pc = rec['per_class'].get(c)
            if pc is None or pc['total'] == 0:
                row.append('\u2014')
            else:
                row.append(f'{pc["correct"]}/{pc["total"]}')
        row.append(_fmt(rec['test_acc'], 3))
        rows.append(row)
    # aggregate across folds
    agg_row = ['MEAN']
    for c in GESTURE_LABELS:
        accs = []
        for f in loso_results:
            rec = f['variants'][[v['name'] for v in f['variants']].index(v_name)]
            pc = rec['per_class'].get(c)
            if pc and pc['total'] > 0:
                accs.append(pc['accuracy'])
        agg_row.append(_fmt(np.mean(accs), 3) if accs else '\u2014')
    agg_row.append(_fmt(s['test_mean'], 3))
    rows.append(agg_row)

    cell_style = ParagraphStyle('cell', fontName='Helvetica', fontSize=6.5, leading=8)
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

# Also save raw results json for downstream re-use
out_json = OUT_DIR / f'loso_{args.exp_id}_{ts}.json'
with open(out_json, 'w') as fh:
    json.dump({
        'exp_id': args.exp_id, 'exp_label': args.exp_label,
        'config': vars(args), 'subjects': SUBJECTS,
        'variant_summary': variant_summary,
        'loso_results': [{'held_out': f['held_out'],
                          'n_train_trials': f['n_train_trials'],
                          'n_test_trials':  f['n_test_trials'],
                          'variants': f['variants']} for f in loso_results],
    }, fh, indent=2, default=str)
print(f'JSON: {out_json}', flush=True)
print('DONE', flush=True)
