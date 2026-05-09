# `ML/loso_results/`

Output folder for the LOSO (Leave-One-Subject-Out) experiment grid driven by
[`../loso_runner.py`](../loso_runner.py) and
[`../run_all_experiments.sh`](../run_all_experiments.sh).

Each experiment writes the following files here:

| Pattern                                  | What it is |
|------------------------------------------|------------|
| `loso_<exp_id>_<timestamp>.pdf`          | Final report — config, summary, heatmap, per-variant detail pages |
| `loso_<exp_id>_<timestamp>.json`         | Raw per-fold per-variant results (human-readable JSON) |
| `<exp_id>_state.json`                    | Per-fold state file used for resuming interrupted runs |
| `<exp_id>_heatmap_<timestamp>.png`       | Per-subject × variant accuracy heatmap (also embedded in PDF) |
| `<exp_id>_bar_<timestamp>.png`           | LOSO mean ± std bar chart by variant |
| `<exp_id>_box_<timestamp>.png`           | Box plot of per-fold accuracies |

## Running the grid

From the `ML/` directory:

```bash
./run_all_experiments.sh
```

Or a single experiment:

```bash
python3 loso_runner.py --exp_id A1 \
    --exp_label "Full sensors, including Nothing class" \
    --use_flex --use_ypr --use_accel --use_left --use_right --include_nothing \
    --epochs 8 --cv_folds 3 \
    --state_file loso_results/A1_state.json
```

The runner is **resumable**: re-invoking the same command after an interruption
will skip folds that were already completed (it reads the state file to know
which subjects are done).

## Experiment IDs

| ID            | Description                                                           |
|---------------|-----------------------------------------------------------------------|
| `A1`          | Full sensors (flex+YPR+accel, both hands), **including** Double_Nothing |
| `A2`          | Full sensors (flex+YPR+accel, both hands), excluding Double_Nothing   |
| `B_flex`      | Flex only, both hands, no Nothing                                     |
| `B_ypr`       | YPR only, both hands, no Nothing                                      |
| `B_accel`     | Accelerometers only, both hands, no Nothing                           |
| `B_flex_ypr`  | Flex + YPR, both hands, no Nothing                                    |
| `B_flex_accel`| Flex + accel, both hands, no Nothing                                  |
| `B_ypr_accel` | YPR + accel, both hands, no Nothing                                   |
| `C_left`      | Full sensors, **left hand only**, no Nothing                          |
| `C_right`     | Full sensors, **right hand only**, no Nothing                         |

(Note: full-sensors-no-Nothing-both-hands == `A2`, so it isn't repeated under group B.)
