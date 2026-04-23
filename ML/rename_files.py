"""
rename_files.py
===============
Batch rename CSV files in a directory by replacing a substring in the filename.

Edit the CONFIG block, then run:
    python rename_files.py
"""

import os
import glob

# =============================================================================
# CONFIG
# =============================================================================

# Folder containing the CSV files to rename.
DIRECTORY = '/home/jestin/ThesisRepo/ML/TwoHand_L_Point_R_'

# The substring to find and replace in each filename.
FIND    = 'L_fist_R_fist'
REPLACE = 'L_flat_R_flat'

# Set to True to preview changes without actually renaming anything.
DRY_RUN = False

# =============================================================================

def run():
    directory = os.path.expanduser(DIRECTORY)

    if not os.path.isdir(directory):
        print(f'ERROR: Directory not found: {directory}')
        return

    csv_files = sorted(glob.glob(os.path.join(directory, '*.csv')))
    if not csv_files:
        print(f'No CSV files found in: {directory}')
        return

    matches = [f for f in csv_files if FIND in os.path.basename(f)]
    if not matches:
        print(f'No filenames contain "{FIND}".')
        return

    print(f'{"DRY RUN — " if DRY_RUN else ""}Renaming {len(matches)} file(s) in: {directory}')
    print(f'  {FIND!r}  →  {REPLACE!r}')
    print()

    for old_path in matches:
        old_name = os.path.basename(old_path)
        new_name = old_name.replace(FIND, REPLACE)
        new_path = os.path.join(directory, new_name)

        print(f'  {old_name}')
        print(f'  → {new_name}')

        if not DRY_RUN:
            if os.path.exists(new_path):
                print(f'  SKIP — target already exists')
            else:
                os.rename(old_path, new_path)
        print()

    if DRY_RUN:
        print('Dry run complete. Set DRY_RUN = False to apply changes.')
    else:
        print('Done.')

if __name__ == '__main__':
    run()
