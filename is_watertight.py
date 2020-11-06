# Author: Alex Bagnoud

# ---------------------------------------------------------------------------
# This script checks if all the .stl files are watertight before computing
# mass/inertia/center of mass.
# All .stl files must be watertight for correct computations. The scripts outputs
# which files are not watertight. If all files are ok, it outputs a confirmation.

# HOW TO USE:
# - place the script in the same folder as all the .stl files to verify
# - in a command terminal, run: python is_watertight.py
# - Results are printed on the terminal
# ---------------------------------------------------------------------------

import trimesh
import os

# Loading all files in the folder
files = os.listdir()

# For final print
all_ok = True
nb_check = 0

# Checking all files
for i in range(len(files)):
    if files[i].endswith('.stl'): # Looking only for .stl files
        nb_check += 1
        mesh = trimesh.load(files[i]) # Loading mesh
        if not mesh.is_watertight: # Checking if watertight
            print(files[i], " - NOT WATERTIGHT!") # Error print if not watertight
            all_ok = False

if all_ok: # If all files are ok,
    print("All files are watertight. Number of meshes verified:", nb_check)
