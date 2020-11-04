# Author: Alex Bagnoud

# ---------------------------------------------------------------------------
# This script fits a sphere around a mesh.
# It outputs:
# - Center of the sphere
# - Blender code to create a sphere (if pasted in Blender python console)

# HOW TO USE:
# - Place the script in the same folder as the .stl files
# - in a command terminal, run: python fit_cylinder.py name_body.stl
# - Output is printed in the terminal
#
# ---------------------------------------------------------------------------

import sys
import trimesh
import numpy as np

def sphere(mesh):
    # fitting a sphere
    Min_Sphere = trimesh.nsphere.minimum_nsphere(mesh)

    # Saving output
    S_location = np.round(Min_Sphere[0],6)
    S_radius = Min_Sphere[1]

    # Printing sphere center
    print("")
    print("Center of the sphere: [{C1} {C2} {C3}]".format(
    C1=S_location[0], C2=S_location[1], C3=S_location[2]))

    # Printing output for blender python API
    #bpy.ops.mesh.primitive_uv_sphere_add(size=, location())
    print("-------")
    print("import bpy")
    print("bpy.ops.mesh.primitive_uv_sphere_add(radius=",S_radius,
          ", enter_editmode=False, location=(",S_location[0],",",
          S_location[1],",",S_location[2],"))")
    print("-------")



if __name__ == "__main__":
    mesh = trimesh.load(sys.argv[1])
    sphere(mesh)
