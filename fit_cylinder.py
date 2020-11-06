# Author: Alex Bagnoud

# ---------------------------------------------------------------------------
# This script fits a cylinder around a mesh.
# It outputs:
# - Center of cylinder
# - Blender code to create a cylinder (if pasted in Blender python console)

# HOW TO USE:
# - Place the script in the same folder as the .stl files
# - in a command terminal, run: python fit_cylinder.py name_body.stl
# - Output is printed in the terminal
#
# ---------------------------------------------------------------------------

import sys
import trimesh
import numpy as np

def cylinder(mesh):
    # fitting a sphere
    Min_Cyl = trimesh.bounds.minimum_cylinder(mesh)

    # Saving output
    Radius = Min_Cyl["radius"]
    Height = Min_Cyl["height"]
    R = Min_Cyl["transform"]
    Center = R[0:3,3]

    # Printing cylinder center
    print("")
    print("Center of the cylinder: [{C1} {C2} {C3}]".format(
    C1=Center[0], C2=Cente[1], C3=Center[2]))

    # Printing output for blender python API
    print("-------")
    print("import bpy")
    print("bpy.ops.mesh.primitive_cylinder_add(radius=", Radius, ", depth=", Height,
          ", enter_editmode=False, location=(", Center[0],",",Center[1],", ",Center[2],"))")
    print("Transform = ((",R[0,0],",",R[1,0],",",R[2,0],",",R[3,0],"),",
        "(",R[0,1],",",R[1,1],",",R[2,1],",",R[3,1],"),",
        "(",R[0,2],",",R[1,2],",",R[2,2],",",R[3,2],"),",
        "(",R[0,3],",",R[1,3],",",R[2,3],",",R[3,3],"))")
    print("bpy.context.object.matrix_world = Transform")
    print("-------")


if __name__ == "__main__":
    mesh = trimesh.load(sys.argv[1])
    cylinder(mesh)
