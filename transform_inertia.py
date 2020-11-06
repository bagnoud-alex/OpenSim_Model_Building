# Author: Alex Bagnoud

# ---------------------------------------------------------------------------
# This script returns the transformation matrix to rotate a body along its
# principal axes of inertia. (It also places the body in it's center of mass,
# but it can be moved again on Blender to its original origin point)

# HOW TO USE:
# - place the script in the same folder as the .stl files
# - in a command terminal, run: python transform_inertia.py name_body.stl
# - copy the output between the '---' and paste in the python console in blender
# - (make sure the correct body is highlighted in blender)
# ---------------------------------------------------------------------------

import sys
import trimesh

def transform(mesh):

    print("-----")
    # Preliminary check, mesh must be watertight
    if not mesh.is_watertight:
        print("!!! Mesh is not watertight, ignore all results below !!!")

    T = mesh.principal_inertia_transform
    print("Transform = ((",T[0,0],",",T[1,0],",",T[2,0],",",T[3,0],"),",
    "(",T[0,1],",",T[1,1],",",T[2,1],",",T[3,1],"),",
    "(",T[0,2],",",T[1,2],",",T[2,2],",",T[3,2],"),",
    "(",T[0,3],",",T[1,3],",",T[2,3],",",T[3,3],"))")
    print("import bpy")
    print("bpy.context.object.matrix_world = Transform")
    print("-----")

if __name__ == "__main__":
    mesh = trimesh.load(sys.argv[1])
    transform(mesh)
