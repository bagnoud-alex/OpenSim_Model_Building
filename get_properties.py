# Author: Alex Bagnoud

# ---------------------------------------------------------------------------
# This script outputs the properties of a give .stl file for a body:
# - Mass [kg] (given a density)
# - Center of mass location in function of origin
# - Inertia matrix (Computed at the center of mass)

# HOW TO USE:
# - place the script in the same folder as the .stl file
# - in a command terminal, run: python get_properties.py name_body.stl
# - Output is printed in the terminal

# Note: density is computed with no units. The volume will be the same as the
# units defined in your mesh. If the density is given in kg/m^3, the mass will
# be in kg.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Define the density here
density = 1132 # [kg/m^3]
# ---------------------------------------------------------------------------

import sys
import trimesh
import numpy as np

def inertia(mesh):

    print("-----")
    # Preliminary check, mesh must be watertight
    if not mesh.is_watertight:
        print("!!! Mesh is not watertight, ignore all results below !!!")

    print("density is: ", density, "kg/m^3")
    mesh.density = density
    # print("Volume: ", mesh.volume)

    # Mass
    print("<mass> {m} </mass>".format(m=np.round(mesh.mass, 4)))

    # Center of mass
    CM = np.round(mesh.center_mass, 6)
    print("<mass_center> {CM1} {CM2} {CM3} </mass_center>".format(CM1=CM[0], CM2=CM[1], CM3=CM[2]))

    # Inertia tensor
    I = mesh.moment_inertia
    print("<inertia_xx> {i} </inertia_xx>".format(i=np.format_float_scientific(I[0,0], precision=4)))
    print("<inertia_yy> {i} </inertia_yy>".format(i=np.format_float_scientific(I[1,1], precision=4)))
    print("<inertia_zz> {i} </inertia_zz>".format(i=np.format_float_scientific(I[2,2], precision=4)))
    print("<inertia_xy> {i} </inertia_xy>".format(i=np.format_float_scientific(I[0,1], precision=4)))
    print("<inertia_xz> {i} </inertia_xz>".format(i=np.format_float_scientific(I[0,2], precision=4)))
    print("<inertia_yz> {i} </inertia_yz>".format(i=np.format_float_scientific(I[1,2], precision=4)))
    print(I)
    print("-----")

if __name__ == "__main__":
    mesh = trimesh.load(sys.argv[1])
    inertia(mesh)
