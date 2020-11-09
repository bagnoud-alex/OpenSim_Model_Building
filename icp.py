# Author: Alex Bagnoud

# ---------------------------------------------------------------------------
# This script aligns two meshes using an ICP (Iterative closest point) method
# From the trimesh library. It also provides information for OpenSim

# It is used to align a processed body to a parent body in a default position.

# It first transforms the two meshes into their principle axes of inertia
# and only then performs icp for a more consistent result.
# The three transformations are then concatenated into one at the end.

# The output is specialized for the blender console api

# HOW TO USE:
# - call: python icp.py source_mesh.icp target_mesh.icp
# - The output gives: - Root mean square error after transformation
#                     - <Location_in_parent> and <Orientation_in_parent>
#                     - Blender transform to copy and paste in blender python
#                       console while having the processed body selected
# ---------------------------------------------------------------------------


import sys
import trimesh
import numpy as np
import math
from sklearn.neighbors import NearestNeighbors

# ==============================================================
def icp(mesh_A, mesh_B):

    # 1. Calculate their transformation into principal axes
    Inert_A = np.array(mesh_A.principal_inertia_transform)
    Inert_B = np.array(mesh_B.principal_inertia_transform)

    # 2. Create inverse of the transformation Inert_B
    Inert_B_INV = np.linalg.inv(Inert_B)

    # 3. Copy meshes
    Temp_A = mesh_A.copy()
    Temp_B = mesh_B.copy()
    RMSE_A = mesh_A.copy() # used at point 7.

    # 4. Apply transformation to copies
    Temp_A.apply_transform(Inert_A)
    Temp_B.apply_transform(Inert_B)

    # 5. Find ICP
    Translate = find_best_icp(Temp_A, Temp_B)

    # 6. Rebuild Whole Transformation
    TM = np.matmul(Inert_B_INV, Translate)
    TM = np.matmul(TM, Inert_A)

    # 7. Compute RMSE
    RMSE_A.apply_transform(TM)
    RMSE = compute_RMSE(RMSE_A, mesh_B)
    print("")
    print("RMSE = {e}".format(e=np.format_float_scientific(RMSE, precision=3)))

    # 8. Print output
    print_output(TM)


# ==============================================================
def compute_RMSE(A, B):
    # This function computes RMSE between two meshes
    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(B.vertices)
    distances, _ = neigh.kneighbors(A.vertices, return_distance=True)
    return np.sqrt(np.mean(np.square(distances)))


# ==============================================================
def find_best_icp(A, B):
    # Trimesh has two methods for icp. Both are executed and the best one is kept

    # Method 1
    M_1 = A.register(other=B, icp_first=10000,icp_final=100000)
    # Method 2
    M_2 = trimesh.registration.mesh_other(A, B)

    # Store Error and Transformation, find lowest error
    err = np.array([M_1[1], M_2[1]])
    T = [M_1[0], M_2[0]]
    best_id = np.where(err == np.min(err))[0][0]

    return T[best_id]


# ==============================================================
def print_output(T):

    print("")
    print("==== OPENSIM ====")
    Get_location_and_orientation_in_parent(T)
    print("")
    print("==== BLENDER ====")
    print("Copy between the '---' and Paste in Blender Console:")
    print("-----------")
    print("import bpy")
    print("Transform = ((",T[0,0],",",T[1,0],",",T[2,0],",",T[3,0],"),",
    "(",T[0,1],",",T[1,1],",",T[2,1],",",T[3,1],"),",
    "(",T[0,2],",",T[1,2],",",T[2,2],",",T[3,2],"),",
    "(",T[0,3],",",T[1,3],",",T[2,3],",",T[3,3],"))")
    print("bpy.context.object.matrix_world = Transform")
    print("-----------")
    print("")


# ==============================================================
def Rotation_Matrix_To_Euler_Angles(R):
    # Computes three Euler angles XYZ given a matrix.
    # OpenSim needs those three angles in this order for <orientation_in_parent>
    # Method according to: Slabaugh, Gregory G. « Computing Euler Angles from a Rotation Matrix »

    if abs(abs(R[2,0])-1) < 1e-6:
        Rz = 0
        if R[2,0]+1 < 1e-6:
            Ry = math.pi/2
            Rx = math.atan2(R[0,1],R[0,2])
        else:
            Ry = -math.pi/2
            Rx = math.atan2(-R[0,1],-R[0,2])
    else:
        Ry = -math.asin(R[2,0])
        Rx = math.atan2(R[2,1]/math.cos(Ry), R[2,2]/math.cos(Ry))
        Rz = math.atan2(R[1,0]/math.cos(Ry), R[0,0]/math.cos(Ry))
    return np.array([Rx, Ry, Rz])


# ==============================================================
def Get_location_and_orientation_in_parent(Transformation_Matrix):
    # This function outputs the shift needed to print <location_in_parent>
    # and calls Rotation_Matrix_To_Euler_Angles(R) for <orientation_in_parent>

    # Location_in_parent
    Loc = np.round(Transformation_Matrix[0:3,3], 6)
    print("<location_in_parent> {L1} {L2} {L3}".format(
    L1=Loc[0], L2=Loc[1], L3=Loc[2]))

    # Orientation_in_parent
    Rotation_Matrix = Transformation_Matrix[0:3,0:3].T
    Euler_Angles = - np.round(Rotation_Matrix_To_Euler_Angles(Rotation_Matrix),6)
    print("<orientation_in_parent> {O1} {O2} {O3}".format(
    O1=Euler_Angles[0], O2=Euler_Angles[1], O3=Euler_Angles[2]))


# ==============================================================
if __name__ == "__main__":
    mesh_A = trimesh.load(sys.argv[1])
    mesh_B = trimesh.load(sys.argv[2])
    icp(mesh_A, mesh_B)
