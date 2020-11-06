import sys
import trimesh
import numpy as np
import math

# ==============================================================
# Method according to: Slabaugh, Gregory G. « Computing Euler Angles from a Rotation Matrix »
def Rotation_Matrix_To_Euler_Angles(R):
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
    # ---
    # Transformation_matrix = 4x4 matrix mapping the source mesh to the target
    # ---

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
def find_best_fit(mesh_source, mesh_target):
    #https://trimsh.org/trimesh.registration.html

    points_source = mesh_source.vertices
    points_target = mesh_target.vertices

    # Method 1
    T_1 = trimesh.registration.mesh_other(mesh_source, mesh_target)
    # Method 2
    T_2 = mesh_source.register(other=mesh_target, icp_first=10000,icp_final=100000) #,icp_first=10000,icp_final=100000
    # Method 3
    T_3 = icp(points_source, points_target, init_pose=None, max_iterations=10000000000, tolerance=0.0000000000000001)

    # Store Error and Shift
    err = np.array([T_1[1], T_2[1], np.mean(pow(T_3[1],2))])
    T = [T_1[0], T_2[0], T_3[0]]

    # Find best
    best_id = np.where(err == np.min(err))[0][0]
    #print("method used is nb:", best_id)

    return T[best_id], np.min(err)


# ==============================================================
# Code from: https://github.com/ClayFlannigan/icp/blob/master/icp.py

from sklearn.neighbors import NearestNeighbors

def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Nxm numpy array of corresponding points
      B: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    '''
    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[m-1,:] *= -1
       R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T)

    # homogeneous transformation
    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t

def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''

    assert src.shape == dst.shape

    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()

def icp(A, B, init_pose=None, max_iterations=20, tolerance=0.001):
    '''
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nxm numpy array of source mD points
        B: Nxm numpy array of destination mD point
        init_pose: (m+1)x(m+1) homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation that maps A on to B
        distances: Euclidean distances (errors) of the nearest neighbor
        i: number of iterations to converge
    '''

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # make points homogeneous, copy them to maintain the originals
    src = np.ones((m+1,A.shape[0]))
    dst = np.ones((m+1,B.shape[0]))
    src[:m,:] = np.copy(A.T)
    dst[:m,:] = np.copy(B.T)

    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0

    for i in range(max_iterations):
        # find the nearest neighbors between the current source and destination points
        distances, indices = nearest_neighbor(src[:m,:].T, dst[:m,:].T)

        # compute the transformation between the current source and nearest destination points
        T,_,_ = best_fit_transform(src[:m,:].T, dst[:m,indices].T)

        # update the current source
        src = np.dot(T, src)

        # check error
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    # calculate final transformation
    T,_,_ = best_fit_transform(A, src[:m,:].T)

    return T, distances, i



# ==============================================================
def Compute_ICP_by_steps(meshes):
    T_steps = []
    nb_steps = len(meshes)-1

    # Calculating transformations for each steps
    for i in range(nb_steps):
        T_local, err_local = find_best_fit(meshes[nb_steps-i], meshes[nb_steps-i-1])
        T_steps.append(T_local)
        print("Error at step {i} = {e}".format(i=i, e=np.format_float_scientific(err_local, precision=3)))

    # Calculating whole tranformation
    TM = T_steps[-1]
    for i in range(nb_steps-1):
        TM = np.matmul(TM, T_steps[nb_steps-2-i])

    return TM

# ==============================================================
def print_output(T):

    print("")
    print("#### OPENSIM")
    Get_location_and_orientation_in_parent(T)

    print("#### BLENDER")
    print("Copy between the '---' and Paste in Blender Console:")
    print("-----------")
    print("import bpy")
    print("Transform = ((",T[0,0],",",T[1,0],",",T[2,0],",",T[3,0],"),",
    "(",T[0,1],",",T[1,1],",",T[2,1],",",T[3,1],"),",
    "(",T[0,2],",",T[1,2],",",T[2,2],",",T[3,2],"),",
    "(",T[0,3],",",T[1,3],",",T[2,3],",",T[3,3],"))")
    print("bpy.context.object.matrix_world = Transform")
    print("-----------")

# ==============================================================
# ==============================================================
# ==============================================================
# ==============================================================
# ==============================================================
# ==============================================================
# ==============================================================

# Loading meshes
print("")

# Loading first file:
Loaded = False
while Loaded == False:
    entry = str(input("Enter the SOURCE mesh file: "))
    try:
        mesh_source = trimesh.load(entry)
    except:
        print("File failed to load, try again.")
    else:
        Loaded = True

# Loading second file:
Loaded = False
while Loaded == False:
    entry = str(input("Enter the TARGET mesh file: "))
    try:
        mesh_target = trimesh.load(entry)
    except:
        print("File failed to load, try again.")
    else:
        Loaded = True

print("")

# Testing in 3 tries if there is a possible fit
test_nb = 0
err = 1
print("Finding a direct fit.")
while err > 1e-15 and test_nb < 3:
    print("Try {i}/3".format(i=test_nb+1))
    T, err = find_best_fit(mesh_source, mesh_target)
    test_nb+=1



if err < 1e-15:
    print("A direct fit was found with error:", np.format_float_scientific(err, precision=3))
    print_output(T)

else:
    meshes = []
    meshes_nb = 1
    print("")
    print("#### No direct fit were found after 3 tries. ####")
    print("")
    print("Enter below the meshes in the order FROM RAW TO MOST PROCESSED:")

    entry = ""
    while entry != "X":
        entry = str(input("Enter mesh number {i}, or 'X' when finished: ".format(i=meshes_nb)))
        if entry != "X":
            try:
                meshes.append(trimesh.load(entry))
            except:
                print("File failed to load, try again.")
            else:
                print("Number of meshes entered:", meshes_nb)
                meshes_nb += 1

    print("")
    print("Computing transforms by steps...")
    TM = Compute_ICP_by_steps(meshes)

    print_output(TM)
