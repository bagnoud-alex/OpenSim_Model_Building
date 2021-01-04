# OpenSim Model Building
This Repository aims to provide tools to automate steps to construct a skeletal model in OpenSim.  
All scripts are made to work with the 'trimesh' python library: https://trimsh.org/  
Most of the scripts give an output compatible with the Blender Python console, which is recommended to work with for the building of the model  

### The scripts below are ordered in the suggested flow of work:

**is_watertight.py**
- Checks if all mesh files are watertight in a given folder
- Needed step as most of the other scripts recquire watertight meshes
- use: `python is_watertight.py`

**fit_sphere.py**  
**fit_cylinder.py**
- Both scripts fit a cylinder or a sphere around a given mesh
- Output provides center of the cylinder/sphere as well as blender command to create, place and orient the fitted shape in the right place
- use: `python fit_sphere mesh_name.stl` or `python fit_cylinder mesh_name.stl`

**transform_inertia.py**
- Rotates a mesh file along its principle axes of inertia
- Output provides the transformation matrix to directly paste in Blender python console
- use: `python transform_inertia.py mesh_name.stl`


**Rotate_Reference_Frame.ipynb**
- Notebook which can help to give specific axes to a body

**get_properties.py**
- Outputs properties for Opensim such as mass, center of mass, inertia tensor
- use: `python get_properties.py mesh_name.stl density=1060`
- density is an optionnal input, default value is 1060kg/m^3
- use 1132kg/m^3 for bones, 1060kg/m^3 for a mean density of all tissues

**icp.py**
- Script to align a source mesh to a taget mesh. Useful to place a processed body (where the origin or axes have been shifted) to a unprocessed body (for example from a CT scan)
- Outputs <location_in_parent> as well as <orientation_in_parent>, both needed for OpenSim model building
- use: `python icp.py source_mesh.stl target_mesh.stl`
