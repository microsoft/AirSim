# How to Access Meshes in AIRSIM

AirSim supports the ability to access the static meshes that make up the scene


## Mesh structure
Each mesh is represented with the below struct.
```
struct MeshResponse {

	Vector3r position;
	Quaternionr orientation;

	std::vector<float> vertices;
	std::vector<uint32_t> indices;
	std::string name;
};
```
* The position and orientation are in the Unreal coordinate system.
* The mesh itself is a triangular mesh represented by the vertices and the indices.
	* The triangular mesh type is typically called a [Face-Vertex](https://en.wikipedia.org/wiki/Polygon_mesh#Face-vertex_meshes) Mesh. This means every triplet of indices hold the indexes of the vertices that make up the triangle/face.
	* The x,y,z coordinates of the vertices are all stored in a single vector. This means the vertices vertices is Nx3 where N is number of vertices. 
    * The position of the vertices are the global positions in the Unreal coordinate system. This means they have already been Transformed by the position and orientation.
* 
## How to use
The API to get the meshes in the scene is quite simple. However, one should note that the function call is very expensive and should
 very rarely be called. In general this is ok because this function only accesses the static meshes which for most applications are
 not changing during the duration of your program.

Note that you will have to use a 3rdparty library or your own custom code to actually interact with the recieved meshes. Below I utilize the
python bindings of [libigl](https://github.com/libigl/libigl) to visualize the recieved meshes.

```
import airsim

AIRSIM_HOST_IP='127.0.0.1'

client = airsim.VehicleClient(ip=AIRSIM_HOST_IP)
client.confirmConnection()

# List of returned meshes are received via this function
meshes=client.simGetMeshes()


index=0
for m in meshes:
    # Finds one of the cube meshes in the Blocks environment
    if 'cube' in m.name:

        # Code from here on relies on libigl. Libigl uses pybind11 to wrap C++ code. So here the built pyigl.so
        # library is in the same directory as this example code.
        # This is here as code for your own mesh library should require something similar
        from pyigl import *
        from iglhelpers import *

        # Convert the lists to numpy arrays
        vertex_list=np.array(m.vertices,dtype=np.float32)
        indices=np.array(m.indices,dtype=np.uint32)

        num_vertices=int(len(vertex_list)/3)
        num_indices=len(indices)

        # Libigl requires the shape to be Nx3 where N is number of vertices or indices
        # It also requires the actual type to be double(float64) for vertices and int64 for the triangles/indices
        vertices_reshaped=vertex_list.reshape((num_vertices,3))
        indices_reshaped=indices.reshape((int(num_indices/3),3))
        vertices_reshaped=vertices_reshaped.astype(np.float64)
        indices_reshaped=indices_reshaped.astype(np.int64)

        #Libigl function to convert to internal Eigen format
        v_eig=p2e(vertices_reshaped)
        i_eig=p2e(indices_reshaped)

        # View the mesh
        viewer = igl.glfw.Viewer()
        viewer.data().set_mesh(v_eig,i_eig)
        viewer.launch()
        break
```