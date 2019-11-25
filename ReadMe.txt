Requires a compiler that uses at least c++17 and the eigen library
Run instructions:
1.Build using cmake. All dependencies are already present in the ext folder, except Eigen.
2.Write a json configuration file. An example named example_config.json is provided. Its structure should be as follows:
-necessary argument "plyFilename": Defines the filepath to the ply file on which the simulation will be executed.
-optional argument "nodalDisplacemntsCSV": Defines the filepath of a csv file in which the nodal displacements after the simulation are saved. If omitted the results are not saved.
-optional argument "nodalForcesCSV": Defines the filepath of a csv file in which the nodal forces after the simulation are saved. If omitted the results are not saved.
-optional argument "fixedVertices": Defines the zero-based indices of the fixed vertices. All the DoF of those vertices will be fixed in the simulation.
-optional argument "forces": [[VI1,DoF1,FM1],[VI2,DoF2,FM2],..], where
VIn denotes the index of the vertex on which the force should be applied on.
DoFn denotes the DoF of the force
FMn denotes the magnitude force in Newtons. This should be an integer.
3.After successful building an executable called MeshBeamFEA is generated. Run and pass as a run argument the filepath of the configuration file you created in step 2.
