# MeshBeamFEA

A tool that to model a triangular mesh using the Euler-Bernoulli beam theory and executes a simulation on it which can be configured using a json file. Displacements and forces are exported in a csv file.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

Eigen and C++17

### Installing
Build using cmake.
### Running
1. Write a json configuration file. An example named example_config.json is provided. Its structure should be as follows: <br/>
-*Compulsory* **plyFilename**: Defines the filepath to the ply file on which the simulation will be executed.<br/>
-*Optional* **nodalDisplacementsCSV**: Defines the filepath of a csv file in which the nodal displacements after the simulation are saved. If omitted the results are not saved.<br/>
-*Optional* **nodalForcesCSV**: Defines the filepath of a csv file in which the nodal forces after the simulation are saved. If omitted the results are not saved.<br/>
-*Optional* **fixedVertices**: Defines the zero-based indices of the fixed vertices. All the DoF of those vertices will be fixed in the simulation.<br/>
-*Optional* **forces**: [[VI1,DoF1,FM1],[VI2,DoF2,FM2],..], where <br/>
VIn denotes the index of the vertex on which the force should be applied on.<br/>
DoFn denotes the DoF of the force<br/>
FMn denotes the magnitude force in Newtons. This should be an integer.<br/>
2. After successful building an executable called MeshBeamFEA is generated. Run and pass as a run argument the filepath of the configuration file you created in the previous step.<br/>

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details


