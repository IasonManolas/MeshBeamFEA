#include "edgemesh.hpp"

Eigen::MatrixX2i VCGEdgeMesh::getEigenEdges() const { return eigenEdges; }

Eigen::MatrixX3d VCGEdgeMesh::getEigenVertices() const { return eigenVertices; }

Eigen::MatrixX3d VCGEdgeMesh::getEigenEdgeNormals() const {
  return eigenEdgeNormals;
}

std::vector<BeamDimensions> VCGEdgeMesh::getBeamDimensions() const {
  return handleBeamDimensions._handle->data;
}

std::vector<BeamMaterial> VCGEdgeMesh::getBeamMaterial() const {
  return handleBeamMaterial._handle->data;
}

bool VCGEdgeMesh::savePly(const std::string plyFilename,
                          const Eigen::MatrixX3d &V) {
  std::vector<nanoply::PlyProperty> vertexProp;
  vertexProp.push_back(
      nanoply::PlyProperty(nanoply::NNP_FLOAT32, nanoply::NNP_PXYZ));
  //      std::vector<nanoply::PlyProperty> edgeProp;
  //      edgeProp.push_back(nanoply::PlyProperty(nanoply::NNP_INT32,nanoply::NNP_EDGE_V1));
  //      edgeProp.push_back(nanoply::PlyProperty(nanoply::NNP_INT32,nanoply::NNP_EDGE_V2));

  nanoply::PlyElement vertexElem(nanoply::NNP_VERTEX_ELEM, vertexProp,
                                 V.rows());
  //      nanoply::PlyElement
  //      edgeElem(nanoply::NNP_EDGE_ELEM,edgeProp,E.rows());

  nanoply::Info info;
  info.filename = plyFilename;
  info.binary = false;
  info.AddPlyElement(vertexElem);
  //      info.AddPlyElement(edgeElem);

  nanoply::ElementDescriptor vertexDescriptor(nanoply::NNP_VERTEX_ELEM);
  std::vector<std::vector<double>> vertices;
  igl::matrix_to_list(V, vertices);
  vertexDescriptor.dataDescriptor.push_back(
      new nanoply::DataDescriptor<std::vector<double>, 3, double>(
          nanoply::NNP_PXYZ, &(*vertices.begin())));

  std::vector<nanoply::ElementDescriptor *> meshDescr;
  meshDescr.push_back(&vertexDescriptor);

  bool result = nanoply::SaveModel(info.filename, meshDescr, info);

  for (int i = 0; i < vertexDescriptor.dataDescriptor.size(); i++)
    delete vertexDescriptor.dataDescriptor[i];

  return result;
}

bool VCGEdgeMesh::savePly(const std::string plyFilename) {
  nanoply::NanoPlyWrapper<VCGEdgeMesh>::CustomAttributeDescriptor customAttrib;
  customAttrib.GetMeshAttrib(plyFilename);
  customAttrib.AddEdgeAttribDescriptor<vcg::Point3d, double, 3>(
      plyPropertyEdgeNormalID, nanoply::NNP_LIST_UINT8_FLOAT64,
      &handleEdgeNormals[0]);
  customAttrib.AddEdgeAttribDescriptor<BeamDimensions, float, 2>(
      plyPropertyBeamDimensionsID, nanoply::NNP_LIST_UINT8_FLOAT32,
      &handleBeamDimensions[0]);
  customAttrib.AddEdgeAttribDescriptor<vcg::Point2f, float, 2>(
      plyPropertyBeamMaterialID, nanoply::NNP_LIST_UINT8_FLOAT32,
      &handleBeamMaterial[0]);
  // Load the ply file
  unsigned int mask = 0;
  mask |= nanoply::NanoPlyWrapper<VCGEdgeMesh>::IO_VERTCOORD;
  mask |= nanoply::NanoPlyWrapper<VCGEdgeMesh>::IO_EDGEINDEX;
  mask |= nanoply::NanoPlyWrapper<VCGEdgeMesh>::IO_EDGEATTRIB;
  if (nanoply::NanoPlyWrapper<VCGEdgeMesh>::SaveModel(
          plyFilename.c_str(), *this, mask, customAttrib, false) != 0) {
    return false;
  }
  return true;
}

bool VCGEdgeMesh::loadFromPly(const std::string plyFilename) {

  this->Clear();
  const bool useDefaultImporter = false;
  if (useDefaultImporter) {
    if (!loadUsingDefaultLoader(plyFilename)) {
      return false;
    }

    eigenEdgeNormals.resize(EN(), 3);
    for (int i = 0; i < EN(); i++) {
      eigenEdgeNormals.row(i) = Eigen::Vector3d(0, 1, 0);
    }
  } else {
    if (!loadUsingNanoply(plyFilename)) {
      std::cerr << "Error: Unable to open " + plyFilename << std::endl;
      return false;
    }
    eigenEdgeNormals.resize(EN(), 3);
    for (int edgeIndex = 0; edgeIndex < EN(); edgeIndex++) {
      const vcg::Point3d &edgeNormal = handleEdgeNormals[edgeIndex];
      Eigen::Vector3d eigenEdgeNormal;
      edgeNormal.ToEigenVector(eigenEdgeNormal);
      eigenEdgeNormals.row(edgeIndex) = eigenEdgeNormal;
    }
  }
  getEdges(eigenEdges);
  getVertices(eigenVertices);
  std::cout << plyFilename << " was loaded successfuly." << std::endl;

  std::cout << "Mesh has " << EN() << " edges." << std::endl;
  return true;
}

bool VCGEdgeMesh::loadUsingNanoply(const std::string &plyFilename) {
  assert(plyFileHasAllRequiredFields(plyFilename));
  nanoply::NanoPlyWrapper<VCGEdgeMesh>::CustomAttributeDescriptor customAttrib;
  customAttrib.GetMeshAttrib(plyFilename);
  customAttrib.AddEdgeAttribDescriptor<vcg::Point3d, double, 3>(
      plyPropertyEdgeNormalID, nanoply::NNP_LIST_UINT8_FLOAT64, nullptr);
  customAttrib.AddEdgeAttribDescriptor<BeamDimensions, float, 2>(
      plyPropertyBeamDimensionsID, nanoply::NNP_LIST_UINT8_FLOAT32, nullptr);
  customAttrib.AddEdgeAttribDescriptor<vcg::Point2f, float, 2>(
      plyPropertyBeamMaterialID, nanoply::NNP_LIST_UINT8_FLOAT32, nullptr);
  // Load the ply file
  unsigned int mask = 0;
  mask |= nanoply::NanoPlyWrapper<VCGEdgeMesh>::IO_VERTCOORD;
  mask |= nanoply::NanoPlyWrapper<VCGEdgeMesh>::IO_EDGEINDEX;
  mask |= nanoply::NanoPlyWrapper<VCGEdgeMesh>::IO_EDGEATTRIB;
  if (nanoply::NanoPlyWrapper<VCGEdgeMesh>::LoadModel(
          plyFilename.c_str(), *this, mask, customAttrib) != 0) {
    return false;
  }
  return true;
}

bool VCGEdgeMesh::plyFileHasAllRequiredFields(const std::string &plyFilename) {
  const nanoply::Info info(plyFilename);
  const std::vector<nanoply::PlyElement>::const_iterator edgeElemIt =
      std::find_if(info.elemVec.begin(), info.elemVec.end(),
                   [&](const nanoply::PlyElement &plyElem) {
                     return plyElem.plyElem == nanoply::NNP_EDGE_ELEM;
                   });
  if (edgeElemIt == info.elemVec.end()) {
    std::cerr << "Ply file is missing edge elements." << std::endl;
    return false;
  }

  const std::vector<nanoply::PlyProperty> &edgePropertyVector =
      edgeElemIt->propVec;
  return hasPlyEdgeProperty(plyFilename, edgePropertyVector,
                            plyPropertyEdgeNormalID) &&
         hasPlyEdgeProperty(plyFilename, edgePropertyVector,
                            plyPropertyBeamDimensionsID) &&
         hasPlyEdgeProperty(plyFilename, edgePropertyVector,
                            plyPropertyBeamMaterialID);
}

bool VCGEdgeMesh::hasPlyEdgeProperty(
    const std::string &plyFilename,
    const std::vector<nanoply::PlyProperty> &edgeProperties,
    const std::string &plyEdgePropertyName) {
  const bool hasEdgeProperty = hasProperty(edgeProperties, plyEdgePropertyName);
  if (!hasEdgeProperty) {
    std::cerr << "Ply file " + plyFilename +
                     " is missing the propertry:" + plyEdgePropertyName
              << std::endl;
    return false;
  }
  return true;
}

bool VCGEdgeMesh::hasProperty(const std::vector<nanoply::PlyProperty> &v,
                              const std::string &propertyName) {
  return v.end() != std::find_if(v.begin(), v.end(),
                                 [&](const nanoply::PlyProperty &plyProperty) {
                                   return plyProperty.name == propertyName;
                                 });
}

bool VCGEdgeMesh::loadUsingDefaultLoader(const std::string &plyFilename) {
  int returnValue =
      vcg::tri::io::ImporterPLY<VCGEdgeMesh>::Open(*this, plyFilename.c_str());
  if (returnValue != 0) {
    std::cerr << "Error: Unable to open " + plyFilename + ". Error Message:"
              << vcg::tri::io::ImporterPLY<VCGEdgeMesh>::ErrorMsg(returnValue)
              << std::endl;
    return false;
  }
  return true;
}

void VCGEdgeMesh::getNormals(Eigen::MatrixX3d &normals) const {
  normals.resize(VN(), 3);
  for (int vertexIndex = 0; vertexIndex < VN(); vertexIndex++) {
    VCGEdgeMesh::CoordType vertexNormal =
        vert[static_cast<size_t>(vertexIndex)].cN();
    normals.row(vertexIndex) = convertToEigenVector(vertexNormal);
  }
}

void VCGEdgeMesh::getBeamMesh(const float &beamThickness,
                              VCGTriMesh &beamMesh) const {
  for (size_t edgeIndex = 0; edgeIndex < static_cast<size_t>(this->EN());
       edgeIndex++) {
    const VCGEdgeMesh::EdgeType &edge = this->edge[edgeIndex];
    const VCGEdgeMesh::CoordType &p0 = edge.cP(0);
    const VCGEdgeMesh::CoordType &p1 = edge.cP(1);
    VCGTriMesh beam;
    vcg::tri::OrientedCylinder(beam, p0, p1, beamThickness, true, 4, 1);
    vcg::tri::Append<VCGTriMesh, VCGTriMesh>::Mesh(beamMesh, beam);
  }
}

void VCGEdgeMesh::getEdges(Eigen::MatrixX3d &edgeStartingPoints,
                           Eigen::MatrixX3d &edgeEndingPoints) const {
  edgeStartingPoints.resize(EN(), 3);
  edgeEndingPoints.resize(EN(), 3);
  for (int edgeIndex = 0; edgeIndex < EN(); edgeIndex++) {
    const VCGEdgeMesh::EdgeType &edge = this->edge[edgeIndex];
    edgeStartingPoints.row(edgeIndex) = (convertToEigenVector(edge.cP(0)));
    edgeEndingPoints.row(edgeIndex) = (convertToEigenVector(edge.cP(1)));
  }
}

void VCGEdgeMesh::setDefaultAttributes() {
  for (gsl::index edgeIndex = 0; edgeIndex < EN(); edgeIndex++) {
    handleEdgeNormals[edgeIndex] = vcg::Point3d(0, 0, 1);
    handleBeamMaterial[edgeIndex] = BeamMaterial();
    handleBeamDimensions[edgeIndex] = BeamDimensions(0.01, 0.01);
  }
}

VCGEdgeMesh::VCGEdgeMesh() {
  handleEdgeNormals =
      vcg::tri::Allocator<VCGEdgeMesh>::AddPerEdgeAttribute<vcg::Point3d>(
          *this, plyPropertyEdgeNormalID);
  handleBeamDimensions =
      vcg::tri::Allocator<VCGEdgeMesh>::AddPerEdgeAttribute<BeamDimensions>(
          *this, plyPropertyBeamDimensionsID);
  handleBeamMaterial =
      vcg::tri::Allocator<VCGEdgeMesh>::AddPerEdgeAttribute<BeamMaterial>(
          *this, plyPropertyBeamMaterialID);
}

void VCGEdgeMesh::getVertices(Eigen::MatrixX3d &vertices) {
  vertices.resize(VN(), 3);
  for (int vertexIndex = 0; vertexIndex < VN(); vertexIndex++) {
    VCGEdgeMesh::CoordType vertexCoordinates =
        vert[static_cast<size_t>(vertexIndex)].cP();
    vertices.row(vertexIndex) = convertToEigenVector(vertexCoordinates);
  }
}

void VCGEdgeMesh::getEdges(Eigen::MatrixX2i &edges) {
  edges.resize(EN(), 2);
  for (int edgeIndex = 0; edgeIndex < EN(); edgeIndex++) {
    const VCGEdgeMesh::EdgeType &edge = this->edge[edgeIndex];
    const size_t nodeIndex0 = vcg::tri::Index<VCGEdgeMesh>(*this, edge.cV(0));
    const size_t nodeIndex1 = vcg::tri::Index<VCGEdgeMesh>(*this, edge.cV(1));
    edges.row(edgeIndex) = Eigen::Vector2i(nodeIndex0, nodeIndex1);
  }
}
