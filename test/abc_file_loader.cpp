#include "abc_file_loader.hpp"

#include <Alembic/AbcCoreFactory/All.h>
#include <Alembic/AbcGeom/All.h>

#include <Eigen/Core>
#include <vector>

Eigen::MatrixXf toEigenMatrix(const Alembic::AbcGeom::P3fArraySamplePtr& p) {
  const std::size_t nVerts = p->size();
  Eigen::MatrixXf M(static_cast<int>(nVerts), 3);

  for (std::size_t v = 0; v < nVerts; ++v) {
    const Imath::V3f& q = (*p)[v];
    M(static_cast<int>(v), 0) = q.x;
    M(static_cast<int>(v), 1) = q.y;
    M(static_cast<int>(v), 2) = q.z;
  }
  return M;
}

AlembicObject loadSinglePolyMesh(const Alembic::AbcGeom::IPolyMesh& mesh) {
  using namespace Alembic::AbcGeom;

  const IPolyMeshSchema& schema = mesh.getSchema();
  const std::size_t nSamples = schema.getNumSamples();

  // ----- topology (assumes triangles) -----
  Int32ArraySamplePtr faceCounts = schema.getFaceCountsProperty().getValue(0);
  Int32ArraySamplePtr faceIndices = schema.getFaceIndicesProperty().getValue(0);

  const std::size_t nFaces = faceCounts->size();
  const std::size_t nIndices = faceIndices->size();

  if (nIndices != nFaces * 3U) {
    throw std::runtime_error("Non‑triangle topology detected in mesh \"" +
                             mesh.getName() + "\"");
  }

  Eigen::MatrixXi F(static_cast<int>(nFaces), 3);
  for (std::size_t f = 0; f < nFaces; ++f) {
    if ((*faceCounts)[f] != 3) {
      throw std::runtime_error("Non‑triangle face in mesh \"" + mesh.getName() +
                               "\"");
    }

    F(static_cast<int>(f), 0) = (*faceIndices)[3 * f + 0];
    F(static_cast<int>(f), 1) = (*faceIndices)[3 * f + 1];
    F(static_cast<int>(f), 2) = (*faceIndices)[3 * f + 2];
  }

  // ----- time‑varying positions -----
  AlembicObject out;
  out.name = mesh.getFullName();
  out.F = std::move(F);

  for (std::size_t s = 0; s < nSamples; ++s) {
    P3fArraySamplePtr pos = schema.getPositionsProperty().getValue(s);
    out.series.push_back(toEigenMatrix(pos));
  }

  out.V = out.series.front();  // rest pose (first sample)
  return out;
}

std::vector<AlembicObject> loadAllMeshes(const std::string& path_to_abc) {
  using namespace Alembic;

  AbcCoreFactory::IFactory factory;
  Abc::IArchive archive = factory.getArchive(path_to_abc);
  if (!archive.valid()) {
    throw std::runtime_error("Cannot open Alembic file: " + path_to_abc);
  }

  std::vector<AlembicObject> result;
  std::function<void(const Abc::IObject&)> recurse;
  recurse = [&](const Abc::IObject& obj) {
    for (std::size_t i = 0; i < obj.getNumChildren(); ++i) {
      Abc::IObject child = obj.getChild(i);

      if (AbcGeom::IPolyMesh::matches(child.getHeader())) {
        AbcGeom::IPolyMesh mesh(child, AbcGeom::kWrapExisting);
        result.push_back(loadSinglePolyMesh(mesh));
      }

      recurse(child);  // depth‑first traversal
    }
  };

  recurse(Abc::IObject(archive, Abc::kTop));
  return result;
}
