#include <Alembic/AbcCoreOgawa/All.h>
#include <Alembic/AbcGeom/All.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <system_error>
#include <vector>

#include "object.hpp"

// The Alembic library itself has little documentation. This blender page might
// be helpful: https://developer.blender.org/docs/features/objects/io/alembic

bool write_scene(const std::filesystem::path& path,
                 const std::vector<pIObject>& objects) {
  namespace Abc = Alembic::Abc;
  namespace AbcGeom = Alembic::AbcGeom;
  namespace AbcA = Alembic::AbcCoreAbstract;
  namespace fs = std::filesystem;

  assert(!objects.empty());

  // Create parent dir
  const fs::path parent = path.parent_path();
  if (!parent.empty()) {
    std::error_code ec;
    fs::create_directories(parent, ec);
    if (ec) {
      spdlog::error("Failed to create Alembic export directory '{}': {}",
                    parent.string(), ec.message());
      return false;
    }
  }

  try {
    Abc::OArchive archive(Alembic::AbcCoreOgawa::WriteArchive(), path.string());
    Abc::OObject top = archive.getTop();

    for (auto& obj : objects) {
      assert(obj != nullptr);

      // Time sample.
      auto& cache = obj->get_cache();
      if (cache.empty()) {
        spdlog::info(
            "Object {} has no simulation cache, skip writing to .abc file.",
            obj->get_name());
        continue;
      }

      std::vector<AbcA::chrono_t> sample_times;
      for (const auto& [time, verts] : cache) {
        sample_times.push_back(static_cast<AbcA::chrono_t>(time));
      }
      assert(std::is_sorted(sample_times.begin(), sample_times.end()));

      AbcA::TimeSampling sampling(
          AbcA::TimeSamplingType(AbcA::TimeSamplingType::kAcyclic),
          sample_times);
      uint32_t sampling_index = archive.addTimeSampling(sampling);

      // Face topology.
      const Face& faces = obj->get_faces();
      assert(faces.rows() != 0);
      std::vector<int32_t> face_counts(faces.rows(), 3);
      std::vector<int32_t> face_indices(faces.size());
      for (int i = 0; i < faces.rows(); ++i) {
        face_indices[3 * i] = static_cast<int32_t>(faces(i, 0));
        face_indices[3 * i + 1] = static_cast<int32_t>(faces(i, 1));
        face_indices[3 * i + 2] = static_cast<int32_t>(faces(i, 2));
      }

      // PolyMesh.
      AbcGeom::OPolyMesh mesh_obj{top, obj->get_name()};
      AbcGeom::OPolyMeshSchema& schema = mesh_obj.getSchema();
      schema.setTimeSampling(sampling_index);

      // Vertex position
      auto make_p3f_array_sample = [](const Vert& v) -> Abc::P3fArraySample {
        // P3fArraySample represents a non-owning vector of Imath::V3f. Under
        // default alignment, this is just a continuous float array. So we
        // simply pass our row major eigen matrix and treat it as a vector of
        // Imath::V3f.
        auto ptr = reinterpret_cast<const Imath::V3f*>(v.data());
        size_t size = v.rows();
        return Abc::P3fArraySample{ptr, size};
      };

      // Pass both vertex positions and face topology in the first sample.
      AbcGeom::OPolyMeshSchema::Sample base_sample{
          make_p3f_array_sample(cache.front().second),
          Abc::Int32ArraySample(face_indices),
          Abc::Int32ArraySample(face_counts)};
      schema.set(base_sample);

      // Pass vertex position only for subseqent samples.
      for (size_t i = 1; i < cache.size(); ++i) {
        AbcGeom::OPolyMeshSchema::Sample pos_sample{
            make_p3f_array_sample(cache[i].second)};
        schema.set(pos_sample);
      }
    }

  } catch (const std::exception& e) {
    spdlog::error("Alembic error: {}.", e.what());
    return false;
  }

  return true;
}
