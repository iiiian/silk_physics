#pragma once

#include <Eigen/Core>
#include <array>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "../broadphase/broadphase_common.hpp"

namespace silk::broadphase_benchmark {
class CpuAdapter;
}

namespace silk::narrowphase_benchmark {

enum class QueryKind { EE, VF };

struct Query {
  std::array<Eigen::Vector3f, 8> vertices;
  Eigen::Array3f err;
  bool expected = false;
  bool expected_known = false;
  int timestep = -1;
  int timestep_query = -1;
};

struct QueryOutput {
  float toi = 1.0f;
  bool hit = false;
  bool searched_full_interval = true;
};

using SceneFiles = broadphase_benchmark::SceneFiles;

struct SceneBounds {
  Eigen::Vector3f min;
  Eigen::Vector3f max;
};

struct QueryBatch {
  std::string scene;
  QueryKind kind = QueryKind::EE;
  int timestep_num = 0;
  std::vector<Query> queries;
};

class CpuAdapter {
 public:
  virtual ~CpuAdapter() = default;
  virtual std::string name() const = 0;
  virtual QueryOutput query(const Query& input, float max_time) = 0;
};

std::vector<SceneFiles> resolve_scene_files(
    const std::filesystem::path& data_root);
SceneBounds load_scene_bounds(const SceneFiles& scene_files);
QueryBatch load_batch(const SceneFiles& scene_files, QueryKind kind,
                      const SceneBounds& scene_bounds, int timestep_index,
                      broadphase_benchmark::CpuAdapter* broadphase);
std::string query_name(QueryKind kind);
float compute_minimum_separation(const Query& query, QueryKind kind);

std::unique_ptr<CpuAdapter> make_silk_ticcd_adapter(QueryKind kind,
                                                    int max_iterations);
std::unique_ptr<CpuAdapter> make_original_ticcd_adapter(QueryKind kind,
                                                        int max_iterations);
#ifdef SILK_NARROWPHASE_HAS_CUDA
class GpuAdapter {
 public:
  virtual ~GpuAdapter() = default;
  virtual std::string name() const = 0;
  virtual void prepare(std::span<const Query> queries) = 0;
  virtual void synchronize() = 0;
  virtual void query() = 0;
  virtual std::span<const QueryOutput> output() const = 0;
};

std::unique_ptr<GpuAdapter> make_scalable_gpu_adapter(QueryKind kind,
                                                      int max_iterations);
std::unique_ptr<GpuAdapter> make_silk_hybrid_ticcd_adapter(QueryKind kind,
                                                           int max_iterations);
#endif

}  // namespace silk::narrowphase_benchmark
