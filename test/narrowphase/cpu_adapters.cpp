#include <limits>
#include <memory>
#include <string>
#include <tight_inclusion/ccd.hpp>

#include "backend/cpu/collision/ccd.hpp"
#include "narrowphase_common.hpp"

namespace silk::narrowphase_benchmark {

namespace {

constexpr float TOLERANCE = 1e-6f;

class SilkTiccdAdapter final : public CpuAdapter {
 public:
  SilkTiccdAdapter(QueryKind kind, int max_iterations)
      : kind_(kind), max_iterations_(max_iterations) {}

  std::string name() const override { return "silk_ticcd"; }

  QueryOutput query(const Query& input, float max_time) override {
    std::optional<cpu::CCDResult> result;
    float minimum_separation = compute_minimum_separation(input, kind_);
    if (kind_ == QueryKind::EE) {
      result = cpu::edge_edge_ccd(
          input.vertices[0], input.vertices[1], input.vertices[2],
          input.vertices[3], input.vertices[4], input.vertices[5],
          input.vertices[6], input.vertices[7], input.err, minimum_separation,
          TOLERANCE, max_iterations_, false, max_time);
    } else {
      result = cpu::vertex_face_ccd(
          input.vertices[0], input.vertices[1], input.vertices[2],
          input.vertices[3], input.vertices[4], input.vertices[5],
          input.vertices[6], input.vertices[7], input.err, minimum_separation,
          TOLERANCE, max_iterations_, false, max_time);
    }
    return {.toi = result ? result->t(0) : 1.0f, .hit = result.has_value()};
  }

 private:
  QueryKind kind_;
  int max_iterations_;
};

class OriginalTiccdAdapter final : public CpuAdapter {
 public:
  OriginalTiccdAdapter(QueryKind kind, int max_iterations)
      : kind_(kind), max_iterations_(max_iterations) {}

  std::string name() const override { return "original_ticcd"; }

  QueryOutput query(const Query& input, float max_time) override {
    float toi = std::numeric_limits<float>::infinity();
    float output_tolerance = TOLERANCE;
    float minimum_separation = compute_minimum_separation(input, kind_);
    bool hit;
    if (kind_ == QueryKind::EE) {
      hit = ticcd::edgeEdgeCCD(
          input.vertices[0], input.vertices[1], input.vertices[2],
          input.vertices[3], input.vertices[4], input.vertices[5],
          input.vertices[6], input.vertices[7], input.err, minimum_separation,
          toi, TOLERANCE, max_time, max_iterations_, output_tolerance, false);
    } else {
      hit = ticcd::vertexFaceCCD(
          input.vertices[0], input.vertices[1], input.vertices[2],
          input.vertices[3], input.vertices[4], input.vertices[5],
          input.vertices[6], input.vertices[7], input.err, minimum_separation,
          toi, TOLERANCE, max_time, max_iterations_, output_tolerance, false);
    }
    return {.toi = hit ? toi : 1.0f, .hit = hit};
  }

 private:
  QueryKind kind_;
  int max_iterations_;
};

}  // namespace

std::unique_ptr<CpuAdapter> make_silk_ticcd_adapter(
    QueryKind kind, int max_iterations) {
  return std::make_unique<SilkTiccdAdapter>(kind, max_iterations);
}

std::unique_ptr<CpuAdapter> make_original_ticcd_adapter(
    QueryKind kind, int max_iterations) {
  return std::make_unique<OriginalTiccdAdapter>(kind, max_iterations);
}

}  // namespace silk::narrowphase_benchmark
