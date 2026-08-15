#include "broadphase_common.hpp"
#include "common/perf_region_control.hpp"
#include "cpu_adapters.hpp"

#ifdef SILK_BROADPHASE_HAS_CUDA
#include "gpu_adapters.cuh"
#endif

#include <spdlog/spdlog.h>
#include <tbb/global_control.h>

#include <algorithm>
#include <argparse/argparse.hpp>
#include <array>
#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace silk::broadphase_benchmark {

namespace {

struct Options {
  bool verify = false;
  int repetitions = 3;
  int thread_num = std::max(1U, std::thread::hardware_concurrency());
  std::filesystem::path data_root = SILK_BROADPHASE_DATA_ROOT;
  std::filesystem::path output_path;
  std::string scene;
  std::string broadphase;
};

struct BenchmarkResult {
  std::vector<double> initial_build_ms;
  std::vector<double> rebuild_ms;
  std::vector<double> query_ms;
  uint64_t box_num = 0;
  uint64_t candidate_num = 0;
  int timestep_num = 0;
};

struct BenchmarkKey {
  std::string scene;
  QueryKind query_kind;
  std::string algorithm;
  std::string device;

  auto operator<=>(const BenchmarkKey&) const = default;
};

Options parse_options(int argc, char** argv) {
  Options options;
  argparse::ArgumentParser program("broadphase_benchmark");
  program.add_description(
      "Verify and benchmark broadphase implementations on Scalable CCD data");
  program.add_argument("--verify")
      .help("Check broadphase output against dataset ground truth")
      .store_into(options.verify);
  program.add_argument("--repetitions")
      .help("Number of complete scene sweeps")
      .store_into(options.repetitions);
  program.add_argument("--threads")
      .help("Maximum CPU thread count")
      .store_into(options.thread_num);
  program.add_argument("--data")
      .help("Scalable CCD dataset root")
      .store_into(options.data_root);
  program.add_argument("--out")
      .help("Benchmark CSV output path")
      .store_into(options.output_path);
  program.add_argument("--scene")
      .help(
          "Run only one scene: armadillo-rollers, cloth-ball, cloth-funnel, "
          "n-body-simulation, puffer-ball, or rod-twist")
      .store_into(options.scene);
  program.add_argument("--broadphase")
      .help(
          "Run only one broadphase: silk_kdtree, scalable_sap, embree, "
          "silk_oibvh, scalable_stq, or cubql")
      .store_into(options.broadphase);
  program.parse_args(argc, argv);

  if (options.repetitions <= 0 || options.thread_num <= 0) {
    throw std::runtime_error("Repetitions and thread count must be positive");
  }
  if (!options.verify && options.output_path.empty()) {
    throw std::runtime_error("--out is required in benchmark mode");
  }
  return options;
}

std::string query_name(QueryKind kind) {
  return kind == QueryKind::EE ? "ee" : "vf";
}

std::vector<Pair> canonical_output(const QueryInput& input,
                                   std::span<const Pair> raw_output,
                                   const std::string& algorithm,
                                   const TestCase& test_case) {
  std::vector<Pair> output;
  output.reserve(raw_output.size());
  for (Pair pair : raw_output) {
    pair = normalized_pair(pair, input.kind);
    if (pair.first < 0 || pair.first >= input.group_a.size() ||
        pair.second < 0 ||
        pair.second >= (input.kind == QueryKind::EE ? input.group_a.size()
                                                    : input.group_b.size())) {
      throw std::runtime_error(algorithm + " returned an invalid pair in " +
                               test_case.scene + "/" +
                               std::to_string(test_case.timestep));
    }
    output.push_back(pair);
  }

  std::sort(output.begin(), output.end());
  output.erase(std::unique(output.begin(), output.end()), output.end());
  return output;
}

void verify_output(const QueryInput& input, std::span<const Pair> raw_output,
                   const std::string& algorithm, const TestCase& test_case) {
  std::vector<Pair> output =
      canonical_output(input, raw_output, algorithm, test_case);

  if (!std::includes(output.begin(), output.end(), input.required_pairs.begin(),
                     input.required_pairs.end())) {
    std::vector<Pair> missing;
    std::set_difference(input.required_pairs.begin(),
                        input.required_pairs.end(), output.begin(),
                        output.end(), std::back_inserter(missing));
    Pair first = missing.front();
    throw std::runtime_error(
        algorithm + " missed required pair (" + std::to_string(first.first) +
        ", " + std::to_string(first.second) + ") in " + test_case.scene + "/" +
        std::to_string(test_case.timestep));
  }
}

double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  return values[values.size() / 2];
}

void prepare_repetition(BenchmarkResult& result, int repetition) {
  if (result.initial_build_ms.size() <= repetition) {
    result.initial_build_ms.resize(repetition + 1, 0.0);
    result.rebuild_ms.resize(repetition + 1, 0.0);
    result.query_ms.resize(repetition + 1, 0.0);
  }
}

template <typename Function>
double measure_wall_time(Function&& function) {
  using Clock = std::chrono::steady_clock;
  auto start = Clock::now();
  function();
  auto done = Clock::now();
  return std::chrono::duration<double, std::milli>(done - start).count();
}

void record_measurement(const QueryInput& input, const TestCase& test_case,
                        const std::string& algorithm, const std::string& device,
                        int repetition, bool initial_build, double build_ms,
                        double query_ms, std::size_t candidate_num,
                        std::map<BenchmarkKey, BenchmarkResult>& results) {
  BenchmarkKey key{.scene = test_case.scene,
                   .query_kind = input.kind,
                   .algorithm = algorithm,
                   .device = device};
  BenchmarkResult& result = results[key];
  prepare_repetition(result, repetition);
  if (initial_build) {
    result.initial_build_ms[repetition] += build_ms;
  } else {
    result.rebuild_ms[repetition] += build_ms;
  }
  result.query_ms[repetition] += query_ms;
  if (repetition == 0) {
    result.box_num += input.group_a.size() + input.group_b.size();
    result.candidate_num += candidate_num;
    ++result.timestep_num;
  }
}

void run_cpu_step(CpuAdapter& adapter, const QueryInput& input,
                  const TestCase& test_case, bool initial_build, int repetition,
                  std::map<BenchmarkKey, BenchmarkResult>& results) {
  adapter.prepare(input);
  double build_ms = measure_wall_time([&adapter]() { adapter.build(); });
  adapter.clear_output();
  static PerfRegionControl perf_region;
  perf_region.enable();
  double query_ms = measure_wall_time([&adapter]() { adapter.query(); });
  perf_region.disable();
  adapter.materialize_output();
  record_measurement(input, test_case, adapter.name(), "cpu", repetition,
                     initial_build, build_ms, query_ms, adapter.output().size(),
                     results);
}

#ifdef SILK_BROADPHASE_HAS_CUDA
double run_gpu_query(GpuAdapter& adapter) {
  adapter.clear_output();
  adapter.synchronize();
  double query_ms = measure_wall_time([&adapter]() {
    adapter.query();
    adapter.synchronize();
  });
  adapter.materialize_output();
  return query_ms;
}

void run_gpu_step(GpuAdapter& adapter, const QueryInput& input,
                  const TestCase& test_case, bool initial_build, int repetition,
                  std::map<BenchmarkKey, BenchmarkResult>& results) {
  adapter.prepare(input);
  adapter.synchronize();
  double build_ms = measure_wall_time([&adapter]() {
    adapter.build();
    adapter.synchronize();
  });
  double query_ms = run_gpu_query(adapter);
  record_measurement(input, test_case, adapter.name(), "gpu", repetition,
                     initial_build, build_ms, query_ms, adapter.output().size(),
                     results);
}
#endif

using CpuAdapterFactory = std::unique_ptr<CpuAdapter> (*)();
constexpr std::array<CpuAdapterFactory, 3> CPU_ADAPTER_FACTORIES = {
    make_silk_kdtree_adapter,
    make_scalable_sap_adapter,
    make_embree_adapter,
};

#ifdef SILK_BROADPHASE_HAS_CUDA
using GpuAdapterFactory = std::unique_ptr<GpuAdapter> (*)();
constexpr std::array<GpuAdapterFactory, 3> GPU_ADAPTER_FACTORIES = {
    make_silk_oibvh_adapter,
    make_scalable_stq_adapter,
    make_cubql_adapter,
};
#endif

bool is_selected(const std::string& name, const std::string& selection) {
  return selection.empty() || name == selection;
}

void benchmark_cpu_adapter(const SceneFiles& scene_files,
                           CpuAdapterFactory factory, int repetitions,
                           std::map<BenchmarkKey, BenchmarkResult>& results) {
  for (QueryKind kind : {QueryKind::EE, QueryKind::VF}) {
    for (int repetition = 0; repetition < repetitions; ++repetition) {
      auto adapter = factory();
      for (int t = 0; t < scene_files.boxes_ee.size(); ++t) {
        TestCase test_case = load_case(scene_files, t);
        QueryInput input = make_query(test_case, kind);
        spdlog::info("[{}] Testing {} time step {} ({}).", adapter->name(),
                     test_case.scene, test_case.timestep, query_name(kind));
        run_cpu_step(*adapter, input, test_case, t == 0, repetition, results);
      }
    }
  }
}

#ifdef SILK_BROADPHASE_HAS_CUDA
void benchmark_gpu_adapter(const SceneFiles& scene_files,
                           GpuAdapterFactory factory, int repetitions,
                           std::map<BenchmarkKey, BenchmarkResult>& results) {
  for (QueryKind kind : {QueryKind::EE, QueryKind::VF}) {
    for (int repetition = 0; repetition < repetitions; ++repetition) {
      auto adapter = factory();
      for (int t = 0; t < scene_files.boxes_ee.size(); ++t) {
        TestCase test_case = load_case(scene_files, t);
        QueryInput input = make_query(test_case, kind);
        spdlog::info("[{}] Testing {} time step {} ({}).", adapter->name(),
                     test_case.scene, test_case.timestep, query_name(kind));
        run_gpu_step(*adapter, input, test_case, t == 0, repetition, results);
      }
    }
  }
}
#endif

void verify_cpu_adapter(const SceneFiles& scene_files,
                        CpuAdapterFactory factory) {
  for (QueryKind kind : {QueryKind::EE, QueryKind::VF}) {
    auto adapter = factory();
    for (int timestep_index = 0; timestep_index < scene_files.boxes_ee.size();
         ++timestep_index) {
      TestCase test_case = load_case(scene_files, timestep_index);
      QueryInput input = make_query(test_case, kind);
      spdlog::info("[{}] Testing {} time step {} ({}).", adapter->name(),
                   test_case.scene, test_case.timestep, query_name(kind));
      adapter->prepare(input);
      adapter->build();
      adapter->clear_output();
      adapter->query();
      adapter->materialize_output();
      spdlog::info("[{}] Verifying {} time step {} ({}).", adapter->name(),
                   test_case.scene, test_case.timestep, query_name(kind));
      verify_output(input, adapter->output(), adapter->name(), test_case);
    }
  }
}

#ifdef SILK_BROADPHASE_HAS_CUDA
void verify_gpu_adapter(const SceneFiles& scene_files,
                        GpuAdapterFactory factory) {
  for (QueryKind kind : {QueryKind::EE, QueryKind::VF}) {
    auto adapter = factory();
    for (int timestep_index = 0; timestep_index < scene_files.boxes_ee.size();
         ++timestep_index) {
      TestCase test_case = load_case(scene_files, timestep_index);
      QueryInput input = make_query(test_case, kind);
      spdlog::info("[{}] Testing {} time step {} ({}).", adapter->name(),
                   test_case.scene, test_case.timestep, query_name(kind));
      adapter->prepare(input);
      adapter->synchronize();
      adapter->build();
      adapter->synchronize();
      adapter->clear_output();
      adapter->synchronize();
      adapter->query();
      adapter->synchronize();
      adapter->materialize_output();
      spdlog::info("[{}] Verifying {} time step {} ({}).", adapter->name(),
                   test_case.scene, test_case.timestep, query_name(kind));
      verify_output(input, adapter->output(), adapter->name(), test_case);
    }
  }
}

void run_gpu_verification_step(GpuAdapter& adapter, const QueryInput& input) {
  adapter.prepare(input);
  adapter.synchronize();
  adapter.build();
  adapter.synchronize();
  adapter.clear_output();
  adapter.synchronize();
  adapter.query();
  adapter.synchronize();
  adapter.materialize_output();
}

void verify_touching_boxes_are_candidates() {
  std::array<Box, 2> boxes = {
      Box{.min = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
          .max = Eigen::Vector3f(1.0f, 1.0f, 1.0f),
          .vertex_ids = {0, 1, -1},
          .id = 0},
      Box{.min = Eigen::Vector3f(1.0f, 0.25f, 0.25f),
          .max = Eigen::Vector3f(2.0f, 0.75f, 0.75f),
          .vertex_ids = {2, 3, -1},
          .id = 1},
  };
  QueryInput input{.kind = QueryKind::EE, .group_a = boxes};
  TestCase test_case{.scene = "touching-boxes", .timestep = 0};
  auto oibvh = make_silk_oibvh_adapter();
  auto cubql = make_cubql_adapter();
  run_gpu_verification_step(*oibvh, input);
  run_gpu_verification_step(*cubql, input);

  const std::vector<Pair> expected = {{0, 1}};
  std::vector<Pair> oibvh_output =
      canonical_output(input, oibvh->output(), oibvh->name(), test_case);
  std::vector<Pair> cubql_output =
      canonical_output(input, cubql->output(), cubql->name(), test_case);
  if (oibvh_output != expected || cubql_output != expected) {
    throw std::runtime_error(
        "Touching boxes were not emitted by both OIBVH and cuBQL");
  }
}

void verify_cubql_matches_oibvh(const SceneFiles& scene_files) {
  for (QueryKind kind : {QueryKind::EE, QueryKind::VF}) {
    auto oibvh = make_silk_oibvh_adapter();
    auto cubql = make_cubql_adapter();
    for (int timestep_index = 0; timestep_index < scene_files.boxes_ee.size();
         ++timestep_index) {
      TestCase test_case = load_case(scene_files, timestep_index);
      QueryInput input = make_query(test_case, kind);
      run_gpu_verification_step(*oibvh, input);
      run_gpu_verification_step(*cubql, input);

      std::vector<Pair> oibvh_output =
          canonical_output(input, oibvh->output(), oibvh->name(), test_case);
      std::vector<Pair> cubql_output =
          canonical_output(input, cubql->output(), cubql->name(), test_case);
      if (oibvh_output != cubql_output) {
        std::vector<Pair> missing;
        std::vector<Pair> extra;
        std::set_difference(oibvh_output.begin(), oibvh_output.end(),
                            cubql_output.begin(), cubql_output.end(),
                            std::back_inserter(missing));
        std::set_difference(cubql_output.begin(), cubql_output.end(),
                            oibvh_output.begin(), oibvh_output.end(),
                            std::back_inserter(extra));
        std::string difference;
        if (!missing.empty()) {
          difference += " missing (" + std::to_string(missing.front().first) +
                        ", " + std::to_string(missing.front().second) + ")";
        }
        if (!extra.empty()) {
          difference += " extra (" + std::to_string(extra.front().first) +
                        ", " + std::to_string(extra.front().second) + ")";
        }
        throw std::runtime_error("cuBQL candidate set differs from OIBVH in " +
                                 test_case.scene + "/" +
                                 std::to_string(test_case.timestep) + " (" +
                                 query_name(kind) + "):" + difference);
      }
    }
  }
  spdlog::info("[cubql] Exact candidate sets match OIBVH for {}.",
               scene_files.name);
}
#endif

void print_result_header(std::ostream& output) {
  output << "algorithm,device,scene,query,threads,timesteps,boxes,"
            "candidates,initial_build_ms,rebuild_ms,"
            "query_ms\n"
         << std::flush;
}

void print_completed_results(
    const std::map<BenchmarkKey, BenchmarkResult>& results,
    const std::string& scene, const std::string& algorithm,
    const std::string& device, int thread_num, std::ostream& output) {
  for (QueryKind kind : {QueryKind::EE, QueryKind::VF}) {
    const BenchmarkKey key{.scene = scene,
                           .query_kind = kind,
                           .algorithm = algorithm,
                           .device = device};
    const BenchmarkResult& result = results.at(key);
    double initial_build_ms = median(result.initial_build_ms);
    double rebuild_ms = median(result.rebuild_ms);
    double query_ms = median(result.query_ms);
    output << key.algorithm << ',' << key.device << ',' << key.scene << ','
           << query_name(key.query_kind) << ','
           << (key.device == "cpu" ? thread_num : 0) << ','
           << result.timestep_num << ',' << result.box_num << ','
           << result.candidate_num << ',' << initial_build_ms << ','
           << rebuild_ms << ',' << query_ms << '\n';
  }
  output << std::flush;
  if (!output) {
    throw std::runtime_error("Unable to write benchmark CSV output");
  }
}

}  // namespace
}  // namespace silk::broadphase_benchmark

int main(int argc, char** argv) {
  using namespace silk::broadphase_benchmark;
  try {
    Options options = parse_options(argc, argv);
    tbb::global_control thread_limit(
        tbb::global_control::max_allowed_parallelism, options.thread_num);
    std::vector<SceneFiles> scenes = resolve_scene_files(options.data_root);
    if (scenes.empty()) {
      throw std::runtime_error("No complete dataset cases found");
    }

    bool selected_scene_found = false;
    bool selected_broadphase_found = false;
    int timestep_num = 0;
    std::ofstream output;
    if (!options.verify) {
      output.open(options.output_path);
      if (!output) {
        throw std::runtime_error("Unable to open benchmark output: " +
                                 options.output_path.string());
      }
      print_result_header(output);
    }
#ifdef SILK_BROADPHASE_HAS_CUDA
    if (options.verify &&
        (options.broadphase.empty() || options.broadphase == "cubql" ||
         options.broadphase == "silk_oibvh")) {
      verify_touching_boxes_are_candidates();
    }
#endif
    for (const SceneFiles& scene_files : scenes) {
      if (!is_selected(scene_files.name, options.scene)) {
        continue;
      }
      selected_scene_found = true;
      timestep_num += scene_files.boxes_ee.size();

      for (CpuAdapterFactory factory : CPU_ADAPTER_FACTORIES) {
        auto adapter = factory();
        if (!is_selected(adapter->name(), options.broadphase)) {
          continue;
        }
        selected_broadphase_found = true;
        if (options.verify) {
          verify_cpu_adapter(scene_files, factory);
        } else {
          std::map<BenchmarkKey, BenchmarkResult> results;
          benchmark_cpu_adapter(scene_files, factory, options.repetitions,
                                results);
          print_completed_results(results, scene_files.name, adapter->name(),
                                  "cpu", options.thread_num, output);
        }
      }
#ifdef SILK_BROADPHASE_HAS_CUDA
      for (GpuAdapterFactory factory : GPU_ADAPTER_FACTORIES) {
        auto adapter = factory();
        if (!is_selected(adapter->name(), options.broadphase)) {
          continue;
        }
        selected_broadphase_found = true;
        if (options.verify) {
          verify_gpu_adapter(scene_files, factory);
        } else {
          std::map<BenchmarkKey, BenchmarkResult> results;
          benchmark_gpu_adapter(scene_files, factory, options.repetitions,
                                results);
          print_completed_results(results, scene_files.name, adapter->name(),
                                  "gpu", options.thread_num, output);
        }
      }
      if (options.verify &&
          (options.broadphase.empty() || options.broadphase == "cubql" ||
           options.broadphase == "silk_oibvh")) {
        verify_cubql_matches_oibvh(scene_files);
      }
#endif
    }

    if (!selected_scene_found) {
      throw std::runtime_error("Unknown scene: " + options.scene);
    }
    if (!selected_broadphase_found) {
      throw std::runtime_error("Unknown broadphase: " + options.broadphase);
    }

    if (options.verify) {
      spdlog::info("Verified {} selected frame transitions.", timestep_num);
    }
  } catch (const std::exception& error) {
    spdlog::error("broadphase_benchmark: {}", error.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
