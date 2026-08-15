#include <spdlog/spdlog.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <argparse/argparse.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <ostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "../broadphase/cpu_adapters.hpp"
#include "common/perf_region_control.hpp"
#include "narrowphase_common.hpp"

namespace silk::narrowphase_benchmark {

namespace {

constexpr float TOI_TOLERANCE = 1e-6f;
constexpr int VERIFY_MAX_ITERATIONS = -1;
constexpr int BENCHMARK_MAX_ITERATIONS = 1'000'000;

struct Options {
  bool verify = false;
  int repetitions = 3;
  int thread_num = std::max(1U, std::thread::hardware_concurrency());
  int timestep_num = -1;
  int max_iterations = BENCHMARK_MAX_ITERATIONS;
  std::filesystem::path data_root = SILK_NARROWPHASE_DATA_ROOT;
  std::filesystem::path output_path;
  std::string scene;
  std::string query;
  std::string ccd;
};

struct OutputSummary {
  uint64_t expected_collision_num = 0;
  uint64_t reported_collision_num = 0;
  uint64_t pruned_query_num = 0;
  uint64_t false_negative_num = 0;
  uint64_t false_positive_num = 0;
  uint64_t toi_mismatch_num = 0;
  float max_toi_error = 0.0f;
};

struct CpuRun {
  std::unique_ptr<CpuAdapter> adapter;
  OutputSummary summary;
  std::vector<double> times;
  uint64_t query_count = 0;
  uint64_t analytical_query_count = 0;
};

#ifdef SILK_NARROWPHASE_HAS_CUDA
struct GpuRun {
  std::unique_ptr<GpuAdapter> adapter;
  OutputSummary summary;
  std::vector<double> times;
  uint64_t query_count = 0;
  uint64_t analytical_query_count = 0;
};
#endif

Options parse_options(int argc, char** argv) {
  Options options;
  argparse::ArgumentParser program("narrowphase_benchmark");
  program.add_description(
      "Verify and benchmark narrowphase CCD on Scalable CCD data");
  program.add_argument("--verify")
      .help("Compare against original_ticcd and analytical dataset labels")
      .store_into(options.verify);
  program.add_argument("--repetitions")
      .help("Number of complete query sweeps")
      .store_into(options.repetitions);
  program.add_argument("--threads")
      .help("Maximum CPU thread count")
      .store_into(options.thread_num);
  program.add_argument("--timesteps")
      .help("Maximum number of timesteps per scene")
      .store_into(options.timestep_num);
  program.add_argument("--max-iterations")
      .help("Maximum root-finder iterations per query; -1 means unlimited")
      .store_into(options.max_iterations);
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
  program.add_argument("--query")
      .help("Run only one query kind: ee or vf")
      .store_into(options.query);
  program.add_argument("--ccd")
      .help(
          "Run only one implementation: silk_ticcd, original_ticcd, "
          "scalable_ccd, or silk_hybrid_ticcd")
      .store_into(options.ccd);
  program.parse_args(argc, argv);

  if (options.verify && !program.is_used("--max-iterations")) {
    options.max_iterations = VERIFY_MAX_ITERATIONS;
  }

  if (options.repetitions <= 0 || options.thread_num <= 0 ||
      options.timestep_num == 0 || options.timestep_num < -1) {
    throw std::runtime_error(
        "Repetitions and thread count must be positive; timesteps must be -1 "
        "or positive");
  }
  if (options.max_iterations == 0 || options.max_iterations < -1) {
    throw std::runtime_error(
        "Maximum iterations must be -1 or a positive integer");
  }
  if (!options.verify && options.output_path.empty()) {
    throw std::runtime_error("--out is required in benchmark mode");
  }
  if (!options.query.empty() && options.query != "ee" &&
      options.query != "vf") {
    throw std::runtime_error("Unknown query kind: " + options.query);
  }
  return options;
}

bool is_selected(const std::string& name, const std::string& selection) {
  return selection.empty() || name == selection;
}

double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  return values[values.size() / 2];
}

template <typename Function>
double measure_wall_time(Function&& function) {
  using Clock = std::chrono::steady_clock;
  auto start = Clock::now();
  function();
  auto done = Clock::now();
  return std::chrono::duration<double, std::milli>(done - start).count();
}

template <typename Function>
void for_each_timestep(const QueryBatch& batch, Function&& function) {
  int begin = 0;
  while (begin < batch.queries.size()) {
    int end = begin + 1;
    while (end < batch.queries.size() &&
           batch.queries[end].timestep == batch.queries[begin].timestep) {
      ++end;
    }
    function(begin, end);
    begin = end;
  }
}

OutputSummary verify_output(const QueryBatch& batch,
                            std::span<const QueryOutput> output) {
  if (output.size() != batch.queries.size()) {
    throw std::runtime_error(
        "CCD implementation returned an invalid output size");
  }

  OutputSummary summary;
  for (int i = 0; i < batch.queries.size(); ++i) {
    const Query& query = batch.queries[i];
    summary.reported_collision_num += output[i].hit;
    summary.pruned_query_num += !output[i].searched_full_interval;
    if (!query.expected_known) {
      continue;
    }
    summary.expected_collision_num += query.expected;
    summary.false_negative_num +=
        query.expected && !output[i].hit && output[i].searched_full_interval;
    summary.false_positive_num += !query.expected && output[i].hit;
  }
  return summary;
}

uint64_t verify_analytical_output(const QueryBatch& batch,
                                  std::span<const QueryOutput> output,
                                  const std::string& implementation) {
  if (output.size() != batch.queries.size()) {
    throw std::runtime_error(
        "CCD implementation returned an invalid output size");
  }

  uint64_t analytical_query_count = 0;
  for (int i = 0; i < batch.queries.size(); ++i) {
    const Query& query = batch.queries[i];
    if (!query.expected_known) {
      continue;
    }
    ++analytical_query_count;
    if (query.expected && !output[i].hit) {
      throw std::runtime_error(
          implementation +
          " returned a false negative against the analytical dataset in " +
          batch.scene + "/" + query_name(batch.kind) + " at timestep " +
          std::to_string(query.timestep) + ", query " +
          std::to_string(query.timestep_query));
    }
  }
  return analytical_query_count;
}

void accumulate_summary(OutputSummary& total, const OutputSummary& value) {
  total.expected_collision_num += value.expected_collision_num;
  total.reported_collision_num += value.reported_collision_num;
  total.pruned_query_num += value.pruned_query_num;
  total.false_negative_num += value.false_negative_num;
  total.false_positive_num += value.false_positive_num;
  total.toi_mismatch_num += value.toi_mismatch_num;
  total.max_toi_error = std::max(total.max_toi_error, value.max_toi_error);
}

OutputSummary compare_to_reference(
    const QueryBatch& batch, std::span<const QueryOutput> output,
    std::span<const QueryOutput> reference_output) {
  if (output.size() != batch.queries.size() ||
      reference_output.size() != batch.queries.size()) {
    throw std::runtime_error(
        "CCD implementation returned an invalid output size");
  }

  OutputSummary summary;
  for (int i = 0; i < batch.queries.size(); ++i) {
    bool reference_hit = reference_output[i].hit;
    bool output_hit = output[i].hit;
    summary.expected_collision_num += reference_hit;
    summary.reported_collision_num += output_hit;
    summary.false_negative_num += reference_hit && !output_hit;
    summary.false_positive_num += !reference_hit && output_hit;
    if (reference_hit && output_hit) {
      float toi_error = std::abs(reference_output[i].toi - output[i].toi);
      summary.max_toi_error = std::max(summary.max_toi_error, toi_error);
      summary.toi_mismatch_num += toi_error > TOI_TOLERANCE;
    }
  }
  return summary;
}

std::vector<QueryOutput> run_cpu(CpuAdapter& adapter, const QueryBatch& batch,
                                 bool verify, int repetitions,
                                 std::vector<double>& times) {
  std::vector<QueryOutput> output(batch.queries.size());
  int run_num = verify ? 1 : repetitions;
  if (times.empty()) {
    times.resize(run_num);
  }
  for (int repetition = 0; repetition < run_num; ++repetition) {
    PerfRegionControl perf_region;
    perf_region.enable();
    double time_ms = 0;
    for_each_timestep(batch, [&](int begin, int end) {
      std::atomic<float> earliest_toi{1.0f};
      time_ms += measure_wall_time([&]() {
        tbb::parallel_for(begin, end, [&](int i) {
          float max_time =
              verify ? 1.0f : earliest_toi.load(std::memory_order_relaxed);
          if (max_time <= 0.0f) {
            output[i] = {
                .toi = 1.0f, .hit = false, .searched_full_interval = false};
            return;
          }

          QueryOutput result = adapter.query(batch.queries[i], max_time);
          result.searched_full_interval = max_time == 1.0f;
          output[i] = result;

          if (!verify && result.hit) {
            float previous = earliest_toi.load(std::memory_order_relaxed);
            while (result.toi < previous &&
                   !earliest_toi.compare_exchange_weak(
                       previous, result.toi, std::memory_order_relaxed)) {
            }
          }
        });
      });
    });
    perf_region.disable();
    if (!verify) {
      times[repetition] += time_ms;
    }
  }
  return output;
}

#ifdef SILK_NARROWPHASE_HAS_CUDA
std::vector<QueryOutput> run_gpu(GpuAdapter& adapter, const QueryBatch& batch,
                                 bool verify, int repetitions,
                                 std::vector<double>& times) {
  std::vector<QueryOutput> output(batch.queries.size());
  int run_num = verify ? 1 : repetitions;
  if (times.empty()) {
    times.resize(run_num);
  }
  for (int repetition = 0; repetition < run_num; ++repetition) {
    double time_ms = 0;
    for_each_timestep(batch, [&](int begin, int end) {
      adapter.prepare(std::span(batch.queries).subspan(begin, end - begin));
      adapter.synchronize();
      time_ms += measure_wall_time([&]() {
        adapter.query();
        adapter.synchronize();
      });
      std::ranges::copy(adapter.output(), output.begin() + begin);
    });
    if (!verify) {
      times[repetition] += time_ms;
    }
  }
  return output;
}
#endif

void print_header(std::ostream& output) {
  output << "algorithm,device,scene,query,threads,timesteps,queries,"
            "pruned_queries,expected_collisions,reported_collisions,"
            "false_positives,false_negatives,query_ms,ns_per_query\n"
         << std::flush;
}

void print_result(const std::string& scene, QueryKind kind, int timestep_num,
                  uint64_t query_count, const std::string& algorithm,
                  const std::string& device, int thread_num,
                  const OutputSummary& summary,
                  const std::vector<double>& times, std::ostream& output) {
  double query_ms = median(times);
  double ns_per_query = query_ms * 1e6 / std::max<uint64_t>(query_count, 1);
  int reported_thread_num = device == "gpu" ? 0 : thread_num;
  output << algorithm << ',' << device << ',' << scene << ','
         << query_name(kind) << ',' << reported_thread_num << ','
         << timestep_num << ',' << query_count << ','
         << summary.pruned_query_num << ',' << summary.expected_collision_num
         << ',' << summary.reported_collision_num << ','
         << summary.false_positive_num << ',' << summary.false_negative_num
         << ',' << query_ms << ',' << ns_per_query << '\n'
         << std::flush;
  if (!output) {
    throw std::runtime_error("Unable to write benchmark CSV output");
  }
}

}  // namespace
}  // namespace silk::narrowphase_benchmark

int main(int argc, char** argv) {
  using namespace silk::narrowphase_benchmark;
  try {
    Options options = parse_options(argc, argv);
    tbb::global_control thread_limit(
        tbb::global_control::max_allowed_parallelism, options.thread_num);
    std::vector<SceneFiles> scenes = resolve_scene_files(options.data_root);

    std::ofstream output;
    if (!options.verify) {
      output.open(options.output_path);
      if (!output) {
        throw std::runtime_error("Unable to open benchmark output: " +
                                 options.output_path.string());
      }
      print_header(output);
    }

    bool selected_scene_found = false;
    bool selected_ccd_found = false;
    uint64_t verified_query_num = 0;
    int max_iterations = options.max_iterations;
    for (const SceneFiles& scene : scenes) {
      if (!is_selected(scene.name, options.scene)) {
        continue;
      }
      selected_scene_found = true;
      SceneBounds scene_bounds = load_scene_bounds(scene);
      for (QueryKind kind : {QueryKind::EE, QueryKind::VF}) {
        if (!is_selected(query_name(kind), options.query)) {
          continue;
        }
        std::vector<CpuRun> cpu_runs;
        for (auto factory :
             {make_silk_ticcd_adapter, make_original_ticcd_adapter}) {
          std::unique_ptr<CpuAdapter> adapter = factory(kind, max_iterations);
          if (!is_selected(adapter->name(), options.ccd)) {
            continue;
          }
          selected_ccd_found = true;
          cpu_runs.push_back(CpuRun{.adapter = std::move(adapter)});
        }
        std::unique_ptr<CpuAdapter> reference_adapter;
        std::vector<double> reference_times;
        if (options.verify) {
          reference_adapter = make_original_ticcd_adapter(kind, max_iterations);
        }

#ifdef SILK_NARROWPHASE_HAS_CUDA
        std::vector<GpuRun> gpu_runs;
        for (auto factory :
             {make_scalable_gpu_adapter, make_silk_hybrid_ticcd_adapter}) {
          std::unique_ptr<GpuAdapter> adapter = factory(kind, max_iterations);
          if (!is_selected(adapter->name(), options.ccd)) {
            continue;
          }
          selected_ccd_found = true;
          gpu_runs.push_back(GpuRun{.adapter = std::move(adapter)});
        }
#endif

        if (cpu_runs.empty()
#ifdef SILK_NARROWPHASE_HAS_CUDA
            && gpu_runs.empty()
#endif
        ) {
          continue;
        }

        std::unique_ptr<silk::broadphase_benchmark::CpuAdapter> broadphase;
        if (!options.verify) {
          broadphase = silk::broadphase_benchmark::make_embree_adapter();
        }
        uint64_t generated_query_num = 0;
        int timestep_num = scene.boxes_ee.size();
        if (options.timestep_num > 0) {
          timestep_num = std::min(timestep_num, options.timestep_num);
        }
        for (int timestep_index = 0; timestep_index < timestep_num;
             ++timestep_index) {
          int timestep = scene.first_timestep + timestep_index;
          // The published n-body archive omits these ground-truth queries.
          if (scene.name == "n-body-simulation" &&
              ((kind == QueryKind::VF && timestep == 53) || (timestep == 56))) {
            continue;
          }
          QueryBatch batch = load_batch(scene, kind, scene_bounds,
                                        timestep_index, broadphase.get());
          generated_query_num += batch.queries.size();
          std::vector<QueryOutput> reference_output;
          if (reference_adapter) {
            reference_output =
                run_cpu(*reference_adapter, batch, true, 1, reference_times);
            verify_analytical_output(batch, reference_output,
                                     reference_adapter->name());
          }
          for (CpuRun& run : cpu_runs) {
            std::vector<QueryOutput> query_output =
                run_cpu(*run.adapter, batch, options.verify,
                        options.repetitions, run.times);
            if (options.verify) {
              run.analytical_query_count += verify_analytical_output(
                  batch, query_output, run.adapter->name());
              accumulate_summary(
                  run.summary,
                  compare_to_reference(batch, query_output, reference_output));
            } else {
              accumulate_summary(run.summary,
                                 verify_output(batch, query_output));
            }
            run.query_count += batch.queries.size();
          }

#ifdef SILK_NARROWPHASE_HAS_CUDA
          for (GpuRun& run : gpu_runs) {
            std::vector<QueryOutput> query_output =
                run_gpu(*run.adapter, batch, options.verify,
                        options.repetitions, run.times);
            if (options.verify) {
              run.analytical_query_count += verify_analytical_output(
                  batch, query_output, run.adapter->name());
              accumulate_summary(
                  run.summary,
                  compare_to_reference(batch, query_output, reference_output));
            } else {
              accumulate_summary(run.summary,
                                 verify_output(batch, query_output));
            }
            run.query_count += batch.queries.size();
          }
#endif
          if (timestep_index % 25 == 0 || timestep_index + 1 == timestep_num) {
            if (options.verify) {
              spdlog::info(
                  "[dataset] Loaded {} {} queries through timestep {}/{} for "
                  "{}.",
                  generated_query_num, query_name(kind), timestep_index + 1,
                  timestep_num, scene.name);
            } else {
              spdlog::info(
                  "[embree] Generated {} {} queries through timestep {}/{} "
                  "for {}.",
                  generated_query_num, query_name(kind), timestep_index + 1,
                  timestep_num, scene.name);
            }
          }
        }

        for (CpuRun& run : cpu_runs) {
          spdlog::info("[{}] Processed {} {} queries for {}.",
                       run.adapter->name(), run.query_count, query_name(kind),
                       scene.name);
          bool output_mismatch = run.summary.false_negative_num != 0 ||
                                 run.summary.false_positive_num != 0 ||
                                 run.summary.toi_mismatch_num != 0;
          if (options.verify && output_mismatch) {
            throw std::runtime_error(
                run.adapter->name() + " differs from original_ticcd: " +
                std::to_string(run.summary.false_negative_num) +
                " false negatives, " +
                std::to_string(run.summary.false_positive_num) +
                " false positives, " +
                std::to_string(run.summary.toi_mismatch_num) +
                " TOI mismatches, maximum TOI error " +
                std::to_string(run.summary.max_toi_error));
          }
          if (!options.verify && run.summary.false_negative_num != 0) {
            throw std::runtime_error(
                run.adapter->name() + " returned " +
                std::to_string(run.summary.false_negative_num) +
                " false negatives in " + scene.name + "/" + query_name(kind));
          }
          if (options.verify) {
            spdlog::info("[{}] Checked {} analytical dataset labels.",
                         run.adapter->name(), run.analytical_query_count);
            spdlog::info(
                "[{}] Compared against original_ticcd: {} hit mismatches, "
                "{} TOI mismatches, maximum TOI error {}.",
                run.adapter->name(),
                run.summary.false_negative_num + run.summary.false_positive_num,
                run.summary.toi_mismatch_num, run.summary.max_toi_error);
          }
          if (!options.verify) {
            print_result(scene.name, kind, timestep_num, run.query_count,
                         run.adapter->name(), "cpu", options.thread_num,
                         run.summary, run.times, output);
          }
          verified_query_num += run.query_count;
        }

#ifdef SILK_NARROWPHASE_HAS_CUDA
        for (GpuRun& run : gpu_runs) {
          spdlog::info("[{}] Processed {} {} queries for {}.",
                       run.adapter->name(), run.query_count, query_name(kind),
                       scene.name);
          bool output_mismatch = run.summary.false_negative_num != 0 ||
                                 run.summary.false_positive_num != 0 ||
                                 run.summary.toi_mismatch_num != 0;
          if (options.verify && output_mismatch) {
            throw std::runtime_error(
                run.adapter->name() + " differs from original_ticcd: " +
                std::to_string(run.summary.false_negative_num) +
                " false negatives, " +
                std::to_string(run.summary.false_positive_num) +
                " false positives, " +
                std::to_string(run.summary.toi_mismatch_num) +
                " TOI mismatches, maximum TOI error " +
                std::to_string(run.summary.max_toi_error));
          }
          if (!options.verify && run.summary.false_negative_num != 0) {
            throw std::runtime_error(
                run.adapter->name() + " returned " +
                std::to_string(run.summary.false_negative_num) +
                " false negatives in " + scene.name + "/" + query_name(kind));
          }
          if (options.verify) {
            spdlog::info("[{}] Checked {} analytical dataset labels.",
                         run.adapter->name(), run.analytical_query_count);
            spdlog::info(
                "[{}] Compared against original_ticcd: {} hit mismatches, "
                "{} TOI mismatches, maximum TOI error {}.",
                run.adapter->name(),
                run.summary.false_negative_num + run.summary.false_positive_num,
                run.summary.toi_mismatch_num, run.summary.max_toi_error);
          }
          if (!options.verify) {
            std::string device =
                run.adapter->name() == "silk_hybrid_ticcd" ? "hybrid" : "gpu";
            print_result(scene.name, kind, timestep_num, run.query_count,
                         run.adapter->name(), device, options.thread_num,
                         run.summary, run.times, output);
          }
          verified_query_num += run.query_count;
        }
#endif
      }
    }

    if (!selected_scene_found) {
      throw std::runtime_error("Unknown scene: " + options.scene);
    }
    if (!selected_ccd_found) {
      throw std::runtime_error("Unknown CCD implementation: " + options.ccd);
    }
    if (options.verify) {
      spdlog::info("Verified {} selected queries.", verified_query_num);
    }
  } catch (const std::exception& error) {
    spdlog::error("narrowphase_benchmark: {}", error.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
