#pragma once

#include <cuda_runtime.h>

#include <utility>

#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

class CudaGraph {
 private:
  cudaGraph_t graph_ = nullptr;
  cudaGraphExec_t graph_exec_ = nullptr;

 public:
  CudaGraph() { check_cuda(cudaGraphCreate(&graph_, 0)); }

  ~CudaGraph() { reset(); }

  CudaGraph(const CudaGraph&) = delete;
  CudaGraph& operator=(const CudaGraph&) = delete;

  CudaGraph(CudaGraph&& other) noexcept
      : graph_(std::exchange(other.graph_, nullptr)),
        graph_exec_(std::exchange(other.graph_exec_, nullptr)) {}

  CudaGraph& operator=(CudaGraph&& other) noexcept {
    reset();
    graph_ = std::exchange(other.graph_, nullptr);
    graph_exec_ = std::exchange(other.graph_exec_, nullptr);
    return *this;
  }

  void reset() {
    if (graph_exec_) {
      cudaGraphExecDestroy(graph_exec_);
    }
    if (graph_) {
      cudaGraphDestroy(graph_);
    }
    graph_exec_ = nullptr;
    graph_ = nullptr;
  }

  cudaGraph_t raw() const { return graph_; }

  void update(CudaGraph&& graph) {
    if (graph_exec_) {
      cudaGraphExecUpdateResultInfo result_info{};
      check_cuda(cudaGraphExecUpdate(graph_exec_, graph.graph_, &result_info));
    } else {
      check_cuda(cudaGraphInstantiate(&graph_exec_, graph.graph_, nullptr,
                                      nullptr, 0));
    }

    check_cuda(cudaGraphDestroy(graph_));
    graph_ = std::exchange(graph.graph_, nullptr);
  }

  void launch(CudaRuntime rt) {
    check_cuda(cudaGraphLaunch(graph_exec_, rt.stream.get()));
  }
};
}  // namespace silk::cuda
