#include "backend/cuda/graph_partition.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cuda/std/span>
#include <vector>

using silk::cuda::graph_partition;

namespace {

// Check
// - No empty partition.
// - Partition ID is in range.
// - Partition size within (0, max_part_size].
void check_partition_invariants(int node_num, int max_part_size, int part_num,
                                const std::vector<int> &part_id) {
  REQUIRE(static_cast<int>(part_id.size()) == node_num);

  std::vector<int> part_size(part_num, 0);
  for (int i = 0; i < node_num; ++i) {
    REQUIRE(part_id[i] >= 0);
    REQUIRE(part_id[i] < part_num);
    part_size[part_id[i]] += 1;
  }

  for (int p = 0; p < part_num; ++p) {
    REQUIRE(part_size[p] > 0);
    REQUIRE(part_size[p] <= max_part_size);
  }
}

}  // namespace

TEST_CASE("graph_partition: trivial case fits in one partition",
          "[graph_partition]") {
  int max_part_size = 32;

  std::vector<int> row_ptr = {0};
  std::vector<int> cols;
  std::vector<int> weights;

  int part_num = 0;
  std::vector<int> part_id;

  graph_partition(row_ptr, cols, weights, max_part_size, part_num, part_id);
  REQUIRE(part_num == 1);
  REQUIRE(part_id.empty());
}

TEST_CASE("graph_partition: small chain graph", "[graph_partition]") {
  int max_part_size = 32;

  // Chain graph: 0-1-2-3-4-5-6-7-8-9
  int node_num = 10;
  std::vector<int> row_ptr(node_num + 1);
  std::vector<int> cols;
  std::vector<int> weights;

  for (int i = 0; i < node_num; ++i) {
    row_ptr[i] = static_cast<int>(cols.size());
    if (i > 0) {
      cols.push_back(i - 1);
    }
    if (i < node_num - 1) {
      cols.push_back(i + 1);
    }
  }
  row_ptr[node_num] = static_cast<int>(cols.size());

  int part_num = 0;
  std::vector<int> part_id;

  graph_partition(row_ptr, cols, weights, max_part_size, part_num, part_id);
  check_partition_invariants(node_num, max_part_size, part_num, part_id);
}

TEST_CASE("graph_partition: chain graph requiring multiple partitions",
          "[graph_partition]") {
  int max_part_size = 3;

  // Chain graph: 0-1-2-3-4-5-6-7-8-9
  int node_num = 10;
  std::vector<int> row_ptr(node_num + 1);
  std::vector<int> cols;
  std::vector<int> weights;

  for (int i = 0; i < node_num; ++i) {
    row_ptr[i] = static_cast<int>(cols.size());
    if (i > 0) {
      cols.push_back(i - 1);
    }
    if (i < node_num - 1) {
      cols.push_back(i + 1);
    }
  }
  row_ptr[node_num] = static_cast<int>(cols.size());

  int part_num = 0;
  std::vector<int> part_id;

  graph_partition(row_ptr, cols, weights, max_part_size, part_num, part_id);
  check_partition_invariants(node_num, max_part_size, part_num, part_id);
  REQUIRE(part_num > 1);
}

TEST_CASE("graph_partition: chain graph with weighted edges",
          "[graph_partition]") {
  int max_part_size = 4;

  int node_num = 20;
  std::vector<int> row_ptr(node_num + 1);
  std::vector<int> cols;
  std::vector<int> weights;

  for (int i = 0; i < node_num; ++i) {
    row_ptr[i] = static_cast<int>(cols.size());
    if (i > 0) {
      cols.push_back(i - 1);
      weights.push_back(1);
    }
    if (i < node_num - 1) {
      cols.push_back(i + 1);
      weights.push_back(1);
    }
  }
  row_ptr[node_num] = static_cast<int>(cols.size());

  int part_num = 0;
  std::vector<int> part_id;

  graph_partition(row_ptr, cols, weights, max_part_size, part_num, part_id);
  check_partition_invariants(node_num, max_part_size, part_num, part_id);
}
