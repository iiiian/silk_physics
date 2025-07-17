#pragma once

#include <Eigen/Core>
#include <filesystem>
#include <string>
#include <vector>

enum class QueryType { EdgeEdge, PointTriangle };

struct Query {
  QueryType type;
  bool result;
  std::string source;
  int source_line;

  // edge edge case
  //     edge 1 = v1 v2, edge 2 = v3 v4
  // point triangle case
  //     triangle = v1 v2 v3, point = v4
  // suffix 0 = at t0, suffix 1 = at t1
  Eigen::Vector3d v10, v20, v30, v40;
  Eigen::Vector3d v11, v21, v31, v41;

  std::string to_string() const;
};

std::vector<Query> parse_queries_from_csv(const std::filesystem::path &path,
                                          QueryType type);

struct QueryCategory {
  std::string name;
  std::vector<Query> edge_edge;
  std::vector<Query> point_triangle;

  QueryCategory(const std::filesystem::path &path);
};
