#include "ccd.hpp"

#include <Eigen/Core>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
namespace eg = Eigen;

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
  eg::Vector3d v10, v20, v30, v40;
  eg::Vector3d v11, v21, v31, v41;

  std::string to_string() const {
    std::stringstream ss;
    std::string type_str = (type == QueryType::EdgeEdge)
                               ? "Edge edge query: "
                               : "Point triangle query: ";
    std::string result_str = (result) ? "true" : "false";
    ss << type_str << "result = " << result_str << "\n";
    ss << "    line " << source_line << " from " << source << "\n";
    if (type == QueryType::EdgeEdge) {
      ss << "    v10: " << v10.transpose() << "\n";
      ss << "    v20: " << v20.transpose() << "\n";
      ss << "    v30: " << v30.transpose() << "\n";
      ss << "    v40: " << v40.transpose() << "\n";
      ss << "    v11: " << v11.transpose() << "\n";
      ss << "    v21: " << v21.transpose() << "\n";
      ss << "    v31: " << v31.transpose() << "\n";
      ss << "    v41: " << v41.transpose() << "\n";
    } else {
      ss << "    v10: " << v10.transpose() << "\n";
      ss << "    v20: " << v20.transpose() << "\n";
      ss << "    v30: " << v30.transpose() << "\n";
      ss << "    p0 : " << v40.transpose() << "\n";
      ss << "    v11: " << v11.transpose() << "\n";
      ss << "    v21: " << v21.transpose() << "\n";
      ss << "    v31: " << v31.transpose() << "\n";
      ss << "    p1 : " << v41.transpose() << "\n";
    }
    return ss.str();
  }
};

/// split a line on commas
std::vector<std::string> split(const std::string &s, char delim = ',') {
  std::vector<std::string> elems;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

/// parse "numerator","denominator" into a double
double parse_rational(const std::string &num_str, const std::string &den_str) {
  // these values fit in signed 64-bit
  double num, den;
  try {
    num = std::stod(num_str);
    den = std::stod(den_str);
    if (den == 0) {
      throw std::runtime_error("zero denominator in CSV");
    }
  } catch (const std::exception &e) {
    throw std::runtime_error("fail to parse rational: num = " + num_str +
                             " den = " + den_str);
  }
  return num / den;
}

std::vector<Query> parse_queries_from_csv(const fs::path &path,
                                          QueryType type) {
  std::ifstream in{path};
  if (!in) {
    throw std::runtime_error("cannot open " + path.string());
  }

  std::vector<Query> out;
  std::string line;
  int line_count = 0;
  std::array<eg::Vector3d, 8> buf;
  bool expected_res = false;
  int row_in_group = 0;
  int start_line = 0;

  while (std::getline(in, line)) {
    line_count++;

    if (line.empty()) {
      continue;
    }
    auto f = split(line, ',');
    if (f.size() != 7) {
      throw std::runtime_error("bad column count in CSV line");
    }

    double x = parse_rational(f[0], f[1]);
    double y = parse_rational(f[2], f[3]);
    double z = parse_rational(f[4], f[5]);
    bool res = (f[6] == "1");

    if (row_in_group == 0) {
      expected_res = res;
      start_line = line_count;
    }

    buf[row_in_group] = eg::Vector3d{x, y, z};
    ++row_in_group;

    if (row_in_group == 8) {
      Query q;
      q.type = type;
      q.result = expected_res;
      q.source = path;
      q.source_line = start_line;

      if (type == QueryType::EdgeEdge) {
        q.v10 = buf[0];
        q.v20 = buf[1];
        q.v30 = buf[2];
        q.v40 = buf[3];
        q.v11 = buf[4];
        q.v21 = buf[5];
        q.v31 = buf[6];
        q.v41 = buf[7];
      } else {
        q.v40 = buf[0];
        q.v10 = buf[1];
        q.v20 = buf[2];
        q.v30 = buf[3];
        q.v41 = buf[4];
        q.v11 = buf[5];
        q.v21 = buf[6];
        q.v31 = buf[7];
      }

      out.push_back(q);
      row_in_group = 0;
    }
  }

  if (row_in_group != 0) {
    throw std::runtime_error("trailing incomplete query group");
  }
  return out;
}

template <typename T>
void vector_append(std::vector<T> &main, const std::vector<T> &append) {
  main.reserve(main.size() + append.size());
  main.insert(main.end(), append.begin(), append.end());
}

std::string ccd_solver_to_string(const CCDSolver &solver) {
  std::stringstream ss;
  ss << "CCD solver:\n";
  ss << "    eps = " << solver.eps << "\n";
  ss << "    h = " << solver.h << "\n";
  ss << "    tol = " << solver.tol << "\n";
  ss << "    max iter = " << solver.max_iter << "\n";
  return ss.str();
}

struct QueryCategory {
  std::string name;
  std::vector<Query> edge_edge;
  std::vector<Query> point_triangle;

  QueryCategory(const fs::path &path) {
    if (!fs::is_directory(path)) {
      throw std::runtime_error("path " + path.string() + " is not a directory");
    }

    fs::path edge_edge_dir = path / "edge-edge";
    fs::path point_triangle_dir = path / "vertex-face";
    if (!(fs::is_directory(edge_edge_dir) &&
          fs::is_directory(point_triangle_dir))) {
      throw std::runtime_error(
          "path " + path.string() +
          " is missing edge-edge or vertex-face sub directory");
    }

    this->name = path.filename();
    // edge edge queries
    for (const auto &entry : fs::directory_iterator{edge_edge_dir}) {
      auto entry_path = entry.path();
      if (entry.is_regular_file() && entry_path.extension() == ".csv") {
        vector_append(this->edge_edge,
                      parse_queries_from_csv(entry_path, QueryType::EdgeEdge));
      }
    }
    // point triangle queries
    for (const auto &entry : fs::directory_iterator{point_triangle_dir}) {
      auto entry_path = entry.path();
      if (entry.is_regular_file() && entry_path.extension() == ".csv") {
        vector_append(
            this->point_triangle,
            parse_queries_from_csv(entry_path, QueryType::PointTriangle));
      }
    }
  }
};

void test_query_category(const fs::path &root, const std::string &name) {
  QueryCategory category{root / name};
  CCDSolver solver;
  solver.tol = 0.1;
  solver.eps = 1e-6;
  // solver.h = 0.001;
  solver.max_iter = 10;

  for (const auto &q : category.edge_edge) {
    INFO("Category: " << name << "\n"
                      << ccd_solver_to_string(solver) << q.to_string());

    double dist2_a = (q.v20 - q.v10).squaredNorm();
    double dist2_b = (q.v40 - q.v30).squaredNorm();
    double dist2 = std::min(dist2_a, dist2_b);
    solver.h = 0.01 * std::sqrt(dist2);

    CHECK(solver.edge_edge_ccd(
              q.v10.cast<float>(), q.v20.cast<float>(), q.v30.cast<float>(),
              q.v40.cast<float>(), q.v11.cast<float>(), q.v21.cast<float>(),
              q.v31.cast<float>(), q.v41.cast<float>(), 0.0, 1.0) == q.result);
  }

  for (const auto &q : category.point_triangle) {
    INFO("Category: " << name << "\n"
                      << ccd_solver_to_string(solver) << q.to_string());

    double dist2_a = (q.v20 - q.v10).squaredNorm();
    double dist2_b = (q.v30 - q.v20).squaredNorm();
    double dist2_c = (q.v10 - q.v30).squaredNorm();
    double dist2 = std::min(std::min(dist2_a, dist2_b), dist2_c);
    solver.h = 0.01 * std::sqrt(dist2);
    CHECK(solver.point_triangle_ccd(
              q.v40.cast<float>(), q.v10.cast<float>(), q.v20.cast<float>(),
              q.v30.cast<float>(), q.v41.cast<float>(), q.v11.cast<float>(),
              q.v21.cast<float>(), q.v31.cast<float>(), 0.0, 1.0) == q.result);
  }
}

TEST_CASE("ccd-tests", "[ccd]") {
  fs::path root{SAMPLE_QUERY_ROOT};

  SECTION("chain") { test_query_category(root, "chain"); }
  SECTION("cow-heads") { test_query_category(root, "cow-heads"); }
  // SECTION("erleben-cube-cliff-edges") {
  //   test_query_category(root, "erleben-cube-cliff-edges");
  // }
  // SECTION("erleben-cube-internal-edges") {
  //   test_query_category(root, "erleben-cube-internal-edges");
  // }
  // SECTION("erleben-sliding-spike") {
  //   test_query_category(root, "erleben-sliding-spike");
  // }
  // SECTION("erleben-sliding-wedge") {
  //   test_query_category(root, "erleben-sliding-wedge");
  // }
  // SECTION("erleben-spike-crack") {
  //   test_query_category(root, "erleben-spike-crack");
  // }
  // SECTION("erleben-spike-hole") {
  //   test_query_category(root, "erleben-spike-hole");
  // }
  // SECTION("erleben-spikes") { test_query_category(root, "erleben-spikes"); }
  // SECTION("erleben-spike-wedge") {
  //   test_query_category(root, "erleben-spike-wedge");
  // }
  // SECTION("erleben-wedge-crack") {
  //   test_query_category(root, "erleben-wedge-crack");
  // }
  // SECTION("erleben-wedges") { test_query_category(root, "erleben-wedges"); }
  SECTION("golf-ball") { test_query_category(root, "golf-ball"); }
  SECTION("mat-twist") { test_query_category(root, "mat-twist"); }
  SECTION("unit-tests") { test_query_category(root, "unit-tests"); }
  SECTION("fail") { test_query_category(root, "fail"); }
}
