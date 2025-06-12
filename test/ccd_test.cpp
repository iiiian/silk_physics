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

struct EdgeEdgeQuery {
  // edge 1 = v1 v2, edge 2 = v3 v4
  // suffix 0 = at t0, suffix 1 = at t1
  eg::Vector3d v10, v20, v30, v40;
  eg::Vector3d v11, v21, v31, v41;
  bool result;
  std::string source;
  int source_line;

  std::string to_string() const {
    std::stringstream ss;
    ss << "Edge edge query - " << result << "\n";
    ss << "    line " << source_line << " from " << source << "\n";
    ss << "    v10: " << v10.transpose() << "\n";
    ss << "    v20: " << v20.transpose() << "\n";
    ss << "    v30: " << v30.transpose() << "\n";
    ss << "    v40: " << v40.transpose() << "\n";
    ss << "    v11: " << v11.transpose() << "\n";
    ss << "    v21: " << v21.transpose() << "\n";
    ss << "    v31: " << v31.transpose() << "\n";
    ss << "    v41: " << v41.transpose() << "\n";
    return ss.str();
  }
};

struct PointTriangleQuery {
  // point = p, triangle = v1 v2 v3
  // suffix 0 = at t0, suffix 1 = at t1
  eg::Vector3d p0, v10, v20, v30;
  eg::Vector3d p1, v11, v21, v31;
  bool result;
  std::string source;
  int source_line;

  std::string to_string() const {
    std::stringstream ss;
    ss << "Point triangle query - " << result << "\n";
    ss << "    line " << source_line << " from " << source << "\n";
    ss << "    p0 : " << p0.transpose() << "\n";
    ss << "    v10: " << v10.transpose() << "\n";
    ss << "    v20: " << v20.transpose() << "\n";
    ss << "    v30: " << v30.transpose() << "\n";
    ss << "    p1 : " << p1.transpose() << "\n";
    ss << "    v11: " << v11.transpose() << "\n";
    ss << "    v21: " << v21.transpose() << "\n";
    ss << "    v31: " << v31.transpose() << "\n";
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

std::vector<EdgeEdgeQuery> parse_edge_edge_queries(const fs::path &path) {
  std::ifstream in{path};
  if (!in) {
    throw std::runtime_error("cannot open " + path.string());
  }

  std::vector<EdgeEdgeQuery> out;
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
      EdgeEdgeQuery q;
      q.v10 = buf[0];
      q.v20 = buf[1];
      q.v30 = buf[2];
      q.v40 = buf[3];
      q.v11 = buf[4];
      q.v21 = buf[5];
      q.v31 = buf[6];
      q.v41 = buf[7];
      q.result = expected_res;
      q.source = path;
      q.source_line = start_line;
      out.push_back(q);

      row_in_group = 0;
    }
  }

  if (row_in_group != 0) {
    throw std::runtime_error("trailing incomplete edge–edge group");
  }
  return out;
}

std::vector<PointTriangleQuery> parse_point_triangle_queries(
    const fs::path &path) {
  std::ifstream in{path};
  if (!in) {
    throw std::runtime_error("cannot open " + path.string());
  }

  std::vector<PointTriangleQuery> out;
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
      PointTriangleQuery q;
      q.p0 = buf[0];
      q.v10 = buf[1];
      q.v20 = buf[2];
      q.v30 = buf[3];
      q.p1 = buf[4];
      q.v11 = buf[5];
      q.v21 = buf[6];
      q.v31 = buf[7];
      q.result = expected_res;
      q.source = path;
      q.source_line = start_line;
      out.push_back(q);

      row_in_group = 0;
    }
  }

  if (row_in_group != 0) {
    throw std::runtime_error("trailing incomplete point–triangle group");
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
  std::vector<EdgeEdgeQuery> edge_edge;
  std::vector<PointTriangleQuery> point_triangle;

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
        vector_append(this->edge_edge, parse_edge_edge_queries(entry_path));
      }
    }
    // point triangle queries
    for (const auto &entry : fs::directory_iterator{point_triangle_dir}) {
      auto entry_path = entry.path();
      if (entry.is_regular_file() && entry_path.extension() == ".csv") {
        vector_append(this->point_triangle,
                      parse_point_triangle_queries(entry_path));
      }
    }
  }
};

void test_query_category(const fs::path &root, const std::string &name,
                         CCDSolver &solver) {
  QueryCategory category{root / name};
  for (const auto &q : category.edge_edge) {
    INFO("Category: " << name << "\n"
                      << ccd_solver_to_string(solver) << q.to_string());
    REQUIRE(solver.edge_edge_ccd(q.v10.cast<float>(), q.v20.cast<float>(),
                                 q.v30.cast<float>(), q.v40.cast<float>(),
                                 q.v11.cast<float>(), q.v21.cast<float>(),
                                 q.v31.cast<float>(), q.v41.cast<float>(), 0.0,
                                 1.0) == q.result);
  }

  for (const auto &q : category.point_triangle) {
    INFO("Category: " << name << "\n"
                      << ccd_solver_to_string(solver) << q.to_string());
    REQUIRE(solver.point_triangle_ccd(q.p0.cast<float>(), q.v10.cast<float>(),
                                      q.v20.cast<float>(), q.v30.cast<float>(),
                                      q.p1.cast<float>(), q.v11.cast<float>(),
                                      q.v21.cast<float>(), q.v31.cast<float>(),
                                      0.0, 1.0) == q.result);
  }
}

TEST_CASE("ccd-tests", "[ccd]") {
  CCDSolver solver;
  solver.eps = 1e-6;
  solver.max_iter = 10;
  solver.h = 0.01;
  solver.tol = 0.1;

  fs::path root{SAMPLE_QUERY_ROOT};

  SECTION("chain") { test_query_category(root, "chain", solver); }
  SECTION("cow-heads") { test_query_category(root, "cow-heads", solver); }
  SECTION("erleben-cube-cliff-edges") {
    test_query_category(root, "erleben-cube-cliff-edges", solver);
  }
  SECTION("erleben-cube-internal-edges") {
    test_query_category(root, "erleben-cube-internal-edges", solver);
  }
  SECTION("erleben-sliding-spike") {
    test_query_category(root, "erleben-sliding-spike", solver);
  }
  SECTION("erleben-sliding-wedge") {
    test_query_category(root, "erleben-sliding-wedge", solver);
  }
  SECTION("erleben-spike-crack") {
    test_query_category(root, "erleben-spike-crack", solver);
  }
  SECTION("erleben-spike-hole") {
    test_query_category(root, "erleben-spike-hole", solver);
  }
  SECTION("erleben-spikes") {
    test_query_category(root, "erleben-spikes", solver);
  }
  SECTION("erleben-spike-wedge") {
    test_query_category(root, "erleben-spike-wedge", solver);
  }
  SECTION("erleben-wedge-crack") {
    test_query_category(root, "erleben-wedge-crack", solver);
  }
  SECTION("erleben-wedges") {
    test_query_category(root, "erleben-wedges", solver);
  }
  SECTION("golf-ball") { test_query_category(root, "golf-ball", solver); }
  SECTION("mat-twist") { test_query_category(root, "mat-twist", solver); }
  SECTION("unit-tests") { test_query_category(root, "unit-tests", solver); }
}
