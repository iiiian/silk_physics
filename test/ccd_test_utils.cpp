#include "ccd_test_utils.hpp"

#include <Eigen/Core>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
namespace eg = Eigen;

std::string Query::to_string() const {
  std::stringstream ss;
  std::string type_str = (type == QueryType::EdgeEdge)
                             ? "Edge edge query: "
                             : "Point triangle query: ";
  std::string result_str = (result) ? "true" : "false";
  ss << type_str << "result = " << result_str << "\n";
  ss << "    line " << source_line << " from " << source << "\n";
  if (type == QueryType::EdgeEdge) {
    ss << "    v00: " << v00.transpose() << "\n";
    ss << "    v10: " << v10.transpose() << "\n";
    ss << "    v20: " << v20.transpose() << "\n";
    ss << "    v30: " << v30.transpose() << "\n";
    ss << "    v01: " << v01.transpose() << "\n";
    ss << "    v11: " << v11.transpose() << "\n";
    ss << "    v21: " << v21.transpose() << "\n";
    ss << "    v31: " << v31.transpose() << "\n";
  } else {
    ss << "    v00: " << v00.transpose() << "\n";
    ss << "    v10: " << v10.transpose() << "\n";
    ss << "    v20: " << v20.transpose() << "\n";
    ss << "    p0 : " << v30.transpose() << "\n";
    ss << "    v01: " << v01.transpose() << "\n";
    ss << "    v11: " << v11.transpose() << "\n";
    ss << "    v21: " << v21.transpose() << "\n";
    ss << "    p1 : " << v31.transpose() << "\n";
  }
  return ss.str();
}

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
        q.v00 = buf[0];
        q.v10 = buf[1];
        q.v20 = buf[2];
        q.v30 = buf[3];
        q.v01 = buf[4];
        q.v11 = buf[5];
        q.v21 = buf[6];
        q.v31 = buf[7];
      } else {
        q.v30 = buf[0];
        q.v00 = buf[1];
        q.v10 = buf[2];
        q.v20 = buf[3];
        q.v31 = buf[4];
        q.v01 = buf[5];
        q.v11 = buf[6];
        q.v21 = buf[7];
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

QueryCategory::QueryCategory(const fs::path &path) {
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
