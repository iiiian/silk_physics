#include "json_parse.hpp"
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <cctype>

static bool is_json_path(const std::string& path) {
  std::filesystem::path p{path};
  std::string ext = p.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
  return ext == ".json";
}

ParseResult parse_config(const std::string& path,
                         const std::string& default_path,
                         bool check_readable) {
  ParseResult r;

  // Empty: use default
  r.source_path = path.empty() ? default_path : path;

  // Check if it is JSON
  if (!is_json_path(r.source_path)) {
    r.error = "Not .json File: " + r.source_path;
    return r;
  }

  // 3) Check Readability
  if (check_readable) {
    std::error_code ec;
    if (!std::filesystem::exists(r.source_path, ec) || ec) {
      r.error = "File doesn't exist or unreadable: " + r.source_path;
      return r;
    }
    std::ifstream ifs(r.source_path);
    if (!ifs.is_open()) {
      r.error = "Unable to Open the File: " + r.source_path;
      return r;
    }
  }

  r.ok = true;
  return r;
}
