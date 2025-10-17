#pragma once
#include <string>

#include "config.hpp"

struct Config;

//**************************************************************/
//**                 JSON File Parser                          */
//**************************************************************/

// Struct for simulation parameter
// The struct below is not the one use for simulation enviroment setup,
// rather, it include it--Config struct--inside.

struct ParseResult {
  bool ok{false};
  std::string error;
  std::string source_path;
  Config config;
};

/**
 * @brief Parsing a JSON file into a struct.
 * @param path, default_path, check_readable Storage container to inspect.
 * @return ParseResult struct.
 */

ParseResult parse_config(const std::string& path,
                         const std::string& default_path, bool check_readable);
