#pragma once

#include <optional>
#include <string>

#include "config.hpp"

//**************************************************************/
//**                 JSON File Parser                          */
//**************************************************************/

// Struct for simulation parameter
// The struct below is not the one use for simulation enviroment setup,
// rather, it include it--Config struct--inside.

/**
 * @brief Parsing a JSON file into a struct.
 * @param path, default_path, check_readable Storage container to inspect.
 * @return ParseResult struct.
 */
std::optional<SimConfig> parse_config(const std::string& path);
