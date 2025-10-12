#pragma once
#include <string>

//**************************************************************/
//**                 JSON File Parser                          */
//**************************************************************/

//Struct for simulation parameter
//Work in Progress...
struct ParseResult {
  bool ok = false;           // Parse Success?
  std::string source_path;   // JSON Path
  std::string error;         // Error Info
};

//Path
ParseResult parse_config(const std::string& path,
                         const std::string& default_path = "/DTest/default.json",
                         bool check_readable = true);
