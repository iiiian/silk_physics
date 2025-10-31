#pragma once

#include <filesystem>
#include <vector>

#include "object.hpp"

bool write_scene(const std::filesystem::path& path,
                 const std::vector<pIObject>& objects);
