#pragma once

#include <string>

#include "config.hpp"

/**
 * @brief Run a headless simulation and export the resulting scene.
 * @param config Simulation description including global and object settings.
 * @param out_path Destination Alembic file path for the exported scene.
 */
#include <silk/silk.hpp>

void headless_run(const SimConfig& config, const std::string& out_path,
                  silk::Backend backend);
