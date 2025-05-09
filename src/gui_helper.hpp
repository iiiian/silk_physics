#pragma once
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>

#include <Eigen/Core>
#include <cstdint>

#include "flags.hpp"

enum class UIMode { Normal, Paint, ClothSim };
const std::string NORMAL_MODE_HELP_TXT =
    "NORMAL MODE | Left Click Rotate / Right Click Move";
const std::string PAINT_MODE_HELP_TXT =
    "PAINT MODE | Left Click Paint / Right Click Erase";
const std::string CLOTH_SIM_MODE_HELP_TXT =
    "CLOTH SIM MODE | Left Click Rotate / Right Click Move/ Ctrl + Left Click "
    "Drag";

enum class EventFlag : uint32_t { NoEvent = 0, MeshChange = 1 };
template <>
inline constexpr bool is_bitflag_v<EventFlag> = true;

struct UIContext {
  UIMode ui_mode = UIMode::Normal;
  polyscope::SurfaceMesh* p_surface = nullptr;
  float mesh_diag = 0;
  std::unordered_set<int> selection;
  std::string help_text = NORMAL_MODE_HELP_TXT;
};

struct EngineContext {
  // Basic mesh
  // V needs to be row major for easy vectorization
  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajorBit> V;
  Eigen::MatrixX3i F;
};

class IWidget {
 public:
  virtual ~IWidget() = default;
  virtual EventFlag draw() = 0;
  virtual void on_event(EventFlag events) = 0;
};
