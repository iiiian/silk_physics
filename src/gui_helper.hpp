#pragma once
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>

#include <Eigen/Core>
#include <cstdint>

#include "flags.hpp"

// Helper for ImGui double sliders
template <typename T>
bool DragDouble(const char* label, T* p_data, float v_speed, T* min, T* max,
                const char* format, ImGuiSliderFlags flags) {
  return ImGui::DragScalar(label, ImGuiDataType_Double, p_data, v_speed, min,
                           max, format, flags);
}

enum class UIMode { Normal, Paint };

enum class EventFlag : uint32_t { NoEvent = 0, MeshChange = 1 };
template <>
inline constexpr bool is_bitflag_v<EventFlag> = true;

struct UIContext {
  UIMode ui_mode = UIMode::Normal;
  polyscope::SurfaceMesh* p_surface = nullptr;
  float mesh_diag = 0;
  std::unordered_set<int> selection = {};
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
