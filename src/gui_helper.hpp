#pragma once

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>

#include <cstdint>

#include "bit_flag.hpp"

namespace py = polyscope;

// Helper for ImGui double sliders
template <typename T>
bool DragDouble(const char* label, T* p_data, float v_speed, T* min, T* max,
                const char* format, ImGuiSliderFlags flags) {
  return ImGui::DragScalar(label, ImGuiDataType_Double, p_data, v_speed, min,
                           max, format, flags);
}

enum class UIMode { Normal, Paint };

enum class Event : uint32_t { MeshChange = 0 };

struct AppContext {
  UIMode ui_mode = UIMode::Normal;
  py::SurfaceMesh* p_surface = nullptr;
  float mesh_diag = 0;
  std::unordered_set<int> selection = {};
};

class IWidget {
 public:
  virtual ~IWidget() = default;
  virtual void draw() = 0;
  virtual void on_event(Flags<Event> events) = 0;
};
