#pragma once
#include "gui_helper.hpp"

class MeshStatisticWidget : public IWidget {
    EngineContext& engine_ctx_;
    
    glm::vec3 bbox_min;
    glm::vec3 bbox_max;
    float avg_edge_length;
    bool stats_dirty = true;

    void compute_stats();

public:
    explicit MeshStatisticWidget(EngineContext& engine_context);
    EventFlag draw() override;
    void on_event(EventFlag events) override;
};
