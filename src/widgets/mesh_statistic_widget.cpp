#include "mesh_statistic_widget.hpp"
#include <igl/edges.h>

MeshStatisticWidget::MeshStatisticWidget(EngineContext& engine_context)
    : engine_ctx_(engine_context) {}

void MeshStatisticWidget::compute_stats() {
    if(engine_ctx_.V.size() == 0 || engine_ctx_.F.size() == 0) return;

    // Bounding box
    bbox_min = glm::vec3(INFINITY);
    bbox_max = glm::vec3(-INFINITY);
    for(int i = 0; i < engine_ctx_.V.rows(); i++) {
        Eigen::Vector3f v = engine_ctx_.V.row(i);
        bbox_min = glm::min(bbox_min, glm::vec3(v.x(), v.y(), v.z()));
        bbox_max = glm::max(bbox_max, glm::vec3(v.x(), v.y(), v.z()));
    }

    // Average edge length
    Eigen::MatrixXi E;
    igl::edges(engine_ctx_.F, E);
    
    double total_length = 0;
    for(int i = 0; i < E.rows(); i++) {
        Eigen::Vector3f a = engine_ctx_.V.row(E(i,0));
        Eigen::Vector3f b = engine_ctx_.V.row(E(i,1));
        total_length += (b - a).norm();
    }
    avg_edge_length = E.rows() > 0 ? total_length / E.rows() : 0;

    stats_dirty = false;
}

EventFlag MeshStatisticWidget::draw() {
    if(stats_dirty) compute_stats();

    ImGui::Begin("Mesh Statistics");
    
    ImGui::Text("Bounding Box:");
    ImGui::BulletText("X: %.3f to %.3f", bbox_min.x, bbox_max.x);
    ImGui::BulletText("Y: %.3f to %.3f", bbox_min.y, bbox_max.y); 
    ImGui::BulletText("Z: %.3f to %.3f", bbox_min.z, bbox_max.z);
    ImGui::Text("Dimensions: %.3f x %.3f x %.3f", 
        bbox_max.x - bbox_min.x,
        bbox_max.y - bbox_min.y,
        bbox_max.z - bbox_min.z);
    
    ImGui::Separator();
    ImGui::Text("Average Edge Length: %.3f", avg_edge_length);
    
    ImGui::End();
    return EventFlag::NoEvent;
}

void MeshStatisticWidget::on_event(EventFlag events) {
    if(raw(events & EventFlag::MeshChange)) {
        stats_dirty = true;
    }
}
