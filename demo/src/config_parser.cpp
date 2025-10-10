#include "config_parser.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace cli {

static bool get_vec3(const json& j, const char* key, std::array<double,3>& out) {
  if (!j.contains(key) || !j[key].is_array() || j[key].size()!=3) return false;
  for (int i=0;i<3;i++) if (!j[key][i].is_number()) return false;
  out = { j[key][0].get<double>(), j[key][1].get<double>(), j[key][2].get<double>() };
  return true;
}

static bool is_mesh_ext_ok(const std::string& p) {
  auto dot = p.rfind('.');
  if (dot == std::string::npos) return false;
  std::string ext = p.substr(dot+1);
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return char(std::tolower(c)); });
  return (ext=="off"||ext=="obj"||ext=="ply"||ext=="stl");
}

static bool starts_with_model(const std::string& p) {
  return p.rfind("model/",0)==0 || p.rfind("./model/",0)==0;
}

static bool parse_collision(const json& j, silk::CollisionConfig& out, std::string& err) {
  if (!j.is_object()) { err="collision must be object"; return false; }
  if (!j.contains("enabled") || !j["enabled"].is_boolean()) { err="collision.enabled must be boolean"; return false; }
  if (!j.contains("self_collision") || !j["self_collision"].is_boolean()) { err="collision.self_collision must be boolean"; return false; }
  if (!j.contains("group") || !j["group"].is_number_integer()) { err="collision.group must be integer"; return false; }
  if (!j.contains("restitution") || !j["restitution"].is_number()) { err="collision.restitution must be number"; return false; }
  if (!j.contains("friction") || !j["friction"].is_number()) { err="collision.friction must be number"; return false; }

  double rest = j["restitution"].get<double>(), fric = j["friction"].get<double>();
  if (rest<0.0 || rest>1.0) { err="collision.restitution in [0,1]"; return false; }
  if (fric<0.0 || fric>1.0) { err="collision.friction in [0,1]"; return false; }

  out.is_collision_on      = j["enabled"].get<bool>();
  out.is_self_collision_on = j["self_collision"].get<bool>();
  out.group                = j["group"].get<int>();
  out.restitution          = static_cast<float>(rest);
  out.friction             = static_cast<float>(fric);

  if (j.contains("thickness") && j["thickness"].is_number() && j["thickness"].get<double>()<0.0) {
    err="collision.thickness >= 0"; return false;
  }
  return true;
}

static bool parse_cloth(const json& j, silk::ClothConfig& out, std::string& err) {
  if (!j.is_object()) { err="cloth must be object"; return false; }
  auto need_num = [&](const char* k, double min, bool ge, float& dst)->bool{
    if (!j.contains(k) || !j[k].is_number()) { err=std::string("cloth.")+k+" must be number"; return false; }
    double v = j[k].get<double>();
    if ((ge && v<min) || (!ge && v<=min)) { err=std::string("cloth.")+k+(ge?" >= ":" > ")+std::to_string(min); return false; }
    dst = static_cast<float>(v); return true;
  };
  if (!need_num("elastic_stiffness", 0.0, false, out.elastic_stiffness)) return false;
  if (!need_num("bending_stiffness", 0.0, true,  out.bending_stiffness))  return false;
  if (!need_num("density",           0.0, false, out.density))            return false;
  if (!need_num("damping",           0.0, true,  out.damping))            return false;
  return true;
}

static bool parse_transform(const json& j, Transform& t, std::string& err) {
  if (!j.is_object()) { err="transform must be object"; return false; }
  t.enabled = true;

  std::array<double,3> v;
  if (get_vec3(j, "translation",       v)) t.translation       = v;
  if (get_vec3(j, "rotation_euler_deg",v)) t.rotation_euler_deg= v;

  if (j.contains("scale")) {
    if (j["scale"].is_number()) {
      double s = j["scale"].get<double>(); if (s<=0.0) { err="transform.scale > 0"; return false; }
      t.scale_uniform = s;
    } else if (j["scale"].is_array() && j["scale"].size()==3) {
      for (int i=0;i<3;i++) if (!j["scale"][i].is_number() || j["scale"][i].get<double>()<=0.0) { err="transform.scale each > 0"; return false; }
      t.scale_xyz = { j["scale"][0].get<double>(), j["scale"][1].get<double>(), j["scale"][2].get<double>() };
    } else {
      err="transform.scale must be number or [sx,sy,sz]"; return false;
    }
  }
  return true;
}

bool load_sim_config(const std::string& json_path, SimConfig& out, std::string& err) {
  std::ifstream in(json_path);
  if (!in) { err = "cannot open config: " + json_path; return false; }

  json root;
  try { root = json::parse(in, nullptr, true, true); }
  catch (const std::exception& e) { err = std::string("JSON parse error: ")+e.what(); return false; }

  if (!root.contains("global") || !root["global"].is_object()) { err="missing object 'global'"; return false; }
  const auto& g = root["global"];

  if (!g.contains("dt") || !g["dt"].is_number() || g["dt"].get<double>()<=0.0) { err="global.dt > 0"; return false; }
  out.global.dt = static_cast<float>(g["dt"].get<double>());

  if (!g.contains("max_outer_iteration") || !g["max_outer_iteration"].is_number_integer() || g["max_outer_iteration"].get<int>()<1) { err="global.max_outer_iteration >= 1"; return false; }
  out.global.max_outer_iteration = g["max_outer_iteration"].get<int>();

  if (!g.contains("max_inner_iteration") || !g["max_inner_iteration"].is_number_integer() || g["max_inner_iteration"].get<int>()<1) { err="global.max_inner_iteration >= 1"; return false; }
  out.global.max_inner_iteration = g["max_inner_iteration"].get<int>();

  std::array<double,3> a;
  if (!get_vec3(g, "acceleration", a)) { err="global.acceleration must be [x,y,z]"; return false; }
  out.global.acceleration_x = static_cast<float>(a[0]);
  out.global.acceleration_y = static_cast<float>(a[1]);
  out.global.acceleration_z = static_cast<float>(a[2]);

  int total_steps = -1;
  double max_time = -1.0;
  if (g.contains("total_steps")) {
    if (!g["total_steps"].is_number_integer() || g["total_steps"].get<int>()<1) { err="global.total_steps >= 1"; return false; }
    total_steps = g["total_steps"].get<int>();
  }
  if (g.contains("max_time")) {
    if (!g["max_time"].is_number() || g["max_time"].get<double>()<=0.0) { err="global.max_time > 0"; return false; }
    max_time = g["max_time"].get<double>();
  }
  if (total_steps < 1 && max_time <= 0.0) { err="one of global.total_steps or global.max_time must be set"; return false; }
  if (total_steps < 1 && max_time > 0.0) total_steps = std::max(1, (int)std::floor(max_time / out.global.dt));
  out.stop.total_steps = total_steps;
  out.stop.max_time    = max_time;

  if (!g.contains("headless") || !g["headless"].is_boolean()) { err="global.headless must be boolean"; return false; }
  out.headless = g["headless"].get<bool>();

  if (!root.contains("objects") || !root["objects"].is_array() || root["objects"].empty()) { err="objects must be non-empty array"; return false; }

  out.objects.clear();
  for (const auto& it : root["objects"]) {
    if (!it.is_object()) { err="objects[i] must be object"; return false; }
    ObjectConfig obj;

    if (!it.contains("type") || !it["type"].is_string()) { err="objects[i].type must be string"; return false; }
    std::string t = it["type"].get<std::string>();
    if (t=="cloth") obj.type = ObjectType::Cloth;
    else if (t=="obstacle") obj.type = ObjectType::Obstacle;
    else { err="objects[i].type must be 'cloth' or 'obstacle'"; return false; }

    if (it.contains("name") && it["name"].is_string()) obj.name = it["name"].get<std::string>();

    if (!it.contains("mesh") || !it["mesh"].is_string()) { err="objects[i].mesh must be string"; return false; }
    obj.mesh_path = it["mesh"].get<std::string>();
    if (!starts_with_model(obj.mesh_path)) { err="objects[i].mesh must be under ./model"; return false; }
    if (!is_mesh_ext_ok(obj.mesh_path))   { err="objects[i].mesh extension must be one of .off/.obj/.ply/.stl"; return false; }
    if (!fs::exists(obj.mesh_path))       { err="mesh file not found: "+obj.mesh_path; return false; }

    if (!it.contains("collision")) { err="objects[i].collision missing"; return false; }
    if (!parse_collision(it["collision"], obj.collision, err)) return false;
    obj.has_collision = true;

    if (obj.type == ObjectType::Cloth) {
      if (!it.contains("cloth")) { err="cloth object must have 'cloth' config"; return false; }
      if (!parse_cloth(it["cloth"], obj.cloth, err)) return false;
      obj.has_cloth = true;

      if (it.contains("pins")) {
        const auto& p = it["pins"];
        if (!p.is_object()) { err="pins must be object"; return false; }
        if (p.contains("indices")) {
          if (!p["indices"].is_array()) { err="pins.indices must be array"; return false; }
          for (auto& v : p["indices"]) {
            if (!v.is_number_integer()) { err="pins.indices must be int array"; return false; }
            obj.pin_indices.push_back(v.get<int>());
          }
        }
        if (p.contains("positions")) {
          if (!p["positions"].is_array()) { err="pins.positions must be array"; return false; }
          for (auto& v : p["positions"]) {
            if (!v.is_number()) { err="pins.positions must be number array"; return false; }
            obj.pin_positions.push_back(static_cast<float>(v.get<double>()));
          }
          if (!obj.pin_indices.empty() && obj.pin_positions.size() != obj.pin_indices.size()*3) {
            err = "pins.positions length must be 3 * len(pins.indices)"; return false;
          }
        }
      }
    } else {
      if (it.contains("cloth")) { err="obstacle must NOT have 'cloth' section"; return false; }
      if (it.contains("pins"))  { err="obstacle must NOT have 'pins'"; return false; }
    }

    if (it.contains("transform")) {
      Transform tr;
      if (!parse_transform(it["transform"], tr, err)) return false;
      obj.transform = tr;
    }

    out.objects.push_back(std::move(obj));
  }

  return true;
}

} 
