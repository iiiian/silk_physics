#include "json_parse.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "config.hpp"
#include "widgets/ui_console.hpp"

// helper 1
static bool is_json_path(const std::string& path) {
  std::filesystem::path p{path};
  std::string ext = p.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return ext == ".json";
}

//***********************************************************************/
//**                     JSON PARSER Helper                           ** /
//***********************************************************************/
namespace jx {

struct Cur {
  const std::string* s{};
  size_t i{0};
  char peek() const { return i < s->size() ? (*s)[i] : '\0'; }
  char get() { return i < s->size() ? (*s)[i++] : '\0'; }
  bool eof() const { return i >= s->size(); }
};

static void skip_ws(Cur& c) {
  while (!c.eof() && std::isspace(static_cast<unsigned char>(c.peek()))) c.i++;
}
static bool expect(Cur& c, char ch) {
  skip_ws(c);
  if (c.peek() != ch) return false;
  c.i++;
  return true;
}
static bool consume(Cur& c, char ch) {
  skip_ws(c);
  if (c.peek() == ch) {
    c.i++;
    return true;
  }
  return false;
}
static bool parse_literal(Cur& c, const char* lit) {
  skip_ws(c);
  size_t j = 0;
  while (lit[j]) {
    if (c.eof() || c.peek() != lit[j]) return false;
    c.i++;
    j++;
  }
  return true;
}
static bool parse_bool(Cur& c, bool& out) {
  skip_ws(c);
  if (c.peek() == 't') {
    if (parse_literal(c, "true")) {
      out = true;
      return true;
    }
  }
  if (c.peek() == 'f') {
    if (parse_literal(c, "false")) {
      out = false;
      return true;
    }
  }
  return false;
}
static bool parse_null(Cur& c) { return parse_literal(c, "null"); }
static bool parse_string(Cur& c, std::string& out) {
  skip_ws(c);
  if (!expect(c, '\"')) return false;
  std::string tmp;
  while (!c.eof()) {
    char ch = c.get();
    if (ch == '\"') {
      out = std::move(tmp);
      return true;
    }
    if (ch == '\\') {
      if (c.eof()) return false;
      char esc = c.get();
      switch (esc) {
        case '\"':
          tmp.push_back('\"');
          break;
        case '\\':
          tmp.push_back('\\');
          break;
        case '/':
          tmp.push_back('/');
          break;
        case 'b':
          tmp.push_back('\b');
          break;
        case 'f':
          tmp.push_back('\f');
          break;
        case 'n':
          tmp.push_back('\n');
          break;
        case 'r':
          tmp.push_back('\r');
          break;
        case 't':
          tmp.push_back('\t');
          break;
        default:
          tmp.push_back(esc);
          break;
      }
    } else {
      tmp.push_back(ch);
    }
  }
  return false;
}
static bool parse_number(Cur& c, double& out) {
  skip_ws(c);
  size_t start = c.i;
  if (c.peek() == '-' || c.peek() == '+') c.i++;
  while (std::isdigit(static_cast<unsigned char>(c.peek()))) c.i++;
  if (c.peek() == '.') {
    c.i++;
    while (std::isdigit(static_cast<unsigned char>(c.peek()))) c.i++;
  }
  if (c.peek() == 'e' || c.peek() == 'E') {
    c.i++;
    if (c.peek() == '+' || c.peek() == '-') c.i++;
    while (std::isdigit(static_cast<unsigned char>(c.peek()))) c.i++;
  }
  if (start == c.i) return false;
  const char* begin = c.s->c_str() + start;
  char* endptr = nullptr;
  out = std::strtod(begin, &endptr);
  if (endptr == begin) return false;
  c.i = static_cast<size_t>(endptr - c.s->c_str());
  return true;
}
static bool parse_number_array3(Cur& c, std::array<double, 3>& out) {
  skip_ws(c);
  if (!expect(c, '[')) return false;
  skip_ws(c);
  double a, b, d;
  if (!parse_number(c, a)) return false;
  skip_ws(c);
  if (!consume(c, ',')) return false;
  if (!parse_number(c, b)) return false;
  skip_ws(c);
  if (!consume(c, ',')) return false;
  if (!parse_number(c, d)) return false;
  skip_ws(c);
  if (!expect(c, ']')) return false;
  out = {a, b, d};
  return true;
}
static bool parse_scale3(Cur& c, std::array<double, 3>& out) {
  skip_ws(c);
  if (c.peek() == '[') {
    return parse_number_array3(c, out);
  } else {
    double s;
    if (!parse_number(c, s)) return false;
    out = {s, s, s};
    return true;
  }
}

template <class Fn>
static bool parse_object(Cur& c, Fn&& on_kv) {
  if (!expect(c, '{')) return false;
  skip_ws(c);
  if (consume(c, '}')) return true;
  while (true) {
    std::string key;
    if (!parse_string(c, key)) return false;
    if (!expect(c, ':')) return false;
    if (!on_kv(key)) return false;
    skip_ws(c);
    if (consume(c, ',')) continue;
    if (consume(c, '}')) break;
    return false;
  }
  return true;
}
template <class FnItem>
static bool parse_array(Cur& c, FnItem&& on_item) {
  if (!expect(c, '[')) return false;
  skip_ws(c);
  if (consume(c, ']')) return true;
  while (true) {
    if (!on_item()) return false;
    skip_ws(c);
    if (consume(c, ',')) continue;
    if (consume(c, ']')) break;
    return false;
  }
  return true;
}

static bool skip_value(Cur& cur) {
  skip_ws(cur);
  char ch = cur.peek();
  if (ch == '{')
    return parse_object(cur,
                        [&](const std::string&) { return skip_value(cur); });
  if (ch == '[') return parse_array(cur, [&]() { return skip_value(cur); });
  if (ch == '\"') {
    std::string tmp;
    return parse_string(cur, tmp);
  }
  bool b;
  double d;
  if (parse_bool(cur, b)) return true;
  if (parse_number(cur, d)) return true;
  if (parse_null(cur)) return true;
  return false;
}

}  // namespace jx

//********************************/
//*          Global Parse        */
//********************************/
static bool parse_global_obj(jx::Cur& cur, Global& g) {
  using namespace jx;
  return parse_object(cur, [&](const std::string& k) -> bool {
    if (k == "dt") {
      double v;
      if (!parse_number(cur, v)) return false;
      g.dt = v;
      return true;
    }
    if (k == "max_outer_iteration") {
      double v;
      if (!parse_number(cur, v)) return false;
      g.max_outer_iteration = (int)v;
      return true;
    }
    if (k == "max_inner_iteration") {
      double v;
      if (!parse_number(cur, v)) return false;
      g.max_inner_iteration = (int)v;
      return true;
    }
    if (k == "acceleration") {
      std::array<double, 3> a;
      if (!parse_number_array3(cur, a)) return false;
      g.acceleration = a;
      return true;
    }
    if (k == "total_steps") {
      double v;
      if (!parse_number(cur, v)) return false;
      g.total_steps = (int)v;
      return true;
    }
    if (k == "max_time") {
      double v;
      if (!parse_number(cur, v)) return false;
      g.max_time = v;
      return true;
    }
    if (k == "headless") {
      bool b;
      if (!parse_bool(cur, b)) return false;
      g.headless = b;
      return true;
    }
    // skip unknown
    return jx::skip_value(cur);
  });
}

//********************************/
//*      Collision Parse         */
//********************************/
static bool parse_collision_obj(jx::Cur& cur, Collision& c) {
  using namespace jx;
  return parse_object(cur, [&](const std::string& k) -> bool {
    if (k == "enabled") {
      bool b;
      if (!parse_bool(cur, b)) return false;
      c.enabled = b;
      return true;
    }
    if (k == "self_collision") {
      bool b;
      if (!parse_bool(cur, b)) return false;
      c.self_collision = b;
      return true;
    }
    if (k == "group") {
      double v;
      if (!parse_number(cur, v)) return false;
      c.group = (int)v;
      return true;
    }
    if (k == "restitution") {
      double v;
      if (!parse_number(cur, v)) return false;
      c.restitution = v;
      return true;
    }
    if (k == "friction") {
      double v;
      if (!parse_number(cur, v)) return false;
      c.friction = v;
      return true;
    }
    if (k == "thickness") {
      double v;
      if (!parse_number(cur, v)) return false;
      c.thickness = v;
      return true;
    }
    return jx::skip_value(cur);
  });
}

//********************************/
//*          Global Parse        */
//********************************/
static bool parse_transform_obj(jx::Cur& cur, Transform& t) {
  using namespace jx;
  return parse_object(cur, [&](const std::string& k) -> bool {
    if (k == "translation") {
      std::array<double, 3> v;
      if (!parse_number_array3(cur, v)) return false;
      t.translation = v;
      return true;
    }
    if (k == "rotation_euler_deg") {
      std::array<double, 3> v;
      if (!parse_number_array3(cur, v)) return false;
      t.rotation_euler_deg = v;
      return true;
    }
    if (k == "scale") {
      std::array<double, 3> v;
      if (!parse_scale3(cur, v)) return false;
      t.scale = v;
      return true;
    }
    return jx::skip_value(cur);
  });
}

//********************************/
//*          Cloth Parse         */
//********************************/
static bool parse_cloth_params_obj(jx::Cur& cur, ClothParams& p) {
  using namespace jx;
  return parse_object(cur, [&](const std::string& k) -> bool {
    if (k == "elastic_stiffness") {
      double v;
      if (!parse_number(cur, v)) return false;
      p.elastic_stiffness = v;
      return true;
    }
    if (k == "bending_stiffness") {
      double v;
      if (!parse_number(cur, v)) return false;
      p.bending_stiffness = v;
      return true;
    }
    if (k == "density") {
      double v;
      if (!parse_number(cur, v)) return false;
      p.density = v;
      return true;
    }
    if (k == "damping") {
      double v;
      if (!parse_number(cur, v)) return false;
      p.damping = v;
      return true;
    }
    return jx::skip_value(cur);
  });
}

//********************************/
//*          Object Parse        */
//********************************/
static bool parse_object_item(jx::Cur& cur, Config& cfg) {
  using namespace jx;
  std::string type_str, name, mesh;
  Collision col{};
  Transform tr{};
  ClothParams clothp{};

  if (!parse_object(cur, [&](const std::string& k) -> bool {
        if (k == "type") {
          return parse_string(cur, type_str);
        }
        if (k == "name") {
          return parse_string(cur, name);
        }
        if (k == "mesh") {
          return parse_string(cur, mesh);
        }
        if (k == "collision") {
          return parse_collision_obj(cur, col);
        }
        if (k == "transform") {
          return parse_transform_obj(cur, tr);
        }
        if (k == "cloth") {
          return parse_cloth_params_obj(cur, clothp);
        }
        return jx::skip_value(cur);
      }))
    return false;

  if (type_str == "cloth") {
    ClothObject co;
    co.type = ObjectType::Cloth;
    co.name = name;
    co.mesh = mesh;
    co.collision = col;
    co.transform = tr;
    co.cloth = clothp;
    cfg.cloths.emplace_back(std::move(co));
  } else if (type_str == "obstacle") {
    ObstacleObject oo;
    oo.type = ObjectType::Obstacle;
    oo.name = name;
    oo.mesh = mesh;
    oo.collision = col;
    oo.transform = tr;
    cfg.obstacles.emplace_back(std::move(oo));
  } else {
    UI_LOGI("Unknown object type: {}", type_str);
    return false;
  }
  return true;
}

static bool parse_root(jx::Cur& cur, Config& cfg) {
  using namespace jx;
  return parse_object(cur, [&](const std::string& k) -> bool {
    if (k == "global") {
      return parse_global_obj(cur, cfg.global);
    }
    if (k == "objects") {
      return parse_array(cur, [&]() { return parse_object_item(cur, cfg); });
    }
    return jx::skip_value(cur);
  });
}

//********************************/
//*         TEST OUTPUT          */
//********************************/
static void console_test(const Config& c) {
  // Global
  UI_LOGI("[Global]");
  UI_LOGI("  dt = {:.6f}", c.global.dt);
  UI_LOGI("  max_outer_iteration = {}", c.global.max_outer_iteration);
  UI_LOGI("  max_inner_iteration = {}", c.global.max_inner_iteration);
  UI_LOGI("  acceleration = [{:.6f}, {:.6f}, {:.6f}]", c.global.acceleration[0],
          c.global.acceleration[1], c.global.acceleration[2]);
  UI_LOGI("  total_steps = {}", c.global.total_steps);
  UI_LOGI("  max_time = {:.6f}", c.global.max_time);
  UI_LOGI("  headless = {}", c.global.headless ? "true" : "false");

  // Cloths
  UI_LOGI("[Cloths] count = {}", c.cloths.size());
  for (size_t i = 0; i < c.cloths.size(); ++i) {
    const auto& o = c.cloths[i];
    UI_LOGI("  - [{}] name = {}", i, o.name);
    UI_LOGI("      mesh = {}", o.mesh);
    UI_LOGI(
        "      [collision] enabled={}, self_collision={}, group={}, "
        "restitution={:.6f}, friction={:.6f}, thickness={:.6f}",
        o.collision.enabled ? "true" : "false",
        o.collision.self_collision ? "true" : "false", o.collision.group,
        o.collision.restitution, o.collision.friction, o.collision.thickness);
    UI_LOGI(
        "      [transform] translation=[{:.6f},{:.6f},{:.6f}] "
        "rotation_euler_deg=[{:.6f},{:.6f},{:.6f}] "
        "scale=[{:.6f},{:.6f},{:.6f}]",
        o.transform.translation[0], o.transform.translation[1],
        o.transform.translation[2], o.transform.rotation_euler_deg[0],
        o.transform.rotation_euler_deg[1], o.transform.rotation_euler_deg[2],
        o.transform.scale[0], o.transform.scale[1], o.transform.scale[2]);
    UI_LOGI(
        "      [cloth] elastic_stiffness={:.6f} bending_stiffness={:.6f} "
        "density={:.6f} damping={:.6f}",
        o.cloth.elastic_stiffness, o.cloth.bending_stiffness, o.cloth.density,
        o.cloth.damping);
  }

  // Obstacles
  UI_LOGI("[Obstacles] count = {}", c.obstacles.size());
  for (size_t i = 0; i < c.obstacles.size(); ++i) {
    const auto& o = c.obstacles[i];
    UI_LOGI("  - [{}] name = {}", i, o.name);
    UI_LOGI("      mesh = {}", o.mesh);
    UI_LOGI(
        "      [collision] enabled={}, self_collision={}, group={}, "
        "restitution={:.6f}, friction={:.6f}, thickness={:.6f}",
        o.collision.enabled ? "true" : "false",
        o.collision.self_collision ? "true" : "false", o.collision.group,
        o.collision.restitution, o.collision.friction, o.collision.thickness);
    UI_LOGI(
        "      [transform] translation=[{:.6f},{:.6f},{:.6f}] "
        "rotation_euler_deg=[{:.6f},{:.6f},{:.6f}] "
        "scale=[{:.6f},{:.6f},{:.6f}]",
        o.transform.translation[0], o.transform.translation[1],
        o.transform.translation[2], o.transform.rotation_euler_deg[0],
        o.transform.rotation_euler_deg[1], o.transform.rotation_euler_deg[2],
        o.transform.scale[0], o.transform.scale[1], o.transform.scale[2]);
  }
}

// JSON Parser 2 Helpers

static std::optional<std::string> check_json_path(const std::string& path) {
  if (!is_json_path(path)) {
    return " .JSON File Required: " + path;
  }
  return std::nullopt;
}

static std::optional<std::string> check_readable_if_needed(
    const std::string& path, bool check) {
  if (!check) {
    return std::nullopt;
  }
  std::error_code ec;
  if (!std::filesystem::exists(path, ec) || ec) {
    return "File doesn't exist or unreadable: " + path;
  }
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    return "Unable to Open the File: " + path;
  }
  return std::nullopt;
}

static std::optional<std::string> load_text_noexcept(const std::string& path,
                                                     std::string& out_text) {
  std::ifstream ifs(path);
  if (!ifs) {
    return "Unable to Open: " + path;
  }
  out_text.assign(std::istreambuf_iterator<char>(ifs),
                  std::istreambuf_iterator<char>());
  return std::nullopt;
}

//***********************************************************************/
//**                    JSON PARSER [std::optional]                   ** /
//***********************************************************************/

/**
 * @brief Parsing a JSON file into a struct.
 * @param path, default_path, check_readable Storage container to inspect.
 * @return ParseResult struct.
 */

ParseResult parse_config(const std::string& path,
                         const std::string& default_path, bool check_readable) {
  ParseResult r;
  //[M1] path?
  if (path.empty()) {
    r.source_path = default_path;
  } else {
    r.source_path = path;
  }
  // [M2] .json?
  if (auto e = check_json_path(r.source_path)) {
    r.error = *e;
    return r;
  }

  // [M3] Readable?
  if (auto e = check_readable_if_needed(r.source_path, check_readable)) {
    r.error = *e;
    return r;
  }

  // [M4] Json to string
  std::string text;
  if (auto e = load_text_noexcept(r.source_path, text)) {
    r.error = *e;
    return r;
  }

  // [M5] String to Struct
  jx::Cur cur{&text, 0};
  if (!parse_root(cur, r.config)) {
    r.ok = false;
    r.error = "JSON parse error near offset " + std::to_string(cur.i);
    return r;
  }

  r.ok = true;

  // console output test
  if (true) {
    console_test(r.config);
    UI_LOGI("Source Path: {}", r.source_path);
  }
  return r;
}
