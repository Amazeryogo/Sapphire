#include "renderer.h"

#include <cmath>
#include <cstddef>

namespace {
constexpr double kPi = 3.14159265358979323846;

const char* kBodyVertex = R"(#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec4 i_row0;
layout(location = 3) in vec4 i_row1;
layout(location = 4) in vec4 i_row2;
layout(location = 5) in vec4 i_color;
layout(location = 6) in vec4 i_params;
uniform mat4 u_view_proj;
uniform vec3 u_light;       // unit, towards the light
uniform int u_shadow;       // 1: flatten onto the floor along the light
out vec3 v_world;
out vec3 v_normal;
out vec3 v_local;
out vec4 v_color;
out vec4 v_params;
void main() {
    vec4 p = vec4(a_position, 1.0);
    vec3 world = vec3(dot(i_row0, p), dot(i_row1, p), dot(i_row2, p));
    // Normal matrix of a rotation times a (possibly non-uniform) scale:
    // each column of the linear part divided by its squared length.
    vec3 c0 = vec3(i_row0.x, i_row1.x, i_row2.x);
    vec3 c1 = vec3(i_row0.y, i_row1.y, i_row2.y);
    vec3 c2 = vec3(i_row0.z, i_row1.z, i_row2.z);
    v_normal = c0 * (a_normal.x / dot(c0, c0)) + c1 * (a_normal.y / dot(c1, c1)) + c2 * (a_normal.z / dot(c2, c2));
    if (u_shadow == 1) {
        world -= u_light * (max(world.y, 0.0) / u_light.y);
        world.y = 0.12;
    }
    v_world = world;
    v_local = a_position;
    v_color = i_color;
    v_params = i_params;
    gl_Position = u_view_proj * vec4(world, 1.0);
}
)";

const char* kBodyFragment = R"(#version 330 core
in vec3 v_world;
in vec3 v_normal;
in vec3 v_local;
in vec4 v_color;
in vec4 v_params;
uniform vec3 u_light;
uniform vec3 u_eye;
uniform int u_shadow;
uniform vec3 u_room;        // room size: shadows stop at the walls
out vec4 frag;
void main() {
    if (u_shadow == 1) {
        if (v_world.x < 0.0 || v_world.z < 0.0 || v_world.x > u_room.x || v_world.z > u_room.z) discard;
        frag = vec4(0.0, 0.0, 0.0, 0.34 * v_color.a);
        return;
    }
    vec3 n = normalize(v_normal);
    if (!gl_FrontFacing) n = -n;
    vec3 view = normalize(u_eye - v_world);
    vec3 base = v_color.rgb;

    // Markings that make rotation visible.
    if (v_params.z > 0.5 && v_params.z < 1.5) {
        float band = smoothstep(0.10, 0.14, abs(v_local.y));
        float cap = smoothstep(0.93, 0.95, abs(v_local.x));
        base *= mix(0.62, 1.0, band) * mix(1.0, 0.62, cap);
    } else if (v_params.z > 1.5) {
        vec3 a = abs(v_local);
        float edge = max(max(min(a.x, a.y), min(a.y, a.z)), min(a.x, a.z));
        base *= 1.0 - 0.30 * smoothstep(0.90, 1.0, edge);
    }

    float diffuse = max(dot(n, u_light), 0.0);
    vec3 ambient = mix(vec3(0.20, 0.21, 0.26), vec3(0.46, 0.48, 0.54), n.y * 0.5 + 0.5);
    vec3 halfway = normalize(u_light + view);
    float specular = pow(max(dot(n, halfway), 0.0), 48.0) * v_params.y;
    float rim = pow(1.0 - max(dot(n, view), 0.0), 3.0) * 0.18 * v_params.y;
    vec3 color = base * (ambient + diffuse * 0.78) + vec3(specular + rim) + base * v_params.x;
    float alpha = v_color.a;
    if (alpha < 0.99) alpha = clamp(alpha + rim * 1.5 + specular, 0.0, 1.0);    // glass: brighter at grazing angles
    frag = vec4(color, alpha);
}
)";

const char* kRoomVertex = R"(#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
uniform mat4 u_view_proj;
out vec3 v_world;
out vec3 v_normal;
void main() {
    v_world = a_position;
    v_normal = a_normal;
    gl_Position = u_view_proj * vec4(a_position, 1.0);
}
)";

const char* kRoomFragment = R"(#version 330 core
in vec3 v_world;
in vec3 v_normal;
uniform vec3 u_light;
out vec4 frag;
float grid(vec2 p, float spacing, float width) {
    vec2 g = abs(fract(p / spacing - 0.5) - 0.5) * spacing;
    vec2 w = fwidth(p) * width;
    vec2 line = 1.0 - smoothstep(vec2(0.0), w, g);
    return max(line.x, line.y);
}
void main() {
    vec3 n = normalize(v_normal);
    vec2 p = abs(n.y) > 0.5 ? v_world.xz : (abs(n.x) > 0.5 ? v_world.zy : v_world.xy);
    bool floor_face = n.y > 0.5;
    vec3 base = floor_face ? vec3(0.205, 0.215, 0.250) : vec3(0.150, 0.158, 0.188);
    base *= 0.80 + 0.20 * max(dot(n, u_light), 0.0);
    float fine = grid(p, 50.0, 1.2);
    float coarse = grid(p, 100.0, 2.0);
    vec3 color = mix(base, vec3(0.33, 0.36, 0.44), max(fine * 0.35, coarse * 0.6) * (floor_face ? 1.0 : 0.6));
    frag = vec4(color, 1.0);
}
)";

void append_sphere(std::vector<float>& v, std::vector<unsigned>& idx, int slices, int stacks) {
    for (int i = 0; i <= stacks; ++i) {
        const double phi = kPi * i / stacks;
        for (int j = 0; j <= slices; ++j) {
            const double theta = 2.0 * kPi * j / slices;
            const float x = float(std::sin(phi) * std::cos(theta)), y = float(std::cos(phi)), z = float(std::sin(phi) * std::sin(theta));
            v.insert(v.end(), {x, y, z, x, y, z});
        }
    }
    for (int i = 0; i < stacks; ++i)
        for (int j = 0; j < slices; ++j) {
            const unsigned a = unsigned(i * (slices + 1) + j), b = a + unsigned(slices + 1);
            idx.insert(idx.end(), {a, a + 1, b, a + 1, b + 1, b});
        }
}

void append_cube(std::vector<float>& v, std::vector<unsigned>& idx) {
    const float n[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (int f = 0; f < 6; ++f) {
        const float* normal = n[f];
        // Two axes spanning the face.
        const int axis = f / 2, u = (axis + 1) % 3, w = (axis + 2) % 3;
        const unsigned base = unsigned(v.size() / 6);
        for (int corner = 0; corner < 4; ++corner) {
            float p[3];
            p[axis] = normal[axis];
            p[u] = (corner == 0 || corner == 3) ? -1.0f : 1.0f;
            p[w] = (corner < 2) ? -1.0f : 1.0f;
            v.insert(v.end(), {p[0], p[1], p[2], normal[0], normal[1], normal[2]});
        }
        idx.insert(idx.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
}

// Unit rod along +y from 0 to 1, radius 1.
void append_rod(std::vector<float>& v, std::vector<unsigned>& idx, int slices) {
    for (int j = 0; j <= slices; ++j) {
        const double theta = 2.0 * kPi * j / slices;
        const float x = float(std::cos(theta)), z = float(std::sin(theta));
        v.insert(v.end(), {x, 0.0f, z, x, 0.0f, z});
        v.insert(v.end(), {x, 1.0f, z, x, 0.0f, z});
    }
    for (int j = 0; j < slices; ++j) {
        const unsigned a = unsigned(2 * j);
        idx.insert(idx.end(), {a, a + 1, a + 2, a + 1, a + 3, a + 2});
    }
}

void rotation_columns(const Quat& q, Vec3 c[3]) {
    c[0] = q.axis(0); c[1] = q.axis(1); c[2] = q.axis(2);
}

void set_color(float out[4], const Rgba& c) {
    out[0] = float(c.r) / 255.0f; out[1] = float(c.g) / 255.0f; out[2] = float(c.b) / 255.0f; out[3] = float(c.a) / 255.0f;
}

// Colour shift that makes temperature visible: hot → red/orange, cold → blue.
void thermal_tint(float c[4], double temperature) {
    if (temperature > 25.0) {
        const float k = float(clampd((temperature - 25.0) / 250.0, 0.0, 0.85));
        c[0] += (1.0f - c[0]) * k; c[1] += (0.35f - c[1]) * k; c[2] += (0.16f - c[2]) * k;
    } else if (temperature < 10.0) {
        const float k = float(clampd((10.0 - temperature) / 120.0, 0.0, 0.8));
        c[0] += (0.47f - c[0]) * k; c[1] += (0.78f - c[1]) * k; c[2] += (1.0f - c[2]) * k;
    }
}
} // namespace

bool Renderer::initialize() {
    initializeOpenGLFunctions();
    auto build = [&](std::unique_ptr<QOpenGLShaderProgram>& program, const char* vs, const char* fs) {
        program = std::make_unique<QOpenGLShaderProgram>();
        if (!program->addShaderFromSourceCode(QOpenGLShader::Vertex, vs) ||
            !program->addShaderFromSourceCode(QOpenGLShader::Fragment, fs) || !program->link()) {
            error_ = program->log();
            return false;
        }
        return true;
    };
    if (!build(body_program_, kBodyVertex, kBodyFragment)) return false;
    if (!build(room_program_, kRoomVertex, kRoomFragment)) return false;

    std::vector<float> v;
    std::vector<unsigned> idx;
    append_sphere(v, idx, 28, 18);
    sphere_ = make_mesh(v, idx);
    v.clear(); idx.clear();
    append_sphere(v, idx, 10, 7);
    particle_ = make_mesh(v, idx);
    v.clear(); idx.clear();
    append_cube(v, idx);
    cube_ = make_mesh(v, idx);
    v.clear(); idx.clear();
    append_rod(v, idx, 10);
    rod_ = make_mesh(v, idx);

    glGenVertexArrays(1, &room_vao_);
    glGenBuffers(1, &room_vbo_);
    glBindVertexArray(room_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, room_vbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glBindVertexArray(0);
    ready_ = true;
    return true;
}

void Renderer::release() {
    if (!ready_) return;
    for (Mesh* m : {&sphere_, &particle_, &cube_, &rod_}) {
        glDeleteVertexArrays(1, &m->vao);
        glDeleteBuffers(1, &m->vbo);
        glDeleteBuffers(1, &m->ibo);
        glDeleteBuffers(1, &m->instance_vbo);
    }
    glDeleteVertexArrays(1, &room_vao_);
    glDeleteBuffers(1, &room_vbo_);
    body_program_.reset();
    room_program_.reset();
    ready_ = false;
}

Renderer::Mesh Renderer::make_mesh(const std::vector<float>& vertices, const std::vector<unsigned>& indices) {
    Mesh m;
    m.index_count = int(indices.size());
    glGenVertexArrays(1, &m.vao);
    glBindVertexArray(m.vao);
    glGenBuffers(1, &m.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(vertices.size() * sizeof(float)), vertices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glGenBuffers(1, &m.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(indices.size() * sizeof(unsigned)), indices.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &m.instance_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m.instance_vbo);
    for (unsigned attribute = 0; attribute < 5; ++attribute) {
        glEnableVertexAttribArray(2 + attribute);
        glVertexAttribPointer(2 + attribute, 4, GL_FLOAT, GL_FALSE, sizeof(Instance),
                              reinterpret_cast<void*>(std::size_t(attribute) * 4 * sizeof(float)));
        glVertexAttribDivisor(2 + attribute, 1);
    }
    glBindVertexArray(0);
    return m;
}

void Renderer::draw_instances(const Mesh& mesh, const std::vector<Instance>& instances) {
    if (instances.empty()) return;
    glBindVertexArray(mesh.vao);
    glBindBuffer(GL_ARRAY_BUFFER, mesh.instance_vbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(instances.size() * sizeof(Instance)), instances.data(), GL_STREAM_DRAW);
    glDrawElementsInstanced(GL_TRIANGLES, mesh.index_count, GL_UNSIGNED_INT, nullptr, GLsizei(instances.size()));
}

Renderer::Instance Renderer::sphere_instance(const Vec3& centre, double radius, const Quat& q, const float color[4],
                                             float emissive, float gloss, float pattern) {
    return box_instance(centre, Vec3(radius, radius, radius), q, color, emissive, gloss, pattern);
}

Renderer::Instance Renderer::box_instance(const Vec3& centre, const Vec3& half, const Quat& q, const float color[4],
                                          float emissive, float gloss, float pattern) {
    Vec3 c[3];
    if (q.is_identity()) { c[0] = Vec3(1, 0, 0); c[1] = Vec3(0, 1, 0); c[2] = Vec3(0, 0, 1); }
    else rotation_columns(q, c);
    Instance in;
    float* rows[3] = {in.row0, in.row1, in.row2};
    for (int r = 0; r < 3; ++r) {
        rows[r][0] = float(c[0][r] * half.x); rows[r][1] = float(c[1][r] * half.y); rows[r][2] = float(c[2][r] * half.z);
        rows[r][3] = float(centre[r]);
    }
    for (int k = 0; k < 4; ++k) in.color[k] = color[k];
    in.params[0] = emissive; in.params[1] = gloss; in.params[2] = pattern; in.params[3] = 0.0f;
    return in;
}

Renderer::Instance Renderer::rod_instance(const Vec3& from, const Vec3& to, double radius, const float color[4], float emissive) {
    const Vec3 axis = to - from;
    const double length = axis.length();
    const Vec3 y = length > 1e-9 ? axis / length : Vec3(0, 1, 0);
    const Vec3 helper = std::abs(y.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    const Vec3 x = y.cross(helper).normalized();
    const Vec3 z = x.cross(y);
    const Vec3 c[3] = {x * radius, y * length, z * radius};
    Instance in;
    float* rows[3] = {in.row0, in.row1, in.row2};
    for (int r = 0; r < 3; ++r) {
        rows[r][0] = float(c[0][r]); rows[r][1] = float(c[1][r]); rows[r][2] = float(c[2][r]);
        rows[r][3] = float(from[r]);
    }
    for (int k = 0; k < 4; ++k) in.color[k] = color[k];
    in.params[0] = emissive; in.params[1] = 0.15f; in.params[2] = 0.0f; in.params[3] = 0.0f;
    return in;
}

void Renderer::collect(const World& world, const PhysicsObject* highlight, double time_seconds) {
    spheres_.clear(); particles_.clear(); boxes_.clear(); rods_.clear();
    glass_boxes_.clear(); glass_spheres_.clear(); gas_.clear();

    for (const auto& ptr : world.objects) {
        const PhysicsObject& o = *ptr;
        if (!is_valid(o.position) || !is_valid(o.radius) || !is_valid(o.orientation)) continue;
        float color[4];
        set_color(color, o.color);
        float emissive = (&o == highlight) ? 0.22f : 0.0f;

        if (o.is_gas) {
            thermal_tint(color, o.temperature);
            gas_.push_back(sphere_instance(o.position, std::max(o.radius * 1.6, 2.6), Quat(), color, 0.35f, 0.0f, 0.0f));
            continue;
        }
        if (o.is_fluid) {
            thermal_tint(color, std::round(o.temperature / 5.0) * 5.0);
            particles_.push_back(sphere_instance(o.position, 6.2, Quat(), color, emissive, 0.55f, 0.0f));
            continue;
        }
        if (o.is_ice) {
            particles_.push_back(sphere_instance(o.position, 6.0, Quat(), color, emissive + 0.08f, 0.8f, 0.0f));
            continue;
        }

        const bool glass = o.color.a < 250;
        if (o.is_heater || o.is_cooler) emissive += 0.55f;
        else if (!o.is_fixture && !o.is_container_piston) thermal_tint(color, o.temperature);
        if (o.fuse > 0) {
            // A lit fuse flashes faster as it runs down.
            const double rate = 3.0 + 9.0 * (1.0 - o.fuse / 120.0);
            if (std::sin(time_seconds * rate * 2.0 * kPi) > 0.0) { color[0] = 0.95f; color[1] = 0.2f; color[2] = 0.15f; emissive += 0.6f; }
        }
        const float gloss = o.material == Material::Steel || glass ? 0.7f : (o.material == Material::Foam ? 0.05f : 0.3f);
        if (o.shape_type == ShapeType::BOX) {
            const float pattern = (o.is_static || glass) ? 0.0f : 2.0f;
            (glass ? glass_boxes_ : boxes_).push_back(box_instance(o.position, o.half_extents, o.orientation, color, emissive, gloss, pattern));
        } else {
            const float pattern = (o.can_rotate() && o.radius >= 8.0 && !glass) ? 1.0f : 0.0f;
            (glass ? glass_spheres_ : spheres_).push_back(sphere_instance(o.position, o.radius, o.orientation, color, emissive, gloss, pattern));
        }
    }

    // ---- Links ----
    const float spring_color[4] = {0.78f, 0.80f, 0.84f, 1.0f};
    const float link_color[4] = {0.72f, 0.60f, 0.40f, 1.0f};
    const float rope_color[4] = {0.82f, 0.82f, 0.86f, 1.0f};
    const float rod_color[4] = {1.0f, 0.55f, 0.20f, 1.0f};
    for (const auto& s : world.springs) rods_.push_back(rod_instance(s.end_a(), s.end_b(), 1.6, spring_color, 0.0f));
    for (const auto& l : world.rigid_links)
        if (!l.ice_bond) rods_.push_back(rod_instance(l.end_a(), l.end_b(), 2.2, link_color, 0.0f));
    for (const auto& p : world.pulleys) {
        rods_.push_back(rod_instance(p.obj_a->position, p.anchor, 1.2, rope_color, 0.0f));
        rods_.push_back(rod_instance(p.obj_b->position, p.anchor, 1.2, rope_color, 0.0f));
        spheres_.push_back(sphere_instance(p.anchor, 7.0, Quat(), rope_color, 0.0f, 0.6f, 0.0f));
    }
    // Pistons: a thick barrel anchored on obj_b and a thinner rod to obj_a.
    for (const auto& l : world.pistons) {
        const Vec3 delta = l.obj_a->position - l.obj_b->position;
        const double dist = delta.length();
        if (dist < 1e-6) continue;
        float barrel[4] = {0.47f, 0.49f, 0.53f, 1.0f};
        thermal_tint(barrel, l.gas_temperature);
        rods_.push_back(rod_instance(l.obj_b->position, l.obj_a->position, 2.0, rod_color, 0.0f));
        rods_.push_back(rod_instance(l.obj_b->position, l.obj_b->position + delta / dist * std::min(dist, l.min_len), 5.5, barrel, 0.0f));
    }
    // Drawn shapes: a tube through the beads.
    for (const auto& c : world.clusters) {
        const std::size_t n = c.nodes.size();
        for (std::size_t i = 0; i < n; ++i) {
            const PhysicsObject* a = c.nodes[i];
            const PhysicsObject* b = c.nodes[(i + 1) % n];
            float color[4];
            set_color(color, a->color);
            rods_.push_back(rod_instance(a->position, b->position, a->radius * 0.8, color, 0.0f));
        }
    }
    // Vessels: the piston's guide rod.
    for (const auto& c : world.containers) {
        if (!c.piston) continue;
        const Vec3 p = c.piston->position;
        rods_.push_back(rod_instance(p, p + Vec3(0, 26, 0), 3.0, rod_color, 0.0f));
    }
}

void Renderer::draw_room(const World& world, const Camera& camera, const QMatrix4x4& view_proj) {
    const float w = float(world.width), h = float(world.height), d = float(world.depth);
    const Vec3 eye = camera.eye();
    std::vector<float> v;
    auto quad = [&](const float p[4][3], const float n[3], bool visible) {
        if (!visible) return;
        const int order[6] = {0, 1, 2, 0, 2, 3};
        for (int k : order) v.insert(v.end(), {p[k][0], p[k][1], p[k][2], n[0], n[1], n[2]});
    };
    // Only the faces seen from inside-out are drawn, so the walls between
    // the camera and the scene never hide it.
    { const float p[4][3] = {{0, 0, 0}, {w, 0, 0}, {w, 0, d}, {0, 0, d}}; const float n[3] = {0, 1, 0}; quad(p, n, eye.y > 0.0); }
    { const float p[4][3] = {{0, h, 0}, {w, h, 0}, {w, h, d}, {0, h, d}}; const float n[3] = {0, -1, 0}; quad(p, n, eye.y > h); }
    { const float p[4][3] = {{0, 0, 0}, {0, h, 0}, {0, h, d}, {0, 0, d}}; const float n[3] = {1, 0, 0}; quad(p, n, eye.x > 0.0); }
    { const float p[4][3] = {{w, 0, 0}, {w, h, 0}, {w, h, d}, {w, 0, d}}; const float n[3] = {-1, 0, 0}; quad(p, n, eye.x < w); }
    { const float p[4][3] = {{0, 0, 0}, {w, 0, 0}, {w, h, 0}, {0, h, 0}}; const float n[3] = {0, 0, 1}; quad(p, n, eye.z > 0.0); }
    { const float p[4][3] = {{0, 0, d}, {w, 0, d}, {w, h, d}, {0, h, d}}; const float n[3] = {0, 0, -1}; quad(p, n, eye.z < d); }
    if (v.empty()) return;

    room_program_->bind();
    room_program_->setUniformValue("u_view_proj", view_proj);
    room_program_->setUniformValue("u_light", QVector3D(0.33f, 0.86f, 0.39f).normalized());
    glBindVertexArray(room_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, room_vbo_);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(v.size() * sizeof(float)), v.data(), GL_STREAM_DRAW);
    glDrawArrays(GL_TRIANGLES, 0, GLsizei(v.size() / 6));
}

void Renderer::draw(const World& world, const Camera& camera, int width_px, int height_px,
                    const PhysicsObject* highlight, double time_seconds) {
    if (!ready_ || width_px <= 0 || height_px <= 0) return;
    collect(world, highlight, time_seconds);

    const QMatrix4x4 view_proj = camera.projection(double(width_px) / double(height_px)) * camera.view();
    const QVector3D light = QVector3D(0.33f, 0.86f, 0.39f).normalized();
    const Vec3 eye = camera.eye();

    glViewport(0, 0, width_px, height_px);
    glDisable(GL_SCISSOR_TEST);
    glClearColor(0.085f, 0.09f, 0.11f, 1.0f);
    glClearStencil(0);
    glDepthMask(GL_TRUE);
    glStencilMask(0xff);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glDisable(GL_STENCIL_TEST);

    draw_room(world, camera, view_proj);

    body_program_->bind();
    body_program_->setUniformValue("u_view_proj", view_proj);
    body_program_->setUniformValue("u_light", light);
    body_program_->setUniformValue("u_eye", QVector3D(float(eye.x), float(eye.y), float(eye.z)));
    body_program_->setUniformValue("u_room", QVector3D(float(world.width), float(world.height), float(world.depth)));

    // ---- Shadows on the floor: each pixel is darkened once (stencil). ----
    if (eye.y > 0.0) {
        body_program_->setUniformValue("u_shadow", 1);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_EQUAL, 0, 0xff);
        glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
        glDepthMask(GL_FALSE);
        draw_instances(sphere_, spheres_);
        draw_instances(particle_, particles_);
        draw_instances(cube_, boxes_);
        draw_instances(rod_, rods_);
        glDepthMask(GL_TRUE);
        glDisable(GL_STENCIL_TEST);
        glDisable(GL_BLEND);
    }

    // ---- Opaque bodies ----
    body_program_->setUniformValue("u_shadow", 0);
    draw_instances(sphere_, spheres_);
    draw_instances(particle_, particles_);
    draw_instances(cube_, boxes_);
    draw_instances(rod_, rods_);

    // ---- Translucent: gas, then glass ----
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    draw_instances(particle_, gas_);
    draw_instances(sphere_, glass_spheres_);
    draw_instances(cube_, glass_boxes_);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    body_program_->release();
    glDisable(GL_DEPTH_TEST);
    glStencilFunc(GL_ALWAYS, 0, 0xff);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
}
