#pragma once
// OpenGL drawing of the World: instanced spheres, boxes and rods with
// per-pixel lighting, shadows cast on the floor and a gridded room.

#include "camera.h"
#include "world.h"

#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>

#include <memory>
#include <vector>

class Renderer : protected QOpenGLExtraFunctions {
public:
    // Call with the GL context current.
    bool initialize();
    void release();
    bool ready() const { return ready_; }
    QString error() const { return error_; }

    // 'highlight' (may be null) is drawn brighter; 'time' animates fuses.
    void draw(const World& world, const Camera& camera, int width_px, int height_px,
              const PhysicsObject* highlight, double time_seconds);

private:
    struct Instance {
        float row0[4], row1[4], row2[4];   // rows of the 3 × 4 model matrix
        float color[4];
        float params[4];                   // emissive, gloss, pattern (0 none, 1 ball, 2 box), unused
    };
    struct Mesh {
        unsigned vao = 0, vbo = 0, ibo = 0, instance_vbo = 0;
        int index_count = 0;
    };

    Mesh make_mesh(const std::vector<float>& vertices, const std::vector<unsigned>& indices);
    void draw_instances(const Mesh& mesh, const std::vector<Instance>& instances);
    void draw_room(const World& world, const Camera& camera, const QMatrix4x4& view_proj);
    void collect(const World& world, const PhysicsObject* highlight, double time_seconds);

    static Instance sphere_instance(const Vec3& centre, double radius, const Quat& q, const float color[4],
                                    float emissive, float gloss, float pattern);
    static Instance box_instance(const Vec3& centre, const Vec3& half, const Quat& q, const float color[4],
                                 float emissive, float gloss, float pattern);
    static Instance rod_instance(const Vec3& from, const Vec3& to, double radius, const float color[4], float emissive);

    bool ready_ = false;
    QString error_;
    std::unique_ptr<QOpenGLShaderProgram> body_program_, room_program_;
    Mesh sphere_, particle_, cube_, rod_;
    unsigned room_vbo_ = 0, room_vao_ = 0;

    // Per-frame batches.
    std::vector<Instance> spheres_, particles_, boxes_, rods_;
    std::vector<Instance> glass_boxes_, glass_spheres_, gas_;
};
