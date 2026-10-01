#include <QApplication>
#include <QWidget>
#include <QMainWindow>
#include <QPainter>
#include <QTimer>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QSlider>
#include <QStackedWidget>
#include <QInputDialog>
#include <QColor>
#include <QPen>
#include <QBrush>
#include <QPolygonF>
#include <cmath>
#include <vector>
#include <memory>
#include <random>
#include <algorithm>

enum class ShapeType { CIRCLE, SQUARE, TRIANGLE };
enum class ObjectType { BALL, BOX, SPRING, PULLEY, WHEEL, MOTOR, CHASSIS, PIN, ROPE, WATER, POLYGON,GAS,BOMB };

double random_double(double min, double max) {
    static std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> dist(min, max);
    return dist(gen);
}

int random_int(int min, int max) {
    static std::mt19937 gen(std::random_device{}());
    std::uniform_int_distribution<int> dist(min, max);
    return dist(gen);
}

struct Vector {
    double x, y, z;
    Vector(double _x = 0.0, double _y = 0.0, double _z = 0.0) : x(_x), y(_y), z(_z) {}

    Vector operator+(const Vector& o) const { return Vector(x + o.x, y + o.y, z + o.z); }
    Vector operator-(const Vector& o) const { return Vector(x - o.x, y - o.y, z - o.z); }
    Vector operator*(double s) const { return Vector(x * s, y * s, z * s); }
    Vector operator/(double s) const { return Vector(x / s, y / s, z / s); }

    Vector& operator+=(const Vector& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vector& operator-=(const Vector& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    double length() const { return std::hypot(x, y, z); }
    double dot(const Vector& o) const { return x * o.x + y * o.y + z * o.z; }
    QPointF to_qpointf() const { return QPointF(x, y); }
};

bool is_valid(const Vector& val) {
    return !(std::isnan(val.x) || std::isnan(val.y) || std::isinf(val.x) || std::isinf(val.y));
}
bool is_valid(double val) {
    return !(std::isnan(val) || std::isinf(val));
}
class PhysicsObject {
public:
    Vector position;
    Vector velocity;
    Vector acceleration;
    double radius;
    double elasticity;
    double mass;
    QColor color;
    ShapeType shape_type;
    bool is_static;
    bool is_fluid;
    bool is_dragged = false;
    bool is_gas = false;
    bool is_motor = false;
    bool is_bouncy = true;
    double angular_velocity = 0.0;
    int fuse = -1;
    std::vector<QPointF> trail;

    PhysicsObject(double x, double y, double r = 18.0, double el = 0.8, double m = -1.0,
                  QColor c = QColor(), ShapeType st = ShapeType::CIRCLE,
                  bool is_stat = false, bool is_fl = false)
    {
        position = Vector(x, y, 0.0);
        velocity = is_stat ? Vector(0,0,0) : Vector(random_double(-2.0, 2.0), random_double(-2.0, 1.0), 0.0);
        acceleration = Vector(0,0,0);
        radius = r;
        elasticity = el;
        shape_type = st;
        is_static = is_stat;
        is_fluid = is_fl;
        mass = (m < 0) ? (r * r) / 100.0 : m;

        if (is_fluid) {
            color = QColor(60, 150, 255, 200);
        } else {
            color = c.isValid() ? c : QColor(random_int(80, 255), random_int(80, 255), random_int(80, 255));
        }
    }

    void apply_force(const Vector& force) {
        if (!is_static) acceleration += force;
    }

    void update(const Vector& gravity, double air_resistance, double bounds_width, double bounds_height) {
        if (is_static) {
            velocity = Vector(0,0,0);
            return;
        }

        if (is_dragged) {
            acceleration = Vector(0,0,0);
            if (!is_fluid) update_trail();
            return;
        }

        if (is_motor) velocity.x += angular_velocity;
        if (is_gas) {
            velocity.x += random_double(-0.2, 0.2);
        } else {
            apply_force(gravity);
        }
        velocity += acceleration;

        double safe_drag = std::max(0.0, std::min(air_resistance, 0.99));
        velocity = velocity * (1.0 - safe_drag);

        double max_speed = 150.0;
        double current_speed = velocity.length();
        if (current_speed > max_speed) {
            velocity = velocity * (max_speed / current_speed);
        }

        position += velocity;
        acceleration = Vector(0, 0, 0);

        handle_wall_collisions(bounds_width, bounds_height);
        if (!is_fluid) update_trail();
    }

    void update_trail() {
        trail.push_back(position.to_qpointf());
        if (trail.size() > 12) trail.erase(trail.begin());
    }

private:
    void handle_wall_collisions(double width, double height) {
        if (position.y + radius >= height) {
            position.y = height - radius;
            velocity.y *= -elasticity;
            velocity.x *= 0.98;
        } else if (position.y - radius <= 0) {
            position.y = radius;
            velocity.y *= -elasticity;
        }

        if (position.x + radius >= width) {
            position.x = width - radius;
            velocity.x *= -elasticity;
        } else if (position.x - radius <= 0) {
            position.x = radius;
            velocity.x *= -elasticity;
        }
    }
};
class Spring {
public:
    PhysicsObject* obj_a;
    PhysicsObject* obj_b;
    double length;
    double stiffness;
    double damping;

    Spring(PhysicsObject* a, PhysicsObject* b, double l, double s = 0.1, double d = 0.05)
        : obj_a(a), obj_b(b), length(l), stiffness(s), damping(d) {}

    void update() {
        Vector delta_pos = obj_a->position - obj_b->position;
        double distance = delta_pos.length();
        if (distance == 0) return;

        double force_mag = (distance - length) * stiffness;
        Vector force_dir = delta_pos * (1.0 / distance);
        Vector rel_vel = obj_a->velocity - obj_b->velocity;
        Vector damping_force = force_dir * (rel_vel.dot(force_dir) * damping);
        Vector total_force = (force_dir * force_mag) + damping_force;

        if (!obj_a->is_static) obj_a->apply_force(total_force * -1.0);
        if (!obj_b->is_static) obj_b->apply_force(total_force);
    }
};

class RigidLink {
public:
    PhysicsObject* obj_a;
    PhysicsObject* obj_b;
    double length;

    RigidLink(PhysicsObject* a, PhysicsObject* b, double l)
        : obj_a(a), obj_b(b), length(l) {}

    void update() {
        Vector delta = obj_b->position - obj_a->position;
        double dist = delta.length();
        if (dist == 0) return;

        double error = dist - length;
        Vector direction = delta * (1.0 / dist);

        double inv_mass_a = obj_a->is_static ? 0 : 1.0 / obj_a->mass;
        double inv_mass_b = obj_b->is_static ? 0 : 1.0 / obj_b->mass;
        double sum_mass = inv_mass_a + inv_mass_b;
        if (sum_mass == 0) return;

        double correction = error / sum_mass;

        if (!obj_a->is_static) obj_a->position += direction * (correction * inv_mass_a);
        if (!obj_b->is_static) obj_b->position -= direction * (correction * inv_mass_b);
    }
};

class Pulley {
public:
    PhysicsObject* obj_a;
    PhysicsObject* obj_b;
    Vector anchor;
    double total_length;
    double stiffness;

    Pulley(PhysicsObject* a, PhysicsObject* b, Vector anc, double s = 0.5)
        : obj_a(a), obj_b(b), anchor(anc), stiffness(s) {
        total_length = (a->position - anchor).length() + (b->position - anchor).length();
    }

    void update() {
        double dist_a = (obj_a->position - anchor).length();
        double dist_b = (obj_b->position - anchor).length();
        double current_total = dist_a + dist_b;
        if (current_total == 0) return;

        double diff = current_total - total_length;
        if (diff > 0) {
            double force_mag = diff * stiffness;
            Vector dir_a = dist_a > 0 ? (anchor - obj_a->position) * (1.0 / dist_a) : Vector(0,0);
            Vector dir_b = dist_b > 0 ? (anchor - obj_b->position) * (1.0 / dist_b) : Vector(0,0);
            if (!obj_a->is_static) obj_a->apply_force(dir_a * force_mag);
            if (!obj_b->is_static) obj_b->apply_force(dir_b * force_mag);
        }
    }
};

// ==========================================
// PHYSICS CANVAS
// ==========================================

class PhysicsCanvas : public QWidget {
    Q_OBJECT
public:
    Vector gravity = Vector(0.0, 0.3, 0.0);
    double air_resistance = 0.01;
    double elasticity = 0.8;

    std::vector<std::shared_ptr<PhysicsObject>> balls;
    std::vector<Spring> springs;
    std::vector<Pulley> pulleys;
    std::vector<RigidLink> rigid_links;

    PhysicsObject* dragged_ball = nullptr;
    QPointF last_mouse_pos;
    bool has_last_mouse = false;
    Vector mouse_velocity = Vector(0,0,0);
    PhysicsObject* connection_start_obj = nullptr;

    ObjectType active_tool = ObjectType::BALL;
    bool is_paused = false;
    std::vector<QPointF> drawing_points;
    QTimer timer;

    PhysicsCanvas(QWidget* parent = nullptr) : QWidget(parent) {
        connect(&timer, &QTimer::timeout, this, &PhysicsCanvas::game_loop);
        timer.start(16);
    }

    void restart(int num_balls) {
        balls.clear(); springs.clear(); pulleys.clear(); rigid_links.clear();
        dragged_ball = nullptr; connection_start_obj = nullptr;
        update();
    }

    void set_tool(ObjectType tool_type) { active_tool = tool_type; }

    void remove_object(PhysicsObject* obj) {
        // Erase constraints referencing this pointer
        springs.erase(std::remove_if(springs.begin(), springs.end(),
            [obj](const Spring& s) { return s.obj_a == obj || s.obj_b == obj; }), springs.end());

        rigid_links.erase(std::remove_if(rigid_links.begin(), rigid_links.end(),
            [obj](const RigidLink& l) { return l.obj_a == obj || l.obj_b == obj; }), rigid_links.end());

        pulleys.erase(std::remove_if(pulleys.begin(), pulleys.end(),
            [obj](const Pulley& p) { return p.obj_a == obj || p.obj_b == obj; }), pulleys.end());

        // Erase the object itself
        balls.erase(std::remove_if(balls.begin(), balls.end(),
            [obj](const std::shared_ptr<PhysicsObject>& b) { return b.get() == obj; }), balls.end());
    }

    PhysicsObject* spawn_at(double x, double y, ShapeType shape, double radius=18.0, double mass=-1.0,
                            bool is_static=false, bool is_fluid=false) {
        auto obj = std::make_shared<PhysicsObject>(x, y, radius, elasticity, mass, QColor(), shape, is_static, is_fluid);
        if (is_fluid) obj->elasticity = 0.1;
        balls.push_back(obj);
        return obj.get();
    }

protected:
    void mousePressEvent(QMouseEvent* event) override {
        QPointF pos = event->position();
        Vector v_pos(pos.x(), pos.y());
        last_mouse_pos = pos;
        has_last_mouse = true;

        if (event->button() == Qt::RightButton) {
            if (active_tool == ObjectType::POLYGON && !drawing_points.empty()) {
                finalize_polygon();
                return;
            }
            for (auto it = balls.rbegin(); it != balls.rend(); ++it) {
                if ((v_pos - (*it)->position).length() <= (*it)->radius) {
                    remove_object((*it).get());
                    break;
                }
            }
            return;
        }

        if (event->button() == Qt::LeftButton) {
            if (active_tool == ObjectType::POLYGON) {
                if (!drawing_points.empty() &&
                    Vector(pos.x() - drawing_points[0].x(), pos.y() - drawing_points[0].y()).length() < 15) {
                    finalize_polygon();
                } else {
                    drawing_points.push_back(pos);
                }
                return;
            }

            PhysicsObject* clicked_obj = nullptr;
            for (auto it = balls.rbegin(); it != balls.rend(); ++it) {
                if ((v_pos - (*it)->position).length() <= (*it)->radius) {
                    clicked_obj = (*it).get();
                    break;
                }
            }

            if (clicked_obj) {
                if (active_tool == ObjectType::SPRING || active_tool == ObjectType::PULLEY) {
                    connection_start_obj = clicked_obj;
                } else {
                    dragged_ball = clicked_obj;
                    clicked_obj->is_dragged = true;
                    clicked_obj->velocity = Vector(0,0,0);
                }
            } else {
                handle_spawning(pos.x(), pos.y());
            }
        }
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        QPointF pos = event->position();
        if (has_last_mouse) {
            mouse_velocity = Vector(pos.x() - last_mouse_pos.x(), pos.y() - last_mouse_pos.y());
        }
        last_mouse_pos = pos;
        has_last_mouse = true;

        if (dragged_ball) {
            dragged_ball->position.x = pos.x();
            dragged_ball->position.y = pos.y();
            dragged_ball->velocity = Vector(0,0,0);
        }
    }

    void mouseReleaseEvent(QMouseEvent* event) override {
        if (event->button() == Qt::LeftButton) {
            if (dragged_ball) {
                dragged_ball->is_dragged = false;
                if (!is_paused) dragged_ball->velocity = mouse_velocity * 0.8;
                dragged_ball = nullptr;
            } else if (connection_start_obj) {
                QPointF pos = event->position();
                for (auto it = balls.rbegin(); it != balls.rend(); ++it) {
                    PhysicsObject* ball = (*it).get();
                    if (ball != connection_start_obj && (Vector(pos.x(), pos.y()) - ball->position).length() <= ball->radius) {
                        if (active_tool == ObjectType::SPRING) {
                            double dist = (connection_start_obj->position - ball->position).length();
                            springs.emplace_back(connection_start_obj, ball, std::max(dist, 10.0));
                        } else if (active_tool == ObjectType::PULLEY) {
                            double anchor_x = (connection_start_obj->position.x + ball->position.x) / 2.0;
                            double anchor_y = std::min(connection_start_obj->position.y, ball->position.y) - 100.0;
                            pulleys.emplace_back(connection_start_obj, ball, Vector(anchor_x, std::max(10.0, anchor_y)));
                        }
                        break;
                    }
                }
                connection_start_obj = nullptr;
            }
            has_last_mouse = false;
        }
    }

    void paintEvent(QPaintEvent* event) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(), QColor(30, 32, 38));

        if (!drawing_points.empty()) {
            painter.setPen(QPen(QColor(100, 255, 100), 2, Qt::DashLine));
            for (size_t i = 0; i < drawing_points.size() - 1; ++i) {
                painter.drawLine(drawing_points[i], drawing_points[i+1]);
            }
            if (has_last_mouse) painter.drawLine(drawing_points.back(), last_mouse_pos);
            painter.setBrush(QBrush(QColor(100, 255, 100)));
            painter.drawEllipse(drawing_points[0], 5, 5);
        }

        painter.setPen(QPen(QColor(200, 200, 200, 150), 2));
        for (const auto& p : pulleys) {
            if (is_valid(p.obj_a->position) && is_valid(p.obj_b->position)) {
                painter.drawLine(p.obj_a->position.to_qpointf(), p.anchor.to_qpointf());
                painter.drawLine(p.obj_b->position.to_qpointf(), p.anchor.to_qpointf());
                painter.setBrush(QBrush(QColor(255, 255, 255)));
                painter.drawEllipse(p.anchor.to_qpointf(), 5, 5);
            }
        }

        painter.setPen(QPen(QColor(150, 150, 150), 3));
        for (const auto& s : springs) {
            if (is_valid(s.obj_a->position) && is_valid(s.obj_b->position))
                painter.drawLine(s.obj_a->position.to_qpointf(), s.obj_b->position.to_qpointf());
        }

        painter.setPen(QPen(QColor(180, 150, 100), 4));
        for (const auto& l : rigid_links) {
            if (is_valid(l.obj_a->position) && is_valid(l.obj_b->position))
                painter.drawLine(l.obj_a->position.to_qpointf(), l.obj_b->position.to_qpointf());
        }

        if (connection_start_obj && has_last_mouse) {
            painter.setPen(QPen(QColor(100, 200, 255, 200), 2, Qt::DashLine));
            painter.drawLine(connection_start_obj->position.to_qpointf(), last_mouse_pos);
        }

        for (const auto& obj_ptr : balls) {
            PhysicsObject* obj = obj_ptr.get();
            if (!is_valid(obj->position) || !is_valid(obj->radius)) continue;

            if (obj->trail.size() > 2 && !obj->is_static && !obj->is_fluid) {
                QPen t_pen(obj->color);
                for (size_t i = 1; i < obj->trail.size(); ++i) {
                    int opacity = int(255 * (float(i) / obj->trail.size()));
                    t_pen.setColor(QColor(obj->color.red(), obj->color.green(), obj->color.blue(), opacity));
                    t_pen.setWidth(std::max(2, int((obj->radius/2.0) * (float(i)/obj->trail.size()))));
                    painter.setPen(t_pen);
                    painter.drawLine(obj->trail[i-1], obj->trail[i]);
                }
            }

            if (obj->is_static) painter.setPen(QPen(QColor(255, 255, 255), 2));
            else if (obj->is_motor) painter.setPen(QPen(QColor(255, 255, 100), 3));
            else if (obj->is_fluid) painter.setPen(Qt::NoPen);
            else painter.setPen(Qt::NoPen);

            painter.setBrush(QBrush(obj->color));

            if (obj->shape_type == ShapeType::CIRCLE) {
                painter.drawEllipse(obj->position.to_qpointf(), obj->radius, obj->radius);
            } else if (obj->shape_type == ShapeType::SQUARE) {
                double r = obj->radius;
                painter.drawRect(QRectF(obj->position.x - r, obj->position.y - r, r * 2, r * 2));
            } else if (obj->shape_type == ShapeType::TRIANGLE) {
                double r = obj->radius;
                QPolygonF poly;
                poly << QPointF(obj->position.x, obj->position.y - r)
                     << QPointF(obj->position.x - r, obj->position.y + r)
                     << QPointF(obj->position.x + r, obj->position.y + r);
                painter.drawPolygon(poly);
            }
        }
    }

private:
    void handle_spawning(double x, double y) {
        if (active_tool == ObjectType::BALL) spawn_at(x, y, ShapeType::CIRCLE);
        else if (active_tool == ObjectType::BOX) spawn_at(x, y, ShapeType::SQUARE);
        else if (active_tool == ObjectType::WHEEL) spawn_at(x, y, ShapeType::CIRCLE, 25);
        else if (active_tool == ObjectType::PIN) {
            auto obj = spawn_at(x, y, ShapeType::SQUARE, 20, -1, true);
            obj->color = QColor(100, 100, 120);
        } else if (active_tool == ObjectType::MOTOR) {
            auto obj = spawn_at(x, y, ShapeType::CIRCLE, 20);
            obj->is_motor = true; obj->angular_velocity = 0.5; obj->color = QColor(255, 80, 80);
        } else if (active_tool == ObjectType::ROPE) {
                    std::vector<PhysicsObject*> nodes;
                    for (int i = 0; i < 12; ++i) {
                        auto node = spawn_at(x, y + (i * 15), ShapeType::CIRCLE, 6, 1.0);

                        // --- ADD THESE TWO LINES ---
                        node->elasticity = 0.0;     // Dead drop, no bounce
                        node->is_bouncy = false;    // Ignore the global slider
                        // ---------------------------

                        if (i == 0) { node->is_static = true; node->color = QColor(200, 200, 200); }
                        nodes.push_back(node);
                    }
                    for (size_t i = 0; i < nodes.size() - 1; ++i) {
                        rigid_links.emplace_back(nodes[i], nodes[i+1], 15.0);
                    }
        } else if (active_tool == ObjectType::WATER) {
            for (int i = 0; i < 30; ++i) {
                for (int j = 0; j < 30; ++j) {
                    auto node = spawn_at(x + i*10, y + j*10, ShapeType::CIRCLE, 0.6, 0.1, false, true);
                    node->is_bouncy = false; // Add this to keep water from bouncing
                }
            }
        } else if (active_tool == ObjectType::GAS) {
            // Spawn a burst of gas particles
            for (int i = 0; i < 400; ++i) {
                auto node = spawn_at(x + random_double(-15, 15), y + random_double(-15, 15), ShapeType::CIRCLE, 6.0, 0.5);
                node->is_gas = true;
                node->is_bouncy = true;
                node->color = QColor(150, 255, 100, 150);
            }
        } else if (active_tool == ObjectType::BOMB) {
            // Spawn a heavy, dark grey bomb with a 120-frame fuse (~2 seconds)
            auto obj = spawn_at(x, y, ShapeType::CIRCLE, 15, 10.0);
            obj->color = QColor(30, 30, 30);
            obj->fuse = 120;
        }
    }

    void finalize_polygon() {
        if (drawing_points.size() < 3) { drawing_points.clear(); return; }

        double cx = 0, cy = 0;
        for (const auto& p : drawing_points) { cx += p.x(); cy += p.y(); }
        cx /= drawing_points.size(); cy /= drawing_points.size();

        QColor poly_color(random_int(100, 255), random_int(100, 255), random_int(100, 255));

        auto center_node = spawn_at(cx, cy, ShapeType::CIRCLE, 8);
        center_node->color = poly_color;

        std::vector<PhysicsObject*> b_nodes;
        for (const auto& p : drawing_points) {
            auto node = spawn_at(p.x(), p.y(), ShapeType::CIRCLE, 6);
            node->color = poly_color;
            b_nodes.push_back(node);
        }

        int n = b_nodes.size();
        for (int i = 0; i < n; ++i) {
            auto n1 = b_nodes[i];
            auto n2 = b_nodes[(i + 1) % n];

            rigid_links.emplace_back(n1, n2, (n1->position - n2->position).length());
            rigid_links.emplace_back(n1, center_node, (n1->position - center_node->position).length());

            if (n > 3) {
                auto n3 = b_nodes[(i + 2) % n];
                rigid_links.emplace_back(n1, n3, (n1->position - n3->position).length());
            }
        }
        drawing_points.clear();
    }

    void resolve_collisions() {
        std::vector<PhysicsObject*> sorted_balls;
        sorted_balls.reserve(balls.size());
        for (auto& b : balls) sorted_balls.push_back(b.get());

        // Fast 1D Spatial Sort (Sweep and Prune)
        std::sort(sorted_balls.begin(), sorted_balls.end(), [](PhysicsObject* a, PhysicsObject* b) {
            return a->position.x < b->position.x;
        });

        int n = sorted_balls.size();
        for (int i = 0; i < n; ++i) {
            PhysicsObject* b1 = sorted_balls[i];
            for (int j = i + 1; j < n; ++j) {
                PhysicsObject* b2 = sorted_balls[j];

                if ((b2->position.x - b1->position.x) > (b1->radius + b2->radius + 20.0)) break;
                if (b1->is_static && b2->is_static) continue;

                Vector delta_pos = b1->position - b2->position;
                double distance = delta_pos.length();

                // FLUID SPH LOGIC
                if (b1->is_fluid && b2->is_fluid) {
                    double smoothing_radius = b1->radius * 2.5;
                    if (distance > 0 && distance < smoothing_radius) {
                        double q = 1.0 - (distance / smoothing_radius);
                        Vector normal = delta_pos * (1.0 / distance);

                        double pressure = (q * q) * 2.0;
                        double viscosity = 0.05;

                        Vector push = normal * pressure;
                        b1->velocity += push * (1.0 / b1->mass);
                        b2->velocity -= push * (1.0 / b2->mass);

                        Vector vel_diff = b2->velocity - b1->velocity;
                        b1->velocity += vel_diff * (viscosity * q);
                        b2->velocity -= vel_diff * (viscosity * q);

                        double core_radius = b1->radius * 1.5;
                        if (distance < core_radius) {
                            double overlap = core_radius - distance;
                            Vector pos_correction = normal * (overlap * 0.5);
                            b1->position += pos_correction;
                            b2->position -= pos_correction;
                        }
                    }
                }
                // SOLID RIGID BODY LOGIC
                else {
                    double min_distance = b1->radius + b2->radius;
                    if (distance > 0 && distance < min_distance) {
                        Vector normal = delta_pos * (1.0 / distance);
                        double overlap = min_distance - distance;

                        if (b1->is_static) b2->position -= normal * overlap;
                        else if (b2->is_static) b1->position += normal * overlap;
                        else {
                            double total_mass = b1->mass + b2->mass;
                            b1->position += normal * (overlap * (b2->mass / total_mass));
                            b2->position -= normal * (overlap * (b1->mass / total_mass));
                        }

                        double restitution = (b1->is_fluid || b2->is_fluid) ? 0.0 : std::min(b1->elasticity, b2->elasticity);
                        Vector delta_vel = b1->velocity - b2->velocity;
                        double vel_along_normal = delta_vel.dot(normal);

                        if (vel_along_normal < 0) {
                            double inv_mass1 = b1->is_static ? 0 : 1.0 / b1->mass;
                            double inv_mass2 = b2->is_static ? 0 : 1.0 / b2->mass;
                            double impulse_scalar = -(1.0 + restitution) * vel_along_normal / (inv_mass1 + inv_mass2);
                            Vector impulse = normal * impulse_scalar;

                            if (!b1->is_static) b1->velocity += impulse * inv_mass1;
                            if (!b2->is_static) b2->velocity -= impulse * inv_mass2;
                        }
                    }
                }
            }
        }
    }
    void detonate_bomb(PhysicsObject* bomb) {
            double blast_radius = 250.0; // How far the shockwave reaches
            double max_force = 1500.0;   // The power of the shockwave

            for (auto& obj_ptr : balls) {
                PhysicsObject* target = obj_ptr.get();
                if (target == bomb || target->is_static) continue;

                Vector delta = target->position - bomb->position;
                double distance = delta.length();

                // If the object is inside the blast radius
                if (distance > 0 && distance < blast_radius) {
                    // Calculate falloff: 1.0 at the center, 0.0 at the edge of the radius
                    double falloff = 1.0 - (distance / blast_radius);

                    // Direction of the push
                    Vector push_dir = delta * (1.0 / distance);

                    // Apply the force (square the falloff so it's more powerful at the center)
                    double applied_force = max_force * (falloff * falloff);

                    // Send the object flying! (Lighter objects fly faster)
                    target->velocity += push_dir * (applied_force / target->mass);
                }
            }
        }

        void game_loop() {
                if (!is_paused) {
                    for (auto& s : springs) s.update();
                    for (auto& p : pulleys) p.update();
                    for (int i = 0; i < 4; ++i) {
                        for (auto& l : rigid_links) l.update();
                    }
                    std::vector<PhysicsObject*> to_delete;
                    for (auto& b : balls) {
                        b->update(gravity, air_resistance, width(), height());

                        if (b->fuse > 0) {
                            b->fuse--;
                            if (b->fuse % 10 < 5) b->color = QColor(255, 255, 255);
                            else b->color = QColor(30, 30, 30);

                            if (b->fuse == 0) {
                                detonate_bomb(b.get());
                                to_delete.push_back(b.get()); // Mark for removal
                            }
                        }
                    }
                    for (auto* bomb : to_delete) {
                        remove_object(bomb);
                    }
                    // ---------------------------------

                    resolve_collisions();
                }
                update();
            }
};
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    QStackedWidget* stack;
    PhysicsCanvas* canvas;
    std::vector<std::pair<QPushButton*, ObjectType>> tool_buttons;
    QPushButton* pause_btn;
    QSlider *grav_slider, *elas_slider, *wind_slider;

    MainWindow() {
        setWindowTitle("Sapphire");
        resize(1000, 800);
        setStyleSheet("background-color: #282c34;");

        stack = new QStackedWidget(this);
        setCentralWidget(stack);

        // Intro Page
        QWidget* intro_page = new QWidget();
        QVBoxLayout* intro_layout = new QVBoxLayout(intro_page);
        intro_layout->setAlignment(Qt::AlignCenter);

        QLabel* title = new QLabel("Sapphire");
        title->setFont(QFont("Arial", 32, QFont::Bold));
        title->setStyleSheet("color: #61afef; margin-bottom: 20px;");
        intro_layout->addWidget(title, 0, Qt::AlignCenter);

        QPushButton* start_btn = new QPushButton("Enter Sandbox");
        start_btn->setFixedSize(300, 60);
        start_btn->setStyleSheet(
            "QPushButton { background-color: #3e4451; color: white; font-size: 18px; border-radius: 10px; }"
            "QPushButton:hover { background-color: #4b5263; }"
        );
        connect(start_btn, &QPushButton::clicked, this, &MainWindow::start_module);
        intro_layout->addWidget(start_btn, 0, Qt::AlignCenter);

        stack->addWidget(intro_page);
    }

private slots:
    void start_module() {
        bool ok;
        int num_balls = QInputDialog::getInt(this, "Initialization", "Initial object count:", 10, 0, 500, 1, &ok);
        if (ok) {
            setup_sandbox(num_balls);
            stack->setCurrentIndex(1);
        }
    }

    void setup_sandbox(int num_balls) {
        QWidget* sandbox_widget = new QWidget();
        QHBoxLayout* sandbox_layout = new QHBoxLayout(sandbox_widget);
        sandbox_layout->setContentsMargins(0,0,0,0);
        sandbox_layout->setSpacing(0);

        // Toolbox
        QWidget* toolbox = new QWidget();
        toolbox->setFixedWidth(160);
        toolbox->setStyleSheet("background-color: #21252b; border-right: 1px solid #333;");
        QVBoxLayout* tb_layout = new QVBoxLayout(toolbox);

        QLabel* l1 = new QLabel("<b style='color:#abb2bf; font-size:14px'>TOOLS</b>");
        tb_layout->addWidget(l1);

        std::vector<std::pair<QString, ObjectType>> tools = {
            {"Ball", ObjectType::BALL}, {"Box", ObjectType::BOX}, {"Pin (Static)", ObjectType::PIN},
            {"Draw Shape", ObjectType::POLYGON}, {"Gas Cloud", ObjectType::GAS},{"Rope Chain", ObjectType::ROPE}, {"Water Drop", ObjectType::WATER},
            {"Motor", ObjectType::MOTOR}, {"Wheel", ObjectType::WHEEL}, {"Car Chassis", ObjectType::CHASSIS},{"Shockwave Bomb", ObjectType::BOMB}
        };

        for (auto& t : tools) {
            QPushButton* btn = new QPushButton(t.first);
            btn->setCheckable(true);
            btn->setStyleSheet(
                "QPushButton { background-color: #3e4451; color: white; padding: 8px; border-radius: 4px; }"
                "QPushButton:checked { background-color: #61afef; color: black; font-weight: bold; }"
            );
            ObjectType t_type = t.second;
            connect(btn, &QPushButton::clicked, this, [this, t_type]() { select_tool(t_type); });
            tb_layout->addWidget(btn);
            tool_buttons.push_back({btn, t_type});
        }

        QLabel* l2 = new QLabel("<b style='color:#abb2bf; font-size:14px; margin-top:15px'>LINK TOOLS</b>");
        tb_layout->addWidget(l2);

        for (auto& t : std::vector<std::pair<QString, ObjectType>>{{"Bouncy Spring", ObjectType::SPRING}, {"Pulley", ObjectType::PULLEY}}) {
            QPushButton* btn = new QPushButton(t.first);
            btn->setCheckable(true);
            btn->setStyleSheet(
                "QPushButton { background-color: #d19a66; color: black; padding: 8px; border-radius: 4px; }"
                "QPushButton:checked { background-color: #e5c07b; font-weight: bold; }"
            );
            ObjectType t_type = t.second;
            connect(btn, &QPushButton::clicked, this, [this, t_type]() { select_tool(t_type); });
            tb_layout->addWidget(btn);
            tool_buttons.push_back({btn, t_type});
        }

        tb_layout->addStretch();

        pause_btn = new QPushButton("Pause Physics");
        pause_btn->setStyleSheet("background-color: #98c379; color: black; padding: 10px; font-weight: bold;");
        connect(pause_btn, &QPushButton::clicked, this, &MainWindow::toggle_pause);
        tb_layout->addWidget(pause_btn);

        QPushButton* clear_btn = new QPushButton("Clear All");
        clear_btn->setStyleSheet("background-color: #e06c75; color: white; padding: 10px; font-weight: bold;");
        connect(clear_btn, &QPushButton::clicked, this, [this]() { canvas->restart(0); });
        tb_layout->addWidget(clear_btn);

        sandbox_layout->addWidget(toolbox);

        // Right side (Canvas + Controls)
        QWidget* right_container = new QWidget();
        QVBoxLayout* right_layout = new QVBoxLayout(right_container);
        right_layout->setContentsMargins(0,0,0,0);
        right_layout->setSpacing(0);

        canvas = new PhysicsCanvas(this);
        right_layout->addWidget(canvas);

        QWidget* controls = new QWidget();
        controls->setStyleSheet("background-color: #1e1e24; color: white; border-top: 1px solid #333;");
        QHBoxLayout* ctrl_layout = new QHBoxLayout(controls);

        auto add_slider = [&](const QString& name, QSlider*& slider_ptr, int min_v, int max_v, int def_v) {
            QVBoxLayout* l = new QVBoxLayout();
            l->addWidget(new QLabel(name));
            slider_ptr = new QSlider(Qt::Horizontal);
            slider_ptr->setRange(min_v, max_v);
            slider_ptr->setValue(def_v);
            connect(slider_ptr, &QSlider::valueChanged, this, &MainWindow::update_physics_params);
            l->addWidget(slider_ptr);
            ctrl_layout->addLayout(l);
        };

        add_slider("Gravity", grav_slider, 0, 100, 30);
        add_slider("Bounciness", elas_slider, 10, 100, 80);
        add_slider("Wind Drag", wind_slider, 0, 50, 10);

        right_layout->addWidget(controls);
        sandbox_layout->addWidget(right_container);
        stack->addWidget(sandbox_widget);

        select_tool(ObjectType::BALL);
        for (int i = 0; i < num_balls; ++i) {
            canvas->spawn_at(random_double(50, 950), random_double(50, 400), ShapeType::CIRCLE);
        }
    }

    void select_tool(ObjectType type) {
        canvas->set_tool(type);
        for (auto& pair : tool_buttons) {
            pair.first->setChecked(pair.second == type);
        }
        if (type == ObjectType::CHASSIS) spawn_car();
    }

    void toggle_pause() {
        canvas->is_paused = !canvas->is_paused;
        if (canvas->is_paused) {
            pause_btn->setText("Resume Physics");
            pause_btn->setStyleSheet("background-color: #e5c07b; color: black; padding: 10px; font-weight: bold;");
        } else {
            pause_btn->setText("Pause Physics");
            pause_btn->setStyleSheet("background-color: #98c379; color: black; padding: 10px; font-weight: bold;");
        }
    }

    void spawn_car() {
            double cx = canvas->width() / 2.0;
            double cy = canvas->height() / 2.0;

            // 1. Create a 2-node rigid chassis (Front and Rear) for stability
            // We use squares to simulate the body frame
            auto chassis_rear = canvas->spawn_at(cx - 35, cy, ShapeType::SQUARE, 20, 6.0);
            auto chassis_front = canvas->spawn_at(cx + 35, cy, ShapeType::SQUARE, 20, 6.0);
            chassis_rear->color = QColor(100, 150, 255);
            chassis_front->color = QColor(100, 150, 255);

            // 2. Create the wheels
            auto w_rear = canvas->spawn_at(cx - 45, cy + 45, ShapeType::CIRCLE, 16, 3.0);
            auto w_front = canvas->spawn_at(cx + 45, cy + 45, ShapeType::CIRCLE, 16, 3.0);

            // Make the rear wheel a motor and paint it red
            w_rear->is_motor = true;
            w_rear->angular_velocity = 1.2; // Slightly higher torque to move the robust frame
            w_rear->color = QColor(255, 50, 50);

            // Paint front wheel dark grey
            w_front->color = QColor(50, 50, 50);

            // 3. Lock the chassis nodes together so the car body cannot bend
            double body_len = (chassis_rear->position - chassis_front->position).length();
            canvas->rigid_links.emplace_back(chassis_rear, chassis_front, body_len);

            // Helper lambda to auto-calculate spring rest-lengths based on spawn positions
            auto add_strut = [&](PhysicsObject* a, PhysicsObject* b, double stiffness) {
                double dist = (a->position - b->position).length();
                canvas->springs.emplace_back(a, b, dist, stiffness);
            };

            // 4. Vertical Suspension (Shocks)
            add_strut(chassis_rear, w_rear, 0.6);
            add_strut(chassis_front, w_front, 0.6);

            // 5. Cross-bracing (Keeps the chassis upright and prevents the parallelogram effect)
            add_strut(chassis_rear, w_front, 0.3);
            add_strut(chassis_front, w_rear, 0.3);

            // 6. Wheelbase strut (Prevents the motor from just pushing the rear wheel under the car)
            add_strut(w_rear, w_front, 0.5);
        }
        void update_physics_params() {
                canvas->gravity.y = grav_slider->value() / 100.0;
                canvas->air_resistance = wind_slider->value() / 1000.0;
                canvas->elasticity = elas_slider->value() / 100.0;

                for (auto& ball : canvas->balls) {
                    // ONLY update objects that are meant to be bouncy
                    if (ball->is_bouncy) {
                        ball->elasticity = canvas->elasticity;
                    }
                }
        }
        };
#include "main.moc"
int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}
