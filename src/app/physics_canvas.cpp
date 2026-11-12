#include "physics_canvas.h"

#include "random.h"

#include <QFont>
#include <QFontMetrics>
#include <QGuiApplication>
#include <QMouseEvent>
#include <QOpenGLWidget>
#include <QPainter>
#include <QPen>
#include <QSurfaceFormat>
#include <QWheelEvent>

#include <cmath>

namespace {
constexpr double kFrameMs = 1000.0 / 60.0;   // one World::step()
constexpr int kMaxStepsPerTick = 4;           // beyond this the sim slows down instead of stalling
constexpr std::size_t kHistoryLimit = 40;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMetresPerSecond = 0.6;      // 1 cm/frame at 60 frames/s
constexpr double kJoules = 0.36;              // 1 kg·cm²/frame²

QColor temp_color(double t) {
    if (t > 50.0) return QColor(255, 100, 100);
    if (t < 0.0) return QColor(100, 240, 255);
    return QColor(255, 255, 255);
}

QString kind_name(const PhysicsObject& o) {
    if (o.is_container_piston) return "Vessel piston";
    if (o.is_fixture) return "Glass wall";
    if (o.is_heater) return "Heater";
    if (o.is_cooler) return "Cooler";
    if (o.is_ice) return "Ice";
    if (o.is_steam) return "Steam";
    if (o.is_gas) return "Air";
    if (o.is_fluid) return "Water";
    if (o.fuse > 0) return "Bomb";
    const QString material = material_props(o.material).name;
    if (o.in_cluster) return material + " shape";
    if (o.is_motor) return "Motor wheel";
    if (o.is_static) return o.half_extents.x > 50.0 ? "Platform" : "Pin";
    return material + (o.shape_type == ShapeType::BOX ? " box" : " ball");
}

bool is_link_tool(ObjectType tool) {
    return tool == ObjectType::SPRING || tool == ObjectType::PULLEY || tool == ObjectType::PISTON;
}
bool spawns_on_click(ObjectType tool) { return tool != ObjectType::POLYGON && !is_link_tool(tool); }

QFont mono(int size, bool bold = false) {
    QFont f("Menlo", size, bold ? QFont::Bold : QFont::Normal);
    f.setStyleHint(QFont::Monospace);
    return f;
}

void draw_panel(QPainter& painter, const QRectF& rect) {
    painter.setBrush(QColor(20, 22, 28, 215));
    painter.setPen(QPen(QColor(100, 150, 255), 1));
    painter.drawRoundedRect(rect, 8, 8);
}
} // namespace

// The OpenGL surface.  Mouse events pass through it to the canvas.
class PhysicsCanvas::View : public QOpenGLWidget {
public:
    explicit View(PhysicsCanvas* canvas) : QOpenGLWidget(canvas), canvas_(canvas) {
        QSurfaceFormat format = QSurfaceFormat::defaultFormat();
        format.setVersion(3, 3);
        format.setProfile(QSurfaceFormat::CoreProfile);
        format.setDepthBufferSize(24);
        format.setStencilBufferSize(8);
        format.setSamples(4);
        setFormat(format);
        setAttribute(Qt::WA_TransparentForMouseEvents);
    }
    ~View() override {
        if (renderer_.ready()) {
            makeCurrent();
            renderer_.release();
            doneCurrent();
        }
    }

protected:
    void initializeGL() override { renderer_.initialize(); }
    void paintGL() override {
        PhysicsCanvas& c = *canvas_;
        c.frame_ms_ += (double(c.paint_clock_.nsecsElapsed()) / 1e6 - c.frame_ms_) * 0.05;
        c.paint_clock_.restart();

        // The 3D scene is raw OpenGL; the HUD on top of it is drawn by QPainter,
        // which has to be told when its GL state has been used by someone else.
        QPainter painter(this);
        const qreal dpr = devicePixelRatioF();
        if (renderer_.ready()) {
            painter.beginNativePainting();
            renderer_.draw(c.world, c.camera, int(width() * dpr), int(height() * dpr), c.world.dragged(),
                           double(c.run_clock_.elapsed()) / 1000.0);
            painter.endNativePainting();
        }
        painter.setRenderHint(QPainter::Antialiasing);
        if (!renderer_.ready()) {
            painter.fillRect(rect(), QColor(30, 32, 38));
            painter.setPen(QColor(229, 192, 123));
            painter.setFont(mono(12));
            painter.drawText(rect().adjusted(20, 20, -20, -20), Qt::AlignCenter | Qt::TextWordWrap,
                             "The 3D view needs OpenGL 3.3.\n" + renderer_.error());
        }
        c.draw_overlay(painter);
    }

private:
    PhysicsCanvas* canvas_;
    Renderer renderer_;
};

PhysicsCanvas::PhysicsCanvas(QWidget* parent) : QWidget(parent) {
    // Headless platforms have no OpenGL (and some Qt versions crash when an
    // OpenGL widget is created on them), so the view is left out there.
    const QString platform = QGuiApplication::platformName();
    if (platform != "offscreen" && platform != "minimal") view_ = new View(this);

    setMouseTracking(true);              // hover readout and previews
    setFocusPolicy(Qt::ClickFocus);
    connect(&timer_, &QTimer::timeout, this, &PhysicsCanvas::game_loop);
    timer_.setTimerType(Qt::PreciseTimer);
    timer_.start(8);
    clock_.start();
    run_clock_.start();
    paint_clock_.start();
    reset_view();
}

PhysicsCanvas::~PhysicsCanvas() = default;

void PhysicsCanvas::refresh() {
    if (view_) view_->update();
    else update();
}

QImage PhysicsCanvas::grab_view() {
    return (view_ && view_->isValid()) ? view_->grabFramebuffer() : QImage();
}

void PhysicsCanvas::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    if (view_) view_->setGeometry(rect());
}

// Only reached without the OpenGL view: the overlay on a plain background.
void PhysicsCanvas::paintEvent(QPaintEvent*) {
    if (view_) return;
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), QColor(22, 23, 28));
    painter.setPen(QColor(150, 158, 172));
    painter.setFont(mono(12));
    painter.drawText(rect(), Qt::AlignCenter, "No OpenGL on this platform: the 3D view is not drawn.");
    draw_overlay(painter);
}

void PhysicsCanvas::reset_view() {
    camera = Camera();
    camera.target = Vec3(0.5 * world.width, 0.38 * world.height, 0.5 * world.depth);
    camera.distance = 0.9 * std::max(world.width, 1.6 * world.height) + 0.6 * world.depth;
    refresh();
}

void PhysicsCanvas::restart(int initial_balls) {
    world.clear();
    reset_interaction();
    for (int i = 0; i < initial_balls; ++i)
        world.spawn_ball(Vec3(random_double(40, world.width - 40), random_double(0.3 * world.height, 0.8 * world.height),
                              random_double(40, world.depth - 40)));
    refresh();
}

// A scene that shows most of what the sandbox does.
void PhysicsCanvas::load_demo() {
    world.clear();
    reset_interaction();
    const Material chosen = world.spawn_material;

    world.spawn_tank(Vec3(190, 0, 250));
    world.spawn_water(Vec3(190, 60, 250));
    world.spawn_water(Vec3(190, 150, 250));
    world.add_sphere(Vec3(150, 260, 220), 16.0, Material::Wood);
    world.add_sphere(Vec3(230, 285, 270), 15.0, Material::Foam);
    world.add_sphere(Vec3(190, 310, 250), 11.0, Material::Steel);

    world.spawn_container(Vec3(700, 110, 300));
    world.spawn_heater(Vec3(700, 40, 300));

    world.spawn_platform(Vec3(480, 300, 130), -0.22);
    world.spawn_material = Material::Rubber;
    for (int k = 0; k < 4; ++k) world.spawn_ball(Vec3(400 + 30.0 * k, 370 + 12.0 * k, 110 + 15.0 * k));
    world.spawn_material = Material::Wood;
    for (int k = 0; k < 4; ++k) {
        PhysicsObject* box = world.spawn_box(Vec3(500, 18.5 + 36.5 * k, 210));
        box->velocity = box->angular_velocity = Vec3();
    }
    world.spawn_car(Vec3(120, 60, 440));
    world.spawn_rope(Vec3(560, 520, 330));
    world.spawn_material = chosen;
    refresh();
}

void PhysicsCanvas::set_paused(bool paused) {
    paused_ = paused;
    accumulator_ms_ = 0.0;
    clock_.restart();
    refresh();
}

// Fixed timestep: the World always advances in 1/60 s steps, however often
// the timer fires or however long painting takes.
void PhysicsCanvas::game_loop() {
    const double elapsed = double(clock_.nsecsElapsed()) / 1e6;
    clock_.restart();
    if (!paused_) {
        accumulator_ms_ += std::min(elapsed, 250.0) * time_scale_;
        int steps = 0;
        while (accumulator_ms_ >= kFrameMs && steps < kMaxStepsPerTick) {
            QElapsedTimer step_clock;
            step_clock.start();
            world.step();
            physics_ms_ += (double(step_clock.nsecsElapsed()) / 1e6 - physics_ms_) * 0.1;
            accumulator_ms_ -= kFrameMs;
            ++steps;
        }
        if (steps == kMaxStepsPerTick) accumulator_ms_ = 0.0;   // drop the backlog
        if (steps > 0) refresh();
    }
    // A link being drawn must not point at an object that has since been
    // removed (e.g. a bomb that went off).
    if (connection_start_ && world.index_of(connection_start_) < 0) connection_start_ = nullptr;
}

// ============================================================
// History
// ============================================================

void PhysicsCanvas::push_undo() {
    undo_stack_.push_back(world.serialize());
    if (undo_stack_.size() > kHistoryLimit) undo_stack_.erase(undo_stack_.begin());
    redo_stack_.clear();
    emit history_changed();
}

void PhysicsCanvas::restore(const std::string& snapshot) {
    reset_interaction();
    world.deserialize(snapshot);
    emit parameters_changed();
    emit history_changed();
    refresh();
}

bool PhysicsCanvas::undo() {
    if (undo_stack_.empty()) return false;
    redo_stack_.push_back(world.serialize());
    const std::string snapshot = std::move(undo_stack_.back());
    undo_stack_.pop_back();
    restore(snapshot);
    return true;
}

bool PhysicsCanvas::redo() {
    if (redo_stack_.empty()) return false;
    undo_stack_.push_back(world.serialize());
    const std::string snapshot = std::move(redo_stack_.back());
    redo_stack_.pop_back();
    restore(snapshot);
    return true;
}

bool PhysicsCanvas::load_scene(const std::string& text, QString* error) {
    const std::string before = world.serialize();
    std::string message;
    reset_interaction();
    if (!world.deserialize(text, &message)) {
        if (error) *error = QString::fromStdString(message);
        return false;
    }
    undo_stack_.push_back(before);
    if (undo_stack_.size() > kHistoryLimit) undo_stack_.erase(undo_stack_.begin());
    redo_stack_.clear();
    emit parameters_changed();
    emit history_changed();
    refresh();
    return true;
}

void PhysicsCanvas::reset_interaction() {
    if (world.dragged()) world.end_drag(Vec3());
    connection_start_ = nullptr;
    drawing_points_.clear();
    drag_samples_.clear();
}

// ============================================================
// Screen ↔ world
// ============================================================

Vec3 PhysicsCanvas::spawn_point(const QPointF& pixel) const {
    const Ray ray = ray_at(pixel);
    Vec3 normal = camera.forward();
    // Looking mostly sideways: a vertical plane facing the camera.  Looking
    // down: the horizontal plane at the target's height.
    if (std::abs(normal.y) < 0.87) { normal.y = 0.0; normal = normal.normalized(Vec3(0, 0, -1)); }
    else normal = Vec3(0, 1, 0);
    Vec3 hit;
    if (!intersect_plane(ray, camera.target, normal, hit)) hit = ray.at(camera.distance);
    return world.clamp_to_room(hit, 8.0);
}

bool PhysicsCanvas::screen_point(const Vec3& world_point, QPointF& pixel) const {
    return camera.project(world_point, width(), height(), pixel);
}

Vec3 PhysicsCanvas::drag_point(const QPointF& pixel) const {
    const Ray ray = ray_at(pixel);
    const Vec3 forward = camera.forward();
    Vec3 hit;
    if (!intersect_plane(ray, camera.eye() + forward * drag_depth_, forward, hit)) hit = ray.at(drag_depth_);
    return hit + drag_offset_;
}

Vec3 PhysicsCanvas::throw_velocity(qint64 now) const {
    // Average the held body's motion over the last ~60 ms.  If it has been
    // still for a moment before release, it is simply let go.  Times are the
    // input events' own timestamps, so a busy frame does not distort them.
    if (drag_samples_.size() < 2) return Vec3();
    const DragSample& last = drag_samples_.back();
    if (now - last.ms > 50) return Vec3();
    const DragSample* first = &drag_samples_.front();
    for (const auto& s : drag_samples_) {
        if (last.ms - s.ms <= 60) { first = &s; break; }
    }
    const double dt_ms = double(last.ms - first->ms);
    if (dt_ms < 1.0) return Vec3();
    Vec3 v = (last.position - first->position) * (kFrameMs / dt_ms);
    const double speed = v.length();
    if (speed > 60.0) v *= 60.0 / speed;
    return v;
}

// ============================================================
// Input
// ============================================================

void PhysicsCanvas::mousePressEvent(QMouseEvent* event) {
    const QPointF pos = event->position();
    mouse_pos_ = pos;
    mouse_inside_ = true;

    if (event->button() == Qt::RightButton || event->button() == Qt::MiddleButton) {
        if (camera_button_ == Qt::NoButton) {
            camera_button_ = event->button();
            camera_press_pos_ = camera_last_pos_ = pos;
            camera_moved_ = false;
        }
        return;
    }
    if (event->button() != Qt::LeftButton) return;

    if (active_tool_ == ObjectType::POLYGON) {
        QPointF first;
        if (!drawing_points_.empty() && screen_point(drawing_points_[0], first) &&
            std::hypot(pos.x() - first.x(), pos.y() - first.y()) < 15.0)
            finalize_polygon();
        else
            drawing_points_.push_back(spawn_point(pos));
        refresh();
        return;
    }

    const Ray ray = ray_at(pos);
    double hit_distance = 0.0;
    PhysicsObject* hit = world.pick(ray.origin, ray.direction, World::Pick::Grab, &hit_distance);
    // A platform or pin behind the place where the new object would go is
    // scenery, not something to grab: the click spawns in front of it.
    if (hit && hit->is_static && spawns_on_click(active_tool_) &&
        hit_distance > (spawn_point(pos) - ray.origin).length() + 5.0)
        hit = nullptr;
    if (hit && is_link_tool(active_tool_)) {
        connection_start_ = hit;
    } else if (hit && world.begin_drag(hit)) {
        // The body keeps its distance from the camera and stays under the cursor where it was grabbed.
        drag_depth_ = std::max(30.0, (hit->position - camera.eye()).dot(camera.forward()));
        drag_offset_ = Vec3();
        drag_offset_ = hit->position - drag_point(pos);
        drag_samples_.clear();
        drag_samples_.push_back({qint64(event->timestamp()), hit->position});
    } else if (!hit && spawns_on_click(active_tool_)) {
        push_undo();
        handle_spawning(spawn_point(pos));
    }
    refresh();
}

void PhysicsCanvas::mouseMoveEvent(QMouseEvent* event) {
    const QPointF pos = event->position();
    mouse_pos_ = pos;
    mouse_inside_ = true;

    if (camera_button_ != Qt::NoButton && (event->buttons() & camera_button_)) {
        const QPointF delta = pos - camera_last_pos_;
        if ((pos - camera_press_pos_).manhattanLength() > 4.0) camera_moved_ = true;
        if (camera_moved_) {
            if (camera_button_ == Qt::MiddleButton || (event->modifiers() & Qt::ShiftModifier))
                camera.pan(delta.x(), delta.y(), height());
            else
                camera.orbit(delta.x(), delta.y());
        }
        camera_last_pos_ = pos;
    }
    if (world.dragged()) {
        const Vec3 target = drag_point(pos);
        world.drag_to(target, paused_);
        if (event->buttons() & Qt::LeftButton) {
            drag_samples_.push_back({qint64(event->timestamp()), target});
            while (drag_samples_.size() > 16) drag_samples_.pop_front();
        }
    }
    refresh();   // hover readout, previews
}

void PhysicsCanvas::mouseReleaseEvent(QMouseEvent* event) {
    const QPointF pos = event->position();
    if (event->button() == camera_button_ && camera_button_ != Qt::NoButton) {
        const bool click = !camera_moved_;
        const Qt::MouseButton button = camera_button_;
        camera_button_ = Qt::NoButton;
        if (click && button == Qt::RightButton) {
            // A right-click (not a drag) closes the outline being drawn or removes what is under it.
            if (active_tool_ == ObjectType::POLYGON && !drawing_points_.empty()) {
                finalize_polygon();
            } else {
                const Ray ray = ray_at(pos);
                if (PhysicsObject* hit = world.pick(ray.origin, ray.direction, World::Pick::Remove)) {
                    push_undo();
                    if (connection_start_ == hit) connection_start_ = nullptr;
                    world.remove_object(hit);
                    if (connection_start_ && world.index_of(connection_start_) < 0) connection_start_ = nullptr;
                }
            }
        }
        refresh();
        return;
    }
    if (event->button() != Qt::LeftButton) return;

    if (world.dragged()) {
        world.end_drag(paused_ ? Vec3() : throw_velocity(qint64(event->timestamp())));
    } else if (connection_start_) {
        const Ray ray = ray_at(pos);
        PhysicsObject* target = world.pick(ray.origin, ray.direction, World::Pick::Grab);
        if (target && target != connection_start_) {
            push_undo();
            if (active_tool_ == ObjectType::SPRING) world.connect_spring(connection_start_, target);
            else if (active_tool_ == ObjectType::PISTON) world.connect_piston(connection_start_, target);
            else if (active_tool_ == ObjectType::PULLEY) world.connect_pulley(connection_start_, target);
        }
        connection_start_ = nullptr;
    }
    drag_samples_.clear();
    refresh();
}

void PhysicsCanvas::wheelEvent(QWheelEvent* event) {
    const double steps = event->angleDelta().y() / 120.0;
    if (steps == 0.0) return;
    if (world.dragged()) {
        // While holding a body the wheel pushes it away or pulls it closer.
        drag_depth_ = clampd(drag_depth_ + steps * 20.0, 30.0, 6000.0);
        world.drag_to(drag_point(mouse_pos_), paused_);
    } else {
        camera.zoom(steps);
    }
    event->accept();
    refresh();
}

void PhysicsCanvas::leaveEvent(QEvent* event) {
    QWidget::leaveEvent(event);
    mouse_inside_ = false;
    refresh();
}

void PhysicsCanvas::handle_spawning(const Vec3& p) {
    switch (active_tool_) {
    case ObjectType::BALL:      world.spawn_ball(p); break;
    case ObjectType::BOX:       world.spawn_box(p); break;
    case ObjectType::WHEEL:     world.spawn_wheel(p); break;
    case ObjectType::PIN:       world.spawn_pin(p); break;
    case ObjectType::PLATFORM:  world.spawn_platform(p); break;
    case ObjectType::RAMP:      world.spawn_platform(p, -0.30); break;
    case ObjectType::MOTOR:     world.spawn_motor(p); break;
    case ObjectType::CHASSIS:   world.spawn_car(p); break;
    case ObjectType::ROPE:      world.spawn_rope(p); break;
    case ObjectType::WATER:     world.spawn_water(p); break;
    case ObjectType::TANK:      world.spawn_tank(p); break;
    case ObjectType::GAS:       world.spawn_gas_cloud(p); break;
    case ObjectType::HEATER:    world.spawn_heater(p); break;
    case ObjectType::COOLER:    world.spawn_cooler(p); break;
    case ObjectType::CONTAINER: world.spawn_container(p); break;
    case ObjectType::BOMB:      world.spawn_bomb(p); break;
    case ObjectType::POLYGON:
    case ObjectType::SPRING:
    case ObjectType::PULLEY:
    case ObjectType::PISTON:
        break;
    }
}

void PhysicsCanvas::finalize_polygon() {
    if (drawing_points_.size() >= 3) {
        push_undo();
        world.spawn_polygon(drawing_points_);
    }
    drawing_points_.clear();
    refresh();
}

// ============================================================
// Rendering
// ============================================================

void PhysicsCanvas::draw_overlay(QPainter& painter) {
    const bool camera_moving = camera_button_ != Qt::NoButton && camera_moved_;

    // Where the next object would appear, with a plumb line to the floor for depth.
    if (mouse_inside_ && !world.dragged() && !camera_moving && !connection_start_ &&
        (spawns_on_click(active_tool_) || active_tool_ == ObjectType::POLYGON)) {
        const Vec3 p = spawn_point(mouse_pos_);
        const Vec3 foot(p.x, 0.0, p.z);
        QPointF sp, sf;
        if (screen_point(p, sp) && screen_point(foot, sf)) {
            painter.setPen(QPen(QColor(120, 200, 255, 110), 1, Qt::DashLine));
            painter.drawLine(sp, sf);
            const double rx = 14.0 * camera.pixels_per_cm(foot, height());
            painter.setPen(QPen(QColor(120, 200, 255, 150), 1.5));
            painter.setBrush(QColor(120, 200, 255, 30));
            painter.drawEllipse(sf, rx, std::max(1.5, rx * std::abs(std::sin(camera.pitch))));
        }
    }

    if (!drawing_points_.empty()) {
        painter.setPen(QPen(QColor(100, 255, 100), 2, Qt::DashLine));
        QPointF previous, current, first;
        const bool has_first = screen_point(drawing_points_[0], first);
        bool has_previous = has_first;
        previous = first;
        for (std::size_t i = 1; i < drawing_points_.size(); ++i) {
            const bool ok = screen_point(drawing_points_[i], current);
            if (ok && has_previous) painter.drawLine(previous, current);
            previous = current;
            has_previous = ok;
        }
        if (mouse_inside_ && has_previous) painter.drawLine(previous, mouse_pos_);
        if (has_first) {
            painter.setBrush(QColor(100, 255, 100));
            painter.drawEllipse(first, 5, 5);
        }
    }

    if (connection_start_) {
        QPointF from;
        if (screen_point(connection_start_->position, from)) {
            painter.setPen(QPen(QColor(100, 200, 255, 220), 2, Qt::DashLine));
            painter.drawLine(from, mouse_pos_);
        }
    }

    // Vessel gauges.
    painter.setFont(mono(10, true));
    for (const auto& c : world.containers) {
        QPointF at;
        if (!screen_point(Vec3(c.left, c.wall_top + 22.0, c.front), at)) continue;
        painter.setPen(QColor(245, 220, 150));
        painter.drawText(at, QString("%1 kPa  %2 °C").arg(c.pressure, 0, 'f', 1).arg(c.average_temperature, 0, 'f', 1));
    }

    draw_hud(painter);
    if (mouse_inside_ && !world.dragged() && !camera_moving && drawing_points_.empty()) draw_hover_readout(painter);

    if (paused_) {
        painter.setPen(QColor(229, 192, 123));
        painter.setFont(mono(13, true));
        painter.drawText(rect().adjusted(0, 12, -16, 0), Qt::AlignTop | Qt::AlignRight, "PAUSED");
    }
}

void PhysicsCanvas::draw_hud(QPainter& painter) const {
    struct Stat { int n = 0; double sum_t = 0.0; double avg() const { return n ? sum_t / n : 0.0; } };
    Stat gas, steam, liquid, ice;
    int solids = 0;
    for (const auto& ptr : world.objects) {
        const PhysicsObject& o = *ptr;
        if (o.is_gas) {
            ++gas.n; gas.sum_t += o.temperature;
            if (o.is_steam) { ++steam.n; steam.sum_t += o.temperature; }
        } else if (o.is_fluid) { ++liquid.n; liquid.sum_t += o.temperature; }
        else if (o.is_ice) { ++ice.n; ice.sum_t += o.temperature; }
        else if (!o.is_fixture) ++solids;
    }

    std::vector<std::pair<QString, QColor>> lines;
    const EnergyReport energy = world.energy();
    lines.push_back({QString("Bodies        : %1").arg(solids), Qt::white});
    lines.push_back({QString("Kinetic E     : %1 J").arg(energy.kinetic * kJoules, 0, 'f', 1), QColor(200, 230, 160)});
    lines.push_back({QString("Potential E   : %1 J").arg(energy.potential * kJoules, 0, 'f', 0), QColor(200, 230, 160)});
    if (gas.n > 0) {
        lines.push_back({QString("Gas particles : %1 (steam %2)").arg(gas.n).arg(steam.n), Qt::white});
        lines.push_back({QString("Gas avg T     : %1 °C").arg(gas.avg(), 0, 'f', 1), temp_color(gas.avg())});
    }
    if (liquid.n > 0)
        lines.push_back({QString("Water         : %1 L @ %2 °C").arg(liquid.n).arg(liquid.avg(), 0, 'f', 1), temp_color(liquid.avg())});
    if (ice.n > 0)
        lines.push_back({QString("Ice           : %1 kg @ %2 °C").arg(ice.n).arg(ice.avg(), 0, 'f', 1), QColor(100, 240, 255)});
    for (std::size_t i = 0; i < world.containers.size(); ++i) {
        const auto& c = world.containers[i];
        lines.push_back({QString("Vessel %1 P    : %2 kPa").arg(i + 1).arg(c.pressure, 0, 'f', 1),
                         c.pressure > 200.0 ? QColor(255, 150, 50) : QColor(255, 255, 255)});
        lines.push_back({QString("  V %1 L  T %2 °C").arg(c.gas_volume / 1000.0, 0, 'f', 0)
                             .arg(c.average_temperature, 0, 'f', 1), temp_color(c.average_temperature)});
        if (c.liquid_particles > 0)
            lines.push_back({QString("  boils at %1 °C").arg(c.boiling_point, 0, 'f', 1), QColor(150, 200, 255)});
    }
    painter.setFont(mono(11, true));
    draw_panel(painter, QRectF(10, 10, 310, 14 + 20 * int(lines.size())));
    for (std::size_t i = 0; i < lines.size(); ++i) {
        painter.setPen(lines[i].second);
        painter.drawText(QPointF(20, 31 + 20 * int(i)), lines[i].first);
    }

    // Room air temperature (driven by heaters, coolers and every object) and timing.
    draw_panel(painter, QRectF(10, height() - 40, 470, 30));
    painter.setPen(temp_color(world.ambient_temperature));
    painter.drawText(QPointF(20, height() - 20), QString("Room air %1 °C (set %2)")
        .arg(world.ambient_temperature, 0, 'f', 1).arg(world.ambient_setpoint, 0, 'f', 0));
    painter.setPen(QColor(150, 158, 172));
    painter.drawText(QPointF(270, height() - 20), QString("%1 fps  physics %2 ms")
        .arg(1000.0 / std::max(1.0, frame_ms_), 0, 'f', 0).arg(physics_ms_, 0, 'f', 1));

    painter.setFont(mono(10));
    painter.setPen(QColor(150, 158, 172));
    painter.drawText(rect().adjusted(0, 0, -14, -14), Qt::AlignBottom | Qt::AlignRight,
                     "right-drag: orbit   shift + right-drag: pan   wheel: zoom   right-click: remove");
}

void PhysicsCanvas::draw_hover_readout(QPainter& painter) const {
    const Ray ray = ray_at(mouse_pos_);
    const PhysicsObject* o = world.pick(ray.origin, ray.direction, World::Pick::Any);
    if (!o) return;

    QStringList lines;
    lines << kind_name(*o);
    const GasContainer* vessel = world.container_of_piston(o);
    if (!vessel && o->is_fixture)
        for (const auto& c : world.containers)
            if (c.group == o->group) vessel = &c;
    if (vessel) {
        lines << QString("Gas: %1 particles").arg(vessel->gas_particles)
              << QString("P %1 kPa   T %2 °C").arg(vessel->pressure, 0, 'f', 1).arg(vessel->average_temperature, 0, 'f', 1)
              << QString("V %1 L").arg(vessel->gas_volume / 1000.0, 0, 'f', 0);
        if (vessel->liquid_particles > 0) lines << QString("Water boils at %1 °C").arg(vessel->boiling_point, 0, 'f', 1);
    } else {
        lines << QString("T %1 °C").arg(o->temperature, 0, 'f', 1);
        if (o->is_water_substance()) {
            lines << QString("Boils at %1 °C (%2 kPa)").arg(o->boiling_point(), 0, 'f', 1).arg(o->local_pressure, 0, 'f', 1);
            if (o->latent_energy != 0.0) {
                const double total = o->mass * ((o->is_ice || (o->is_fluid && o->latent_energy < 0))
                                                ? thermo::kLatentFusion : thermo::kLatentVapor);
                lines << QString("Phase change %1 %").arg(std::abs(o->latent_energy) / total * 100.0, 0, 'f', 0);
            }
        }
        if (!o->is_static) {
            lines << QString("Speed %1 m/s").arg(o->velocity.length() * kMetresPerSecond, 0, 'f', 2);
            lines << QString("Mass %1 kg").arg(o->mass, 0, 'f', o->mass < 10.0 ? 2 : 1);
        }
        if (!o->is_particle() && !o->is_fixture)
            lines << QString("Density %1 g/cm³").arg(o->mass / (PhysicsObject::kWaterDensity * o->volume()), 0, 'f', 2);
        if (o->can_rotate()) lines << QString("Spin %1 rpm").arg(o->angular_velocity.length() * 60.0 * 60.0 / (2.0 * kPi), 0, 'f', 0);
        if (!o->is_gas && !o->is_fluid && !o->is_static && !o->is_ice)
            lines << QString("Bounce %1   friction %2").arg(o->effective_restitution(), 0, 'f', 2).arg(o->friction(), 0, 'f', 2);
        if (o->fuse > 0) lines << QString("Fuse %1 s").arg(o->fuse / 60.0, 0, 'f', 1);
        if (o->container_index >= 0) lines << QString("Inside vessel %1").arg(o->container_index + 1);
    }

    painter.setFont(mono(10));
    const QFontMetrics metrics(painter.font());
    int text_width = 0;
    for (const auto& line : lines) text_width = std::max(text_width, metrics.horizontalAdvance(line));
    const double line_h = metrics.height() + 2;
    QRectF box(mouse_pos_.x() + 16, mouse_pos_.y() + 16, text_width + 20, line_h * lines.size() + 12);
    if (box.right() > width() - 4) box.moveRight(mouse_pos_.x() - 12);
    if (box.bottom() > height() - 4) box.moveBottom(mouse_pos_.y() - 12);
    draw_panel(painter, box);
    for (int i = 0; i < lines.size(); ++i) {
        painter.setPen(i == 0 ? QColor(97, 175, 239) : QColor(220, 223, 228));
        painter.drawText(QPointF(box.left() + 10, box.top() + 6 + metrics.ascent() + i * line_h), lines[i]);
    }
}
