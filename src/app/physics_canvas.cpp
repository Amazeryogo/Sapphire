#include "physics_canvas.h"

#include "random.h"

#include <QFont>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QPolygonF>

#include <cmath>
#include <map>

namespace {
constexpr double kFrameMs = 1000.0 / 60.0;   // one World::step()
constexpr int kMaxStepsPerTick = 4;           // beyond this the sim slows down instead of stalling
constexpr std::size_t kHistoryLimit = 40;
constexpr double kPi = 3.14159265358979323846;

QColor to_qcolor(const Rgba& c) { return QColor(c.r, c.g, c.b, c.a); }

// Colour shift that makes temperature visible: hot → red/orange, cold → blue.
QColor thermal_tint(const QColor& base, double temperature) {
    double r = base.red(), g = base.green(), b = base.blue();
    if (temperature > 25.0) {
        const double k = clampd((temperature - 25.0) / 250.0, 0.0, 0.85);
        r += (255 - r) * k; g += (90 - g) * k; b += (40 - b) * k;
    } else if (temperature < 10.0) {
        const double k = clampd((10.0 - temperature) / 120.0, 0.0, 0.8);
        r += (120 - r) * k; g += (200 - g) * k; b += (255 - b) * k;
    }
    return QColor(int(r), int(g), int(b), base.alpha());
}

QColor temp_color(double t) {
    if (t > 50.0) return QColor(255, 100, 100);
    if (t < 0.0) return QColor(100, 240, 255);
    return QColor(255, 255, 255);
}

QString kind_name(const PhysicsObject& o) {
    if (o.is_container_piston) return "Vessel piston";
    if (o.is_heater) return "Heater";
    if (o.is_cooler) return "Cooler";
    if (o.is_ice) return "Ice";
    if (o.is_steam) return "Steam";
    if (o.is_gas) return "Air";
    if (o.is_fluid) return "Water";
    if (o.fuse > 0) return "Bomb";
    if (o.is_motor) return "Motor wheel";
    if (o.is_static) return "Pin";
    return o.shape_type == ShapeType::SQUARE ? "Box" : "Ball";
}

bool spawns_on_click(ObjectType tool) {
    return tool != ObjectType::POLYGON && tool != ObjectType::SPRING
        && tool != ObjectType::PULLEY && tool != ObjectType::PISTON;
}

void draw_panel(QPainter& painter, const QRectF& rect) {
    painter.setBrush(QColor(20, 22, 28, 220));
    painter.setPen(QPen(QColor(100, 150, 255), 1));
    painter.drawRoundedRect(rect, 8, 8);
}
} // namespace

PhysicsCanvas::PhysicsCanvas(QWidget* parent) : QWidget(parent) {
    setMouseTracking(true);              // hover readout and polygon preview
    setAttribute(Qt::WA_OpaquePaintEvent);
    connect(&timer_, &QTimer::timeout, this, &PhysicsCanvas::game_loop);
    timer_.setTimerType(Qt::PreciseTimer);
    timer_.start(8);
    clock_.start();
}

void PhysicsCanvas::restart(int initial_balls) {
    world.clear();
    reset_interaction();
    const double w = std::max(200.0, world.width), h = std::max(200.0, world.height);
    for (int i = 0; i < initial_balls; ++i)
        world.spawn_ball(random_double(50, w - 50), random_double(50, h * 0.5));
    update();
}

void PhysicsCanvas::set_paused(bool paused) {
    paused_ = paused;
    accumulator_ms_ = 0.0;
    clock_.restart();
    update();
}

// Fixed timestep: the World always advances in 1/60 s steps, however often
// the timer fires or however long painting takes.
void PhysicsCanvas::game_loop() {
    const double elapsed = double(clock_.nsecsElapsed()) / 1e6;
    clock_.restart();
    if (!paused_) {
        accumulator_ms_ += std::min(elapsed, 250.0);
        int steps = 0;
        while (accumulator_ms_ >= kFrameMs && steps < kMaxStepsPerTick) {
            world.step();
            accumulator_ms_ -= kFrameMs;
            ++steps;
        }
        if (steps == kMaxStepsPerTick) accumulator_ms_ = 0.0;   // drop the backlog
        if (steps > 0) update();
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
    update();
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
    update();
    return true;
}

void PhysicsCanvas::reset_interaction() {
    if (world.dragged()) world.end_drag(Vector());
    connection_start_ = nullptr;
    drawing_points_.clear();
}

// ============================================================
// Input
// ============================================================

Vector PhysicsCanvas::throw_velocity(qint64 now) const {
    // Average the pointer motion over the last ~60 ms.  If it has been still
    // for a moment before release, the body is simply let go.  Times are the
    // input events' own timestamps, so a busy frame does not distort them.
    if (mouse_samples_.size() < 2) return Vector();
    const MouseSample& last = mouse_samples_.back();
    if (now - last.ms > 50) return Vector();
    const MouseSample* first = &mouse_samples_.front();
    for (const auto& s : mouse_samples_) {
        if (last.ms - s.ms <= 60) { first = &s; break; }
    }
    const double dt_ms = double(last.ms - first->ms);
    if (dt_ms < 1.0) return Vector();
    Vector v((last.pos.x() - first->pos.x()) / dt_ms * kFrameMs,
             (last.pos.y() - first->pos.y()) / dt_ms * kFrameMs);
    const double speed = v.length();
    if (speed > 60.0) v *= 60.0 / speed;
    return v;
}

void PhysicsCanvas::mousePressEvent(QMouseEvent* event) {
    const QPointF pos = event->position();
    const Vector v_pos(pos.x(), pos.y());
    mouse_pos_ = pos;
    mouse_samples_.clear();
    mouse_samples_.push_back({qint64(event->timestamp()), pos});

    if (event->button() == Qt::RightButton) {
        if (active_tool_ == ObjectType::POLYGON && !drawing_points_.empty()) {
            finalize_polygon();
            return;
        }
        if (PhysicsObject* hit = world.object_at(v_pos)) {
            push_undo();
            if (connection_start_ == hit) connection_start_ = nullptr;
            world.remove_object(hit);
            update();
        }
        return;
    }
    if (event->button() != Qt::LeftButton) return;

    if (active_tool_ == ObjectType::POLYGON) {
        if (!drawing_points_.empty() && std::hypot(pos.x() - drawing_points_[0].x(), pos.y() - drawing_points_[0].y()) < 15)
            finalize_polygon();
        else
            drawing_points_.push_back(pos);
        update();
        return;
    }

    if (PhysicsObject* hit = world.object_at(v_pos)) {
        if (active_tool_ == ObjectType::SPRING || active_tool_ == ObjectType::PULLEY || active_tool_ == ObjectType::PISTON) {
            connection_start_ = hit;
        } else {
            world.begin_drag(hit);
        }
    } else if (spawns_on_click(active_tool_)) {
        push_undo();
        handle_spawning(pos.x(), pos.y());
    }
    update();
}

void PhysicsCanvas::mouseMoveEvent(QMouseEvent* event) {
    const QPointF pos = event->position();
    mouse_pos_ = pos;
    mouse_inside_ = true;
    if (event->buttons() & Qt::LeftButton) {
        mouse_samples_.push_back({qint64(event->timestamp()), pos});
        while (mouse_samples_.size() > 16) mouse_samples_.pop_front();
    }
    if (world.dragged()) world.drag_to(Vector(pos.x(), pos.y()), paused_);
    update();   // hover readout, previews
}

void PhysicsCanvas::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() != Qt::LeftButton) return;
    const QPointF pos = event->position();
    if (world.dragged()) {
        world.end_drag(paused_ ? Vector() : throw_velocity(qint64(event->timestamp())));
    } else if (connection_start_) {
        PhysicsObject* target = world.object_at(Vector(pos.x(), pos.y()));
        if (target && target != connection_start_) {
            push_undo();
            if (active_tool_ == ObjectType::SPRING) world.connect_spring(connection_start_, target);
            else if (active_tool_ == ObjectType::PISTON) world.connect_piston(connection_start_, target);
            else if (active_tool_ == ObjectType::PULLEY) world.connect_pulley(connection_start_, target);
        }
        connection_start_ = nullptr;
    }
    mouse_samples_.clear();
    update();
}

void PhysicsCanvas::leaveEvent(QEvent* event) {
    QWidget::leaveEvent(event);
    mouse_inside_ = false;
    update();
}

void PhysicsCanvas::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    world.width = width();
    world.height = height();
}

void PhysicsCanvas::handle_spawning(double x, double y) {
    switch (active_tool_) {
    case ObjectType::BALL:      world.spawn_ball(x, y); break;
    case ObjectType::BOX:       world.spawn_box(x, y); break;
    case ObjectType::WHEEL:     world.spawn_wheel(x, y); break;
    case ObjectType::PIN:       world.spawn_pin(x, y); break;
    case ObjectType::MOTOR:     world.spawn_motor(x, y); break;
    case ObjectType::CHASSIS:   world.spawn_car(x, y); break;
    case ObjectType::ROPE:      world.spawn_rope(x, y); break;
    case ObjectType::WATER:     world.spawn_water(x, y); break;
    case ObjectType::GAS:       world.spawn_gas_cloud(x, y); break;
    case ObjectType::HEATER:    world.spawn_heater(x, y); break;
    case ObjectType::COOLER:    world.spawn_cooler(x, y); break;
    case ObjectType::CONTAINER: world.spawn_container(x, y); break;
    case ObjectType::BOMB:      world.spawn_bomb(x, y); break;
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
        std::vector<Vector> outline;
        for (const auto& p : drawing_points_) outline.emplace_back(p.x(), p.y());
        world.spawn_polygon(outline);
    }
    drawing_points_.clear();
    update();
}

// ============================================================
// Rendering
// ============================================================

void PhysicsCanvas::paintEvent(QPaintEvent* event) {
    static_cast<void>(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), QColor(30, 32, 38));

    if (!drawing_points_.empty()) {
        painter.setPen(QPen(QColor(100, 255, 100), 2, Qt::DashLine));
        for (std::size_t i = 0; i + 1 < drawing_points_.size(); ++i)
            painter.drawLine(drawing_points_[i], drawing_points_[i + 1]);
        if (mouse_inside_) painter.drawLine(drawing_points_.back(), mouse_pos_);
        painter.setBrush(QBrush(QColor(100, 255, 100)));
        painter.drawEllipse(drawing_points_[0], 5, 5);
    }

    draw_links(painter);
    draw_containers(painter);

    if (connection_start_) {
        painter.setPen(QPen(QColor(100, 200, 255, 200), 2, Qt::DashLine));
        painter.drawLine(QPointF(connection_start_->position.x, connection_start_->position.y), mouse_pos_);
    }

    draw_bodies(painter);
    draw_hud(painter);
    if (mouse_inside_ && !world.dragged() && drawing_points_.empty()) draw_hover_readout(painter);

    if (paused_) {
        painter.setPen(QColor(229, 192, 123));
        painter.setFont(QFont("Consolas", 12, QFont::Bold));
        painter.drawText(rect().adjusted(0, 12, -16, 0), Qt::AlignTop | Qt::AlignRight, "PAUSED");
    }
}

const QPixmap& PhysicsCanvas::particle_sprite(QRgb color, int diameter) const {
    const quint64 key = (quint64(color) << 16) | quint64(diameter & 0xffff);
    auto it = sprite_cache_.find(key);
    if (it != sprite_cache_.end()) return it->second;
    if (sprite_cache_.size() > 512) sprite_cache_.clear();   // bounded
    QPixmap sprite(diameter + 2, diameter + 2);              // 1 px border for antialiasing
    sprite.fill(Qt::transparent);
    QPainter p(&sprite);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor::fromRgba(color));
    p.drawEllipse(QRectF(1, 1, diameter, diameter));
    p.end();
    return sprite_cache_.emplace(key, std::move(sprite)).first->second;
}

void PhysicsCanvas::draw_links(QPainter& painter) const {
    auto pt = [](const PhysicsObject* o) { return QPointF(o->position.x, o->position.y); };

    painter.setPen(QPen(QColor(200, 200, 200, 150), 2));
    painter.setBrush(QBrush(QColor(255, 255, 255)));
    for (const auto& p : world.pulleys) {
        const QPointF anchor(p.anchor.x, p.anchor.y);
        painter.drawLine(pt(p.obj_a), anchor);
        painter.drawLine(pt(p.obj_b), anchor);
        painter.drawEllipse(anchor, 5, 5);
    }

    painter.setPen(QPen(QColor(150, 150, 150), 3));
    for (const auto& s : world.springs) painter.drawLine(pt(s.obj_a), pt(s.obj_b));

    painter.setPen(QPen(QColor(180, 150, 100), 4));
    for (const auto& l : world.rigid_links)
        if (!l.ice_bond) painter.drawLine(pt(l.obj_a), pt(l.obj_b));

    // Pistons: a thick barrel anchored on obj_b and a thinner rod to obj_a.
    for (const auto& l : world.pistons) {
        const Vector delta = l.obj_a->position - l.obj_b->position;
        const double dist = delta.length();
        if (dist < 1e-6) continue;
        const Vector barrel_end = l.obj_b->position + delta / dist * std::min(dist, l.min_len);
        painter.setPen(QPen(QColor(255, 140, 50), 3));
        painter.drawLine(pt(l.obj_a), pt(l.obj_b));
        painter.setPen(QPen(thermal_tint(QColor(120, 125, 135), l.gas_temperature), 10, Qt::SolidLine, Qt::FlatCap));
        painter.drawLine(pt(l.obj_b), QPointF(barrel_end.x, barrel_end.y));
    }
}

void PhysicsCanvas::draw_containers(QPainter& painter) const {
    for (const auto& c : world.containers) {
        QColor chamber = thermal_tint(QColor(80, 110, 90), c.average_temperature);
        chamber.setAlpha(45);
        painter.setPen(Qt::NoPen);
        painter.setBrush(chamber);
        painter.drawRect(QRectF(c.left, c.top(), c.width(), c.bottom - c.top()));

        painter.setPen(QPen(QColor(110, 125, 140), GasContainer::kWallHalfThickness * 2));
        painter.setBrush(Qt::NoBrush);
        painter.drawLine(QPointF(c.left, c.wall_top), QPointF(c.left, c.bottom));
        painter.drawLine(QPointF(c.right, c.wall_top), QPointF(c.right, c.bottom));
        painter.drawLine(QPointF(c.left, c.bottom), QPointF(c.right, c.bottom));
        painter.setPen(QPen(QColor(255, 140, 50), 8, Qt::SolidLine, Qt::FlatCap));
        painter.drawLine(QPointF(c.left + 3, c.top()), QPointF(c.right - 3, c.top()));
        painter.setPen(QPen(QColor(200, 110, 40), 4));
        painter.drawLine(QPointF((c.left + c.right) * 0.5, c.top()), QPointF((c.left + c.right) * 0.5, c.top() - 18));
        painter.setPen(QColor(245, 220, 150));
        painter.setFont(QFont("Consolas", 9, QFont::Bold));
        painter.drawText(QPointF(c.left + 6, c.top() + 17),
            QString("%1 kPa  %2 °C").arg(c.pressure, 0, 'f', 1).arg(c.average_temperature, 0, 'f', 1));
    }
}

void PhysicsCanvas::draw_bodies(QPainter& painter) const {
    // Particles (water, ice, gas, steam) are drawn as cached sprites: each
    // colour/size combination is rendered once as an antialiased disc, then
    // blitted in one drawPixmapFragments call per bucket, which is several
    // times faster than filling one ellipse path per particle.  Temperatures
    // are quantised to 5 °C so the number of buckets stays small.
    std::map<std::pair<QRgb, int>, std::vector<QPainter::PixmapFragment>> batches;
    for (const auto& ptr : world.objects) {
        const PhysicsObject& o = *ptr;
        if (!is_valid(o.position)) continue;
        if (!(o.is_fluid || o.is_gas || o.is_ice)) continue;
        const double t = std::round(o.temperature / 5.0) * 5.0;
        const QColor fill = o.is_ice ? to_qcolor(o.color) : thermal_tint(to_qcolor(o.color), t);
        const int diameter = std::max(2, int(std::lround(2.0 * o.radius)));
        batches[{fill.rgba(), diameter}].push_back(
            QPainter::PixmapFragment::create(QPointF(o.position.x, o.position.y),
                                             QRectF(0, 0, diameter + 2, diameter + 2)));
    }
    for (const auto& [key, fragments] : batches) {
        const QPixmap& sprite = particle_sprite(key.first, key.second);
        painter.drawPixmapFragments(fragments.data(), int(fragments.size()), sprite);
    }

    for (const auto& ptr : world.objects) {
        const PhysicsObject& o = *ptr;
        if (!is_valid(o.position) || !is_valid(o.radius)) continue;
        if (o.is_fluid || o.is_gas || o.is_ice || o.is_container_piston) continue;
        const QColor color = to_qcolor(o.color);
        const QPointF centre(o.position.x, o.position.y);

        if (o.trail.size() > 2 && !o.is_static) {
            for (std::size_t i = 1; i < o.trail.size(); ++i) {
                const double f = double(i) / double(o.trail.size());
                QPen pen(QColor(color.red(), color.green(), color.blue(), int(255 * f)));
                pen.setWidthF(std::max(2.0, o.radius / 2.0 * f));
                painter.setPen(pen);
                painter.drawLine(QPointF(o.trail[i - 1].x, o.trail[i - 1].y), QPointF(o.trail[i].x, o.trail[i].y));
            }
        }

        if (o.fuse > 0) {
            painter.setPen(QPen(o.fuse % 10 < 5 ? QColor(255, 255, 255) : QColor(255, 50, 50), 4));
        } else if (o.is_static) {
            painter.setPen(QPen(QColor(255, 255, 255), 2));
        } else if (o.is_motor) {
            painter.setPen(QPen(QColor(255, 255, 100), 3));
        } else if (std::abs(o.temperature - 20.0) > 15.0) {
            // Temperature ring on solids, so the temperature-dependent bounce is readable.
            const bool hot = o.temperature > 20.0;
            const int alpha = int(clampd(std::abs(o.temperature - 20.0) * 1.5, 40.0, 255.0));
            painter.setPen(QPen(hot ? QColor(255, 110, 40, alpha) : QColor(120, 220, 255, alpha), 3));
        } else {
            painter.setPen(Qt::NoPen);
        }
        painter.setBrush(color);

        if (o.shape_type == ShapeType::CIRCLE) {
            painter.drawEllipse(centre, o.radius, o.radius);
            // A spoke shows rotation (rolling, motors).
            if (o.can_rotate() && o.radius >= 8.0) {
                const QPointF rim(o.position.x + std::cos(o.angle) * o.radius * 0.85,
                                  o.position.y + std::sin(o.angle) * o.radius * 0.85);
                painter.setPen(QPen(color.darker(170), std::max(2.0, o.radius / 6.0), Qt::SolidLine, Qt::RoundCap));
                painter.drawLine(centre, rim);
            }
        } else {
            const double r = o.radius;
            painter.drawRect(QRectF(o.position.x - r, o.position.y - r, r * 2, r * 2));
        }
    }
}

void PhysicsCanvas::draw_hud(QPainter& painter) const {
    struct Stat { int n = 0; double sum_t = 0.0; double avg() const { return n ? sum_t / n : 0.0; } };
    Stat gas, steam, liquid, ice;
    double min_x = width(), max_x = 0, min_y = height(), max_y = 0;
    for (const auto& ptr : world.objects) {
        const PhysicsObject& o = *ptr;
        if (o.is_gas) {
            ++gas.n; gas.sum_t += o.temperature;
            if (o.is_steam) { ++steam.n; steam.sum_t += o.temperature; }
            if (o.container_index < 0) {
                min_x = std::min(min_x, o.position.x); max_x = std::max(max_x, o.position.x);
                min_y = std::min(min_y, o.position.y); max_y = std::max(max_y, o.position.y);
            }
        } else if (o.is_fluid) { ++liquid.n; liquid.sum_t += o.temperature; }
        else if (o.is_ice) { ++ice.n; ice.sum_t += o.temperature; }
    }

    // Free gas is at atmospheric pressure; outline the free cloud.
    if (max_x > min_x && max_y > min_y) {
        painter.setPen(QPen(QColor(150, 255, 100, 80), 2, Qt::DashLine));
        painter.setBrush(Qt::NoBrush);
        painter.drawRect(QRectF(min_x - 10, min_y - 10, (max_x - min_x) + 20, (max_y - min_y) + 20));
    }

    std::vector<std::pair<QString, QColor>> lines;
    if (gas.n > 0) {
        lines.push_back({QString("Gas particles : %1 (steam %2)").arg(gas.n).arg(steam.n), Qt::white});
        lines.push_back({QString("Gas avg T     : %1 °C").arg(gas.avg(), 0, 'f', 1), temp_color(gas.avg())});
    }
    if (liquid.n > 0)
        lines.push_back({QString("Water         : %1 @ %2 °C").arg(liquid.n).arg(liquid.avg(), 0, 'f', 1), temp_color(liquid.avg())});
    if (ice.n > 0)
        lines.push_back({QString("Ice           : %1 @ %2 °C").arg(ice.n).arg(ice.avg(), 0, 'f', 1), QColor(100, 240, 255)});
    for (std::size_t i = 0; i < world.containers.size(); ++i) {
        const auto& c = world.containers[i];
        lines.push_back({QString("Vessel %1 P    : %2 kPa").arg(i + 1).arg(c.pressure, 0, 'f', 1),
                         c.pressure > 200.0 ? QColor(255, 150, 50) : QColor(255, 255, 255)});
        // The sandbox is 2D, so a vessel's "volume" is an area.
        lines.push_back({QString("  A %1k px²  T %2 °C").arg(c.gas_volume / 1000.0, 0, 'f', 1)
                             .arg(c.average_temperature, 0, 'f', 1), temp_color(c.average_temperature)});
        if (c.liquid_particles > 0)
            lines.push_back({QString("  boils at %1 °C").arg(c.boiling_point, 0, 'f', 1), QColor(150, 200, 255)});
    }
    painter.setFont(QFont("Consolas", 11, QFont::Bold));
    if (!lines.empty()) {
        draw_panel(painter, QRectF(10, 10, 300, 14 + 22 * int(lines.size())));
        for (std::size_t i = 0; i < lines.size(); ++i) {
            painter.setPen(lines[i].second);
            painter.drawText(QPointF(20, 32 + 22 * int(i)), lines[i].first);
        }
    }

    // Room air temperature (driven by heaters, coolers and every object).
    draw_panel(painter, QRectF(10, height() - 40, 340, 30));
    painter.setPen(temp_color(world.ambient_temperature));
    painter.drawText(QPointF(20, height() - 20), QString("Room Air : %1 °C  (setpoint %2)")
        .arg(world.ambient_temperature, 0, 'f', 1).arg(world.ambient_setpoint, 0, 'f', 0));
}

void PhysicsCanvas::draw_hover_readout(QPainter& painter) const {
    const PhysicsObject* o = world.object_at(Vector(mouse_pos_.x(), mouse_pos_.y()));
    if (!o) return;

    QStringList lines;
    lines << kind_name(*o);
    if (const GasContainer* c = world.container_of_piston(o)) {
        lines << QString("Gas: %1 particles").arg(c->gas_particles)
              << QString("P %1 kPa   T %2 °C").arg(c->pressure, 0, 'f', 1).arg(c->average_temperature, 0, 'f', 1)
              << QString("A %1k px²").arg(c->gas_volume / 1000.0, 0, 'f', 1);
        if (c->liquid_particles > 0) lines << QString("Water boils at %1 °C").arg(c->boiling_point, 0, 'f', 1);
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
            lines << QString("Speed %1 px/frame").arg(o->velocity.length(), 0, 'f', 2);
            lines << QString("Mass %1").arg(o->mass, 0, 'f', 2);
        }
        if (o->can_rotate()) lines << QString("Spin %1 rpm").arg(o->angular_velocity * 60.0 * 60.0 / (2.0 * kPi), 0, 'f', 0);
        if (!o->is_gas && !o->is_fluid && !o->is_static && !o->is_ice)
            lines << QString("Bounce %1").arg(o->effective_restitution(), 0, 'f', 2);
        if (o->fuse > 0) lines << QString("Fuse %1 s").arg(o->fuse / 60.0, 0, 'f', 1);
        if (o->container_index >= 0) lines << QString("Inside vessel %1").arg(o->container_index + 1);
    }

    painter.setFont(QFont("Consolas", 10));
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
