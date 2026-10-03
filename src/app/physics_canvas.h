#pragma once
// Qt view/controller for the simulation: renders the World, turns mouse input
// into edits, runs the fixed-timestep loop and keeps the undo history.

#include "world.h"

#include <QElapsedTimer>
#include <QPixmap>
#include <QPointF>
#include <QTimer>
#include <QWidget>

#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

enum class ObjectType {
    BALL, BOX, WHEEL, MOTOR, CHASSIS, PIN, ROPE, WATER, GAS, POLYGON,
    HEATER, COOLER, BOMB, CONTAINER, SPRING, PULLEY, PISTON
};

class PhysicsCanvas : public QWidget {
    Q_OBJECT
public:
    explicit PhysicsCanvas(QWidget* parent = nullptr);

    World world;

    void restart(int initial_balls);
    void set_tool(ObjectType tool) { active_tool_ = tool; drawing_points_.clear(); connection_start_ = nullptr; }
    bool is_paused() const { return paused_; }
    void set_paused(bool paused);

    // ---- History (snapshots of the whole scene) ----
    void push_undo();
    bool undo();
    bool redo();
    bool can_undo() const { return !undo_stack_.empty(); }
    bool can_redo() const { return !redo_stack_.empty(); }

    // ---- Files ----
    std::string scene_text() const { return world.serialize(); }
    bool load_scene(const std::string& text, QString* error);

signals:
    void history_changed();
    void parameters_changed();   // world parameters replaced (undo / load): resync the sliders

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void leaveEvent(QEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    void game_loop();
    void handle_spawning(double x, double y);
    void finalize_polygon();
    void restore(const std::string& snapshot);
    void reset_interaction();
    Vector throw_velocity(qint64 release_ms) const;

    void draw_links(class QPainter& painter) const;
    void draw_containers(class QPainter& painter) const;
    void draw_bodies(class QPainter& painter) const;
    void draw_hud(class QPainter& painter) const;
    void draw_hover_readout(class QPainter& painter) const;
    const QPixmap& particle_sprite(QRgb color, int diameter) const;

    ObjectType active_tool_ = ObjectType::BALL;
    bool paused_ = false;

    QTimer timer_;
    QElapsedTimer clock_;
    double accumulator_ms_ = 0.0;

    PhysicsObject* connection_start_ = nullptr;
    std::vector<QPointF> drawing_points_;
    QPointF mouse_pos_;
    bool mouse_inside_ = false;

    struct MouseSample { qint64 ms; QPointF pos; };
    std::deque<MouseSample> mouse_samples_;

    mutable std::unordered_map<quint64, QPixmap> sprite_cache_;

    std::vector<std::string> undo_stack_;
    std::vector<std::string> redo_stack_;
};
