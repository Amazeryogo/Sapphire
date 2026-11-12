#pragma once
// Qt view/controller for the simulation: renders the World in 3D, turns mouse
// input into edits and camera moves, runs the fixed-timestep loop and keeps
// the undo history.

#include "camera.h"
#include "renderer.h"
#include "world.h"

#include <QElapsedTimer>
#include <QOpenGLWidget>
#include <QPointF>
#include <QTimer>

#include <deque>
#include <string>
#include <vector>

enum class ObjectType {
    BALL, BOX, WHEEL, MOTOR, CHASSIS, PIN, PLATFORM, RAMP, ROPE, WATER, TANK, GAS, POLYGON,
    HEATER, COOLER, BOMB, CONTAINER, SPRING, PULLEY, PISTON
};

class PhysicsCanvas : public QOpenGLWidget {
    Q_OBJECT
public:
    explicit PhysicsCanvas(QWidget* parent = nullptr);
    ~PhysicsCanvas() override;

    World world;
    Camera camera;

    void restart(int initial_balls);
    void load_demo();
    void set_tool(ObjectType tool) { active_tool_ = tool; drawing_points_.clear(); connection_start_ = nullptr; update(); }
    bool is_paused() const { return paused_; }
    void set_paused(bool paused);
    void set_time_scale(double scale) { time_scale_ = scale; }
    void reset_view();

    // ---- History (snapshots of the whole scene) ----
    void push_undo();
    bool undo();
    bool redo();
    bool can_undo() const { return !undo_stack_.empty(); }
    bool can_redo() const { return !redo_stack_.empty(); }

    // ---- Files ----
    std::string scene_text() const { return world.serialize(); }
    bool load_scene(const std::string& text, QString* error);

    // ---- Screen ↔ world ----
    // Where a click on empty space puts a new object: on the vertical plane
    // through the camera's target that faces the camera, kept inside the room.
    Vec3 spawn_point(const QPointF& pixel) const;
    // Pixel a world point is drawn at (false if it is behind the camera).
    bool screen_point(const Vec3& world_point, QPointF& pixel) const;

signals:
    void history_changed();
    void parameters_changed();   // world parameters replaced (undo / load): resync the sliders

protected:
    void initializeGL() override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void leaveEvent(QEvent* event) override;

private:
    void game_loop();
    void handle_spawning(const Vec3& p);
    void finalize_polygon();
    void restore(const std::string& snapshot);
    void reset_interaction();
    Ray ray_at(const QPointF& pixel) const { return camera.ray(pixel, width(), height()); }
    Vec3 drag_point(const QPointF& pixel) const;
    Vec3 throw_velocity(qint64 release_ms) const;

    void draw_overlay(class QPainter& painter);
    void draw_hud(class QPainter& painter) const;
    void draw_hover_readout(class QPainter& painter) const;

    ObjectType active_tool_ = ObjectType::BALL;
    bool paused_ = false;
    double time_scale_ = 1.0;

    Renderer renderer_;
    QTimer timer_;
    QElapsedTimer clock_;
    QElapsedTimer run_clock_;
    double accumulator_ms_ = 0.0;
    double physics_ms_ = 0.0;        // smoothed cost of one World::step()
    double frame_ms_ = 16.7;         // smoothed time between paints
    QElapsedTimer paint_clock_;

    PhysicsObject* connection_start_ = nullptr;
    std::vector<Vec3> drawing_points_;
    QPointF mouse_pos_;
    bool mouse_inside_ = false;

    // Dragging a body: it stays on the plane through 'drag_depth_' cm in front of the camera.
    double drag_depth_ = 0.0;
    Vec3 drag_offset_;               // body centre minus the grabbed point
    struct DragSample { qint64 ms; Vec3 position; };
    std::deque<DragSample> drag_samples_;

    // Camera moves with the right or middle button.
    Qt::MouseButton camera_button_ = Qt::NoButton;
    QPointF camera_press_pos_;
    QPointF camera_last_pos_;
    bool camera_moved_ = false;

    std::vector<std::string> undo_stack_;
    std::vector<std::string> redo_stack_;
};
