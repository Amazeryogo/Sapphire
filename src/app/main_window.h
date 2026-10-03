#pragma once

#include "physics_canvas.h"

#include <QMainWindow>

#include <utility>
#include <vector>

class QPushButton;
class QSlider;
class QStackedWidget;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow();

private slots:
    void start_module();
    void select_tool(ObjectType type);
    void toggle_pause();
    void update_physics_params();
    void sync_sliders_from_world();
    void update_history_buttons();
    void save_scene();
    void open_scene();

private:
    void setup_sandbox(int num_balls);
    void install_shortcuts();

    QStackedWidget* stack_ = nullptr;
    PhysicsCanvas* canvas_ = nullptr;
    std::vector<std::pair<QPushButton*, ObjectType>> tool_buttons_;
    QPushButton* pause_btn_ = nullptr;
    QPushButton* undo_btn_ = nullptr;
    QPushButton* redo_btn_ = nullptr;
    QSlider *grav_slider_ = nullptr, *elas_slider_ = nullptr, *wind_slider_ = nullptr;
    QSlider *temp_slider_ = nullptr, *heat_slider_ = nullptr, *visc_slider_ = nullptr;
};
