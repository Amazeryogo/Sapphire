#pragma once

#include "physics_canvas.h"

#include <QImage>
#include <QMainWindow>

#include <utility>
#include <vector>

class QComboBox;
class QLabel;
class QPushButton;
class QSlider;
class QStackedWidget;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow();

    // Opens the sandbox without the start dialog (used by --demo and tests).
    void enter_sandbox(int initial_balls);
    PhysicsCanvas* canvas() const { return canvas_; }
    // Picture of the whole window, 3D view included.
    QImage screenshot();

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
    void update_slider_labels();

    QStackedWidget* stack_ = nullptr;
    PhysicsCanvas* canvas_ = nullptr;
    std::vector<std::pair<QPushButton*, ObjectType>> tool_buttons_;
    QComboBox* material_box_ = nullptr;
    QPushButton* pause_btn_ = nullptr;
    QPushButton* undo_btn_ = nullptr;
    QPushButton* redo_btn_ = nullptr;
    QSlider *grav_slider_ = nullptr, *elas_slider_ = nullptr, *air_slider_ = nullptr, *time_slider_ = nullptr;
    QSlider *temp_slider_ = nullptr, *heat_slider_ = nullptr, *visc_slider_ = nullptr;
    QLabel *grav_label_ = nullptr, *elas_label_ = nullptr, *air_label_ = nullptr, *time_label_ = nullptr;
    QLabel *temp_label_ = nullptr, *heat_label_ = nullptr, *visc_label_ = nullptr;
};
