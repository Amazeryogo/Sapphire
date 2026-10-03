#include "main_window.h"

#include <QFile>
#include <QFileDialog>
#include <QFont>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QKeySequence>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QShortcut>
#include <QSignalBlocker>
#include <QSlider>
#include <QStackedWidget>
#include <QVBoxLayout>

#include <cmath>

namespace {
const char* kToolStyle =
    "QPushButton { background-color: #3e4451; color: white; padding: 6px; border-radius: 4px; }"
    "QPushButton:checked { background-color: #61afef; color: black; font-weight: bold; }";
const char* kLinkStyle =
    "QPushButton { background-color: #d19a66; color: black; padding: 6px; border-radius: 4px; }"
    "QPushButton:checked { background-color: #e5c07b; font-weight: bold; }";
const char* kSmallStyle =
    "QPushButton { background-color: #3e4451; color: white; padding: 5px; border-radius: 4px; }"
    "QPushButton:disabled { color: #6b7280; }";
const char* kRunStyle = "background-color: #98c379; color: black; padding: 10px; font-weight: bold;";
const char* kPausedStyle = "background-color: #e5c07b; color: black; padding: 10px; font-weight: bold;";
const char* kSceneFilter = "Sapphire scenes (*.sapphire);;All files (*)";
} // namespace

MainWindow::MainWindow() {
    setWindowTitle("Sapphire");
    resize(1200, 900);
    setStyleSheet("background-color: #282c34;");

    stack_ = new QStackedWidget(this);
    setCentralWidget(stack_);

    auto* intro_page = new QWidget();
    auto* intro_layout = new QVBoxLayout(intro_page);
    intro_layout->setAlignment(Qt::AlignCenter);

    auto* title = new QLabel("Sapphire");
    title->setFont(QFont("Arial", 32, QFont::Bold));
    title->setStyleSheet("color: #61afef; margin-bottom: 20px;");
    intro_layout->addWidget(title, 0, Qt::AlignCenter);

    auto* start_btn = new QPushButton("Enter Sandbox");
    start_btn->setFixedSize(300, 60);
    start_btn->setStyleSheet(
        "QPushButton { background-color: #3e4451; color: white; font-size: 18px; border-radius: 10px; }"
        "QPushButton:hover { background-color: #4b5263; }");
    connect(start_btn, &QPushButton::clicked, this, &MainWindow::start_module);
    intro_layout->addWidget(start_btn, 0, Qt::AlignCenter);

    stack_->addWidget(intro_page);
}

void MainWindow::start_module() {
    bool ok = false;
    const int num_balls = QInputDialog::getInt(this, "Initialization", "Initial object count:", 10, 0, 500, 1, &ok);
    if (!ok) return;
    if (!canvas_) setup_sandbox(num_balls);
    stack_->setCurrentIndex(1);
}

void MainWindow::setup_sandbox(int num_balls) {
    auto* sandbox_widget = new QWidget();
    auto* sandbox_layout = new QHBoxLayout(sandbox_widget);
    sandbox_layout->setContentsMargins(0, 0, 0, 0);
    sandbox_layout->setSpacing(0);

    auto* toolbox = new QWidget();
    toolbox->setFixedWidth(180);
    toolbox->setStyleSheet("background-color: #21252b; border-right: 1px solid #333;");
    auto* tb_layout = new QVBoxLayout(toolbox);

    tb_layout->addWidget(new QLabel("<b style='color:#abb2bf; font-size:14px'>SPAWN TOOLS</b>"));
    const std::vector<std::pair<QString, ObjectType>> tools = {
        {"Ball", ObjectType::BALL}, {"Box", ObjectType::BOX}, {"Pin (Static)", ObjectType::PIN},
        {"Draw Shape", ObjectType::POLYGON}, {"Gas Cloud", ObjectType::GAS},
        {"Rope Chain", ObjectType::ROPE}, {"Water Drop", ObjectType::WATER},
        {"Container", ObjectType::CONTAINER}, {"Heater", ObjectType::HEATER},
        {"Cooler", ObjectType::COOLER}, {"Wheel", ObjectType::WHEEL},
        {"Motor", ObjectType::MOTOR}, {"Car", ObjectType::CHASSIS},
        {"Shockwave Bomb", ObjectType::BOMB},
    };
    for (const auto& [label, type] : tools) {
        auto* btn = new QPushButton(label);
        btn->setCheckable(true);
        btn->setStyleSheet(kToolStyle);
        connect(btn, &QPushButton::clicked, this, [this, type = type]() { select_tool(type); });
        tb_layout->addWidget(btn);
        tool_buttons_.push_back({btn, type});
    }

    tb_layout->addWidget(new QLabel("<b style='color:#abb2bf; font-size:14px; margin-top:15px'>LINK TOOLS</b>"));
    const std::vector<std::pair<QString, ObjectType>> links = {
        {"Bouncy Spring", ObjectType::SPRING}, {"Pulley", ObjectType::PULLEY}, {"Piston", ObjectType::PISTON},
    };
    for (const auto& [label, type] : links) {
        auto* btn = new QPushButton(label);
        btn->setCheckable(true);
        btn->setStyleSheet(kLinkStyle);
        connect(btn, &QPushButton::clicked, this, [this, type = type]() { select_tool(type); });
        tb_layout->addWidget(btn);
        tool_buttons_.push_back({btn, type});
    }

    tb_layout->addStretch();

    // ---- History and files ----
    auto* edit_row = new QGridLayout();
    undo_btn_ = new QPushButton("Undo");
    redo_btn_ = new QPushButton("Redo");
    auto* save_btn = new QPushButton("Save…");
    auto* open_btn = new QPushButton("Open…");
    undo_btn_->setToolTip("Undo (Ctrl+Z)");
    redo_btn_->setToolTip("Redo (Ctrl+Shift+Z)");
    save_btn->setToolTip("Save scene (Ctrl+S)");
    open_btn->setToolTip("Open scene (Ctrl+O)");
    for (QPushButton* b : {undo_btn_, redo_btn_, save_btn, open_btn}) b->setStyleSheet(kSmallStyle);
    edit_row->addWidget(undo_btn_, 0, 0);
    edit_row->addWidget(redo_btn_, 0, 1);
    edit_row->addWidget(save_btn, 1, 0);
    edit_row->addWidget(open_btn, 1, 1);
    tb_layout->addLayout(edit_row);

    pause_btn_ = new QPushButton("Pause Physics");
    pause_btn_->setToolTip("Pause / resume (Space)");
    pause_btn_->setStyleSheet(kRunStyle);
    connect(pause_btn_, &QPushButton::clicked, this, &MainWindow::toggle_pause);
    tb_layout->addWidget(pause_btn_);

    auto* clear_btn = new QPushButton("Clear All");
    clear_btn->setStyleSheet("background-color: #e06c75; color: white; padding: 10px; font-weight: bold;");
    connect(clear_btn, &QPushButton::clicked, this, [this]() {
        canvas_->push_undo();
        canvas_->restart(0);
    });
    tb_layout->addWidget(clear_btn);

    sandbox_layout->addWidget(toolbox);

    auto* right_container = new QWidget();
    auto* right_layout = new QVBoxLayout(right_container);
    right_layout->setContentsMargins(0, 0, 0, 0);
    right_layout->setSpacing(0);

    canvas_ = new PhysicsCanvas(right_container);
    right_layout->addWidget(canvas_, 1);
    connect(undo_btn_, &QPushButton::clicked, canvas_, &PhysicsCanvas::undo);
    connect(redo_btn_, &QPushButton::clicked, canvas_, &PhysicsCanvas::redo);
    connect(save_btn, &QPushButton::clicked, this, &MainWindow::save_scene);
    connect(open_btn, &QPushButton::clicked, this, &MainWindow::open_scene);
    connect(canvas_, &PhysicsCanvas::history_changed, this, &MainWindow::update_history_buttons);
    connect(canvas_, &PhysicsCanvas::parameters_changed, this, &MainWindow::sync_sliders_from_world);

    auto* controls = new QWidget();
    controls->setStyleSheet("background-color: #1e1e24; color: white; border-top: 1px solid #333; padding: 10px;");
    auto* ctrl_layout = new QGridLayout(controls);

    auto add_slider = [&](const QString& name, QSlider*& slider_ptr, int min_v, int max_v, int def_v, int row, int col) {
        auto* l = new QVBoxLayout();
        auto* lbl = new QLabel(name);
        lbl->setAlignment(Qt::AlignCenter);
        l->addWidget(lbl);
        slider_ptr = new QSlider(Qt::Horizontal);
        slider_ptr->setRange(min_v, max_v);
        slider_ptr->setValue(def_v);
        connect(slider_ptr, &QSlider::valueChanged, this, &MainWindow::update_physics_params);
        l->addWidget(slider_ptr);
        ctrl_layout->addLayout(l, row, col);
    };
    add_slider("Gravity", grav_slider_, 0, 100, 30, 0, 0);
    add_slider("Bounciness (at 20 °C)", elas_slider_, 10, 100, 80, 0, 1);
    add_slider("Wind Drag", wind_slider_, 0, 50, 10, 0, 2);
    add_slider("Room Setpoint (°C)", temp_slider_, -100, 300, 20, 1, 0);
    add_slider("Heat Transfer Rate", heat_slider_, 0, 100, 5, 1, 1);
    add_slider("Fluid Viscosity (at 20 °C)", visc_slider_, 0, 100, 5, 1, 2);

    right_layout->addWidget(controls);
    sandbox_layout->addWidget(right_container, 1);
    stack_->addWidget(sandbox_widget);

    install_shortcuts();
    select_tool(ObjectType::BALL);
    update_history_buttons();

    // Size the world to the canvas before placing the starting balls.
    stack_->setCurrentWidget(sandbox_widget);
    sandbox_widget->layout()->activate();
    canvas_->world.width = std::max(canvas_->width(), 400);
    canvas_->world.height = std::max(canvas_->height(), 300);
    canvas_->restart(num_balls);
}

void MainWindow::install_shortcuts() {
    auto add = [this](QList<QKeySequence> keys, auto slot) {
        auto* sc = new QShortcut(this);
        sc->setKeys(keys);
        sc->setContext(Qt::WindowShortcut);
        connect(sc, &QShortcut::activated, this, slot);
    };
    // Redo: the platform binding plus both common spellings, without duplicates
    // (a key bound twice would be ambiguous and fire neither).
    QList<QKeySequence> redo_keys = QKeySequence::keyBindings(QKeySequence::Redo);
    for (const QKeySequence& extra : {QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_Z), QKeySequence(Qt::CTRL | Qt::Key_Y)})
        if (!redo_keys.contains(extra)) redo_keys.append(extra);

    add(QKeySequence::keyBindings(QKeySequence::Undo), [this]() { canvas_->undo(); });
    add(redo_keys, [this]() { canvas_->redo(); });
    add(QKeySequence::keyBindings(QKeySequence::Save), [this]() { save_scene(); });
    add(QKeySequence::keyBindings(QKeySequence::Open), [this]() { open_scene(); });
    add({QKeySequence(Qt::Key_Space)}, [this]() { toggle_pause(); });
}

void MainWindow::select_tool(ObjectType type) {
    canvas_->set_tool(type);
    for (auto& [button, tool] : tool_buttons_) button->setChecked(tool == type);
}

void MainWindow::toggle_pause() {
    canvas_->set_paused(!canvas_->is_paused());
    pause_btn_->setText(canvas_->is_paused() ? "Resume Physics" : "Pause Physics");
    pause_btn_->setStyleSheet(canvas_->is_paused() ? kPausedStyle : kRunStyle);
}

void MainWindow::update_physics_params() {
    World& w = canvas_->world;
    w.gravity.y = grav_slider_->value() / 100.0;
    w.air_resistance = wind_slider_->value() / 1000.0;
    w.set_elasticity(elas_slider_->value() / 100.0);
    w.ambient_setpoint = temp_slider_->value();
    w.heat_transfer_rate = heat_slider_->value() / 100.0;
    w.fluid_viscosity = visc_slider_->value() / 100.0;
}

void MainWindow::sync_sliders_from_world() {
    const World& w = canvas_->world;
    auto set = [](QSlider* s, double v) { QSignalBlocker block(s); s->setValue(int(std::lround(v))); };
    set(grav_slider_, w.gravity.y * 100.0);
    set(wind_slider_, w.air_resistance * 1000.0);
    set(elas_slider_, w.elasticity * 100.0);
    set(temp_slider_, w.ambient_setpoint);
    set(heat_slider_, w.heat_transfer_rate * 100.0);
    set(visc_slider_, w.fluid_viscosity * 100.0);
}

void MainWindow::update_history_buttons() {
    if (undo_btn_) undo_btn_->setEnabled(canvas_->can_undo());
    if (redo_btn_) redo_btn_->setEnabled(canvas_->can_redo());
}

void MainWindow::save_scene() {
    const QString path = QFileDialog::getSaveFileName(this, "Save scene", "scene.sapphire", kSceneFilter);
    if (path.isEmpty()) return;
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        QMessageBox::warning(this, "Save scene", "Could not write " + path + ":\n" + file.errorString());
        return;
    }
    file.write(QByteArray::fromStdString(canvas_->scene_text()));
}

void MainWindow::open_scene() {
    const QString path = QFileDialog::getOpenFileName(this, "Open scene", QString(), kSceneFilter);
    if (path.isEmpty()) return;
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "Open scene", "Could not read " + path + ":\n" + file.errorString());
        return;
    }
    QString error;
    if (!canvas_->load_scene(file.readAll().toStdString(), &error))
        QMessageBox::warning(this, "Open scene", "This is not a valid Sapphire scene (" + error + ").");
}
