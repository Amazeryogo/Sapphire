#include "main_window.h"

#include <QComboBox>
#include <QFile>
#include <QFileDialog>
#include <QFont>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QKeySequence>
#include <QLabel>
#include <QMessageBox>
#include <QPainter>
#include <QPushButton>
#include <QShortcut>
#include <QSignalBlocker>
#include <QSlider>
#include <QStackedWidget>
#include <QVBoxLayout>

#include <cmath>

namespace {
const char* kToolStyle =
    "QPushButton { background-color: #3e4451; color: white; padding: 5px; border-radius: 4px; }"
    "QPushButton:checked { background-color: #61afef; color: black; font-weight: bold; }";
const char* kLinkStyle =
    "QPushButton { background-color: #d19a66; color: black; padding: 5px; border-radius: 4px; }"
    "QPushButton:checked { background-color: #e5c07b; font-weight: bold; }";
const char* kSmallStyle =
    "QPushButton { background-color: #3e4451; color: white; padding: 5px; border-radius: 4px; }"
    "QPushButton:disabled { color: #6b7280; }";
const char* kRunStyle = "background-color: #98c379; color: black; padding: 9px; font-weight: bold;";
const char* kPausedStyle = "background-color: #e5c07b; color: black; padding: 9px; font-weight: bold;";
const char* kSceneFilter = "Sapphire scenes (*.sapphire);;All files (*)";
const char* kHeading = "<b style='color:#abb2bf; font-size:13px'>%1</b>";

// m/s² → cm/frame² (100 cm per metre, 60 frames per second).
constexpr double kGravityUnit = 100.0 / 3600.0;
} // namespace

MainWindow::MainWindow() {
    setWindowTitle("Sapphire 3D");
    resize(1280, 900);
    setStyleSheet("background-color: #282c34;");

    stack_ = new QStackedWidget(this);
    setCentralWidget(stack_);

    auto* intro_page = new QWidget();
    auto* intro_layout = new QVBoxLayout(intro_page);
    intro_layout->setAlignment(Qt::AlignCenter);

    auto* title = new QLabel("Sapphire");
    title->setFont(QFont("Arial", 32, QFont::Bold));
    title->setStyleSheet("color: #61afef; margin-bottom: 4px;");
    intro_layout->addWidget(title, 0, Qt::AlignCenter);
    auto* subtitle = new QLabel("3D physics and thermodynamics sandbox");
    subtitle->setStyleSheet("color: #abb2bf; font-size: 15px; margin-bottom: 20px;");
    intro_layout->addWidget(subtitle, 0, Qt::AlignCenter);

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
    enter_sandbox(num_balls);
}

void MainWindow::enter_sandbox(int initial_balls) {
    if (!canvas_) setup_sandbox(initial_balls);
    stack_->setCurrentIndex(1);
}

void MainWindow::setup_sandbox(int num_balls) {
    auto* sandbox_widget = new QWidget();
    auto* sandbox_layout = new QHBoxLayout(sandbox_widget);
    sandbox_layout->setContentsMargins(0, 0, 0, 0);
    sandbox_layout->setSpacing(0);

    auto* toolbox = new QWidget();
    toolbox->setFixedWidth(230);
    toolbox->setStyleSheet("background-color: #21252b; border-right: 1px solid #333;");
    auto* tb_layout = new QVBoxLayout(toolbox);
    tb_layout->setSpacing(5);

    auto add_tools = [&](const std::vector<std::pair<QString, ObjectType>>& tools, const char* style) {
        auto* grid = new QGridLayout();
        grid->setSpacing(4);
        int index = 0;
        for (const auto& [label, type] : tools) {
            auto* btn = new QPushButton(label);
            btn->setCheckable(true);
            btn->setStyleSheet(style);
            connect(btn, &QPushButton::clicked, this, [this, type = type]() { select_tool(type); });
            grid->addWidget(btn, index / 2, index % 2);
            tool_buttons_.push_back({btn, type});
            ++index;
        }
        tb_layout->addLayout(grid);
    };

    tb_layout->addWidget(new QLabel(QString(kHeading).arg("SPAWN TOOLS")));
    add_tools({
        {"Ball", ObjectType::BALL}, {"Box", ObjectType::BOX},
        {"Wheel", ObjectType::WHEEL}, {"Motor", ObjectType::MOTOR},
        {"Car", ObjectType::CHASSIS}, {"Draw Shape", ObjectType::POLYGON},
        {"Pin (Static)", ObjectType::PIN}, {"Rope Chain", ObjectType::ROPE},
        {"Platform", ObjectType::PLATFORM}, {"Ramp", ObjectType::RAMP},
        {"Water Drop", ObjectType::WATER}, {"Water Tank", ObjectType::TANK},
        {"Gas Cloud", ObjectType::GAS}, {"Container", ObjectType::CONTAINER},
        {"Heater", ObjectType::HEATER}, {"Cooler", ObjectType::COOLER},
        {"Shockwave Bomb", ObjectType::BOMB},
    }, kToolStyle);

    tb_layout->addWidget(new QLabel(QString(kHeading).arg("MATERIAL")));
    material_box_ = new QComboBox();
    material_box_->setStyleSheet("QComboBox { background-color: #3e4451; color: white; padding: 4px; }"
                                 "QComboBox QAbstractItemView { background-color: #3e4451; color: white; }");
    for (int m = 0; m < int(Material::Count); ++m) {
        const MaterialProps& props = material_props(Material(m));
        material_box_->addItem(QString("%1  (%2 g/cm³)").arg(props.name).arg(props.density, 0, 'f', 2));
    }
    material_box_->setToolTip("What new balls, boxes, wheels and shapes are made of.\n"
                              "Lighter than water (1.00) floats; rubber follows the Bounciness slider.");
    connect(material_box_, &QComboBox::currentIndexChanged, this, [this](int index) {
        if (canvas_) canvas_->world.spawn_material = material_from_int(index);
    });
    tb_layout->addWidget(material_box_);

    tb_layout->addWidget(new QLabel(QString(kHeading).arg("LINK TOOLS")));
    add_tools({
        {"Bouncy Spring", ObjectType::SPRING}, {"Pulley", ObjectType::PULLEY}, {"Piston", ObjectType::PISTON},
    }, kLinkStyle);

    tb_layout->addStretch();

    // ---- View, history and files ----
    auto* edit_row = new QGridLayout();
    edit_row->setSpacing(4);
    auto* view_btn = new QPushButton("Reset View");
    auto* demo_btn = new QPushButton("Demo Scene");
    undo_btn_ = new QPushButton("Undo");
    redo_btn_ = new QPushButton("Redo");
    auto* save_btn = new QPushButton("Save…");
    auto* open_btn = new QPushButton("Open…");
    view_btn->setToolTip("Put the camera back (Home)");
    demo_btn->setToolTip("Replace the scene with a tour of the sandbox");
    undo_btn_->setToolTip("Undo (Ctrl+Z)");
    redo_btn_->setToolTip("Redo (Ctrl+Shift+Z)");
    save_btn->setToolTip("Save scene (Ctrl+S)");
    open_btn->setToolTip("Open scene (Ctrl+O)");
    for (QPushButton* b : {view_btn, demo_btn, undo_btn_, redo_btn_, save_btn, open_btn}) b->setStyleSheet(kSmallStyle);
    edit_row->addWidget(view_btn, 0, 0);
    edit_row->addWidget(demo_btn, 0, 1);
    edit_row->addWidget(undo_btn_, 1, 0);
    edit_row->addWidget(redo_btn_, 1, 1);
    edit_row->addWidget(save_btn, 2, 0);
    edit_row->addWidget(open_btn, 2, 1);
    tb_layout->addLayout(edit_row);

    pause_btn_ = new QPushButton("Pause Physics");
    pause_btn_->setToolTip("Pause / resume (Space)");
    pause_btn_->setStyleSheet(kRunStyle);
    connect(pause_btn_, &QPushButton::clicked, this, &MainWindow::toggle_pause);
    tb_layout->addWidget(pause_btn_);

    auto* clear_btn = new QPushButton("Clear All");
    clear_btn->setStyleSheet("background-color: #e06c75; color: white; padding: 9px; font-weight: bold;");
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
    connect(view_btn, &QPushButton::clicked, canvas_, &PhysicsCanvas::reset_view);
    connect(demo_btn, &QPushButton::clicked, this, [this]() {
        canvas_->push_undo();
        canvas_->load_demo();
    });
    connect(undo_btn_, &QPushButton::clicked, canvas_, &PhysicsCanvas::undo);
    connect(redo_btn_, &QPushButton::clicked, canvas_, &PhysicsCanvas::redo);
    connect(save_btn, &QPushButton::clicked, this, &MainWindow::save_scene);
    connect(open_btn, &QPushButton::clicked, this, &MainWindow::open_scene);
    connect(canvas_, &PhysicsCanvas::history_changed, this, &MainWindow::update_history_buttons);
    connect(canvas_, &PhysicsCanvas::parameters_changed, this, &MainWindow::sync_sliders_from_world);

    auto* controls = new QWidget();
    controls->setStyleSheet("background-color: #1e1e24; color: white; border-top: 1px solid #333; padding: 6px;");
    auto* ctrl_layout = new QGridLayout(controls);

    auto add_slider = [&](QLabel*& label_ptr, QSlider*& slider_ptr, int min_v, int max_v, int def_v, int row, int col) {
        auto* l = new QVBoxLayout();
        label_ptr = new QLabel();
        label_ptr->setAlignment(Qt::AlignCenter);
        l->addWidget(label_ptr);
        slider_ptr = new QSlider(Qt::Horizontal);
        slider_ptr->setRange(min_v, max_v);
        slider_ptr->setValue(def_v);
        connect(slider_ptr, &QSlider::valueChanged, this, &MainWindow::update_physics_params);
        l->addWidget(slider_ptr);
        ctrl_layout->addLayout(l, row, col);
    };
    add_slider(grav_label_, grav_slider_, 0, 250, 98, 0, 0);       // tenths of m/s²
    add_slider(elas_label_, elas_slider_, 10, 100, 80, 0, 1);      // percent
    add_slider(air_label_, air_slider_, 0, 100, 1, 0, 2);          // × real air density
    add_slider(time_label_, time_slider_, 10, 200, 100, 0, 3);     // percent of real time
    add_slider(temp_label_, temp_slider_, -100, 300, 20, 1, 0);    // °C
    add_slider(heat_label_, heat_slider_, 0, 100, 5, 1, 1);
    add_slider(visc_label_, visc_slider_, 0, 100, 5, 1, 2);

    right_layout->addWidget(controls);
    sandbox_layout->addWidget(right_container, 1);
    stack_->addWidget(sandbox_widget);

    install_shortcuts();
    select_tool(ObjectType::BALL);
    update_history_buttons();
    update_physics_params();

    stack_->setCurrentWidget(sandbox_widget);
    sandbox_widget->layout()->activate();
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
    add({QKeySequence(Qt::Key_Home)}, [this]() { canvas_->reset_view(); });
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

void MainWindow::update_slider_labels() {
    grav_label_->setText(QString("Gravity  %1 m/s²").arg(grav_slider_->value() / 10.0, 0, 'f', 1));
    elas_label_->setText(QString("Rubber Bounciness  %1 (at 20 °C)").arg(elas_slider_->value() / 100.0, 0, 'f', 2));
    air_label_->setText(QString("Air Density  %1× real").arg(air_slider_->value()));
    time_label_->setText(QString("Time Scale  %1 %").arg(time_slider_->value()));
    temp_label_->setText(QString("Room Setpoint  %1 °C").arg(temp_slider_->value()));
    heat_label_->setText(QString("Heat Transfer Rate  %1").arg(heat_slider_->value() / 100.0, 0, 'f', 2));
    visc_label_->setText(QString("Water Viscosity  %1 (at 20 °C)").arg(visc_slider_->value() / 100.0, 0, 'f', 2));
}

void MainWindow::update_physics_params() {
    update_slider_labels();
    if (!canvas_) return;
    World& w = canvas_->world;
    w.gravity = Vec3(0.0, -grav_slider_->value() / 10.0 * kGravityUnit, 0.0);
    w.air_density_scale = air_slider_->value();
    w.set_elasticity(elas_slider_->value() / 100.0);
    w.ambient_setpoint = temp_slider_->value();
    w.heat_transfer_rate = heat_slider_->value() / 100.0;
    w.fluid_viscosity = visc_slider_->value() / 100.0;
    canvas_->set_time_scale(time_slider_->value() / 100.0);
}

void MainWindow::sync_sliders_from_world() {
    const World& w = canvas_->world;
    auto set = [](QSlider* s, double v) { QSignalBlocker block(s); s->setValue(int(std::lround(v))); };
    set(grav_slider_, -w.gravity.y / kGravityUnit * 10.0);
    set(air_slider_, w.air_density_scale);
    set(elas_slider_, w.elasticity * 100.0);
    set(temp_slider_, w.ambient_setpoint);
    set(heat_slider_, w.heat_transfer_rate * 100.0);
    set(visc_slider_, w.fluid_viscosity * 100.0);
    {
        QSignalBlocker block(material_box_);
        material_box_->setCurrentIndex(int(w.spawn_material));
    }
    update_slider_labels();
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
        QMessageBox::warning(this, "Open scene", "Could not open this scene: " + error + ".");
}

QImage MainWindow::screenshot() {
    // QWidget::grab() does not capture an OpenGL widget's overlay reliably,
    // so the 3D view is read back from its own framebuffer and pasted in.
    QImage image = grab().toImage();
    if (canvas_ && canvas_->isVisible() && canvas_->has_3d_view()) {
        const QImage view = canvas_->grab_view();
        if (!view.isNull()) {
            QPainter painter(&image);
            const QPoint origin = canvas_->mapTo(this, QPoint(0, 0));
            painter.drawImage(QRectF(origin, QSizeF(canvas_->size())), view);
        }
    }
    return image;
}
