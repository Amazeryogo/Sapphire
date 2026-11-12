// End-to-end smoke test of the Qt app: drives the real window with synthetic
// mouse/keyboard input, checks spawning in 3D, undo/redo, dragging, the
// camera and save/load, and writes a screenshot.  Run headless with
// QT_QPA_PLATFORM=offscreen (the picture is then blank where OpenGL is not
// available, but everything else is still exercised).
//
//   ./sapphire_ui_smoke [screenshot.png]

#include "main_window.h"
#include "physics_canvas.h"

#include <QApplication>
#include <QInputDialog>
#include <QPushButton>
#include <QSurfaceFormat>
#include <QTest>
#include <QTimer>

#include <cmath>
#include <cstdio>

namespace {
int failures = 0;
void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "  ok  " : "  FAIL", what);
    if (!ok) ++failures;
}

QPushButton* button(QWidget& root, const QString& text) {
    for (QPushButton* b : root.findChildren<QPushButton*>())
        if (b->text() == text) return b;
    return nullptr;
}
} // namespace

int main(int argc, char* argv[]) {
    QSurfaceFormat format;
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication app(argc, argv);
    const QString shot = argc > 1 ? QString(argv[1]) : QString("sapphire_screenshot.png");

    MainWindow window;
    window.resize(1280, 900);
    window.show();
    window.activateWindow();
    if (!QTest::qWaitForWindowActive(&window)) std::printf("  (window not active)\n");

    // Enter the sandbox, answering the start dialog with 6 balls.
    QTimer::singleShot(100, [] {
        if (auto* dialog = qobject_cast<QInputDialog*>(QApplication::activeModalWidget())) {
            dialog->setIntValue(6);
            dialog->accept();
        }
    });
    QTest::mouseClick(button(window, "Enter Sandbox"), Qt::LeftButton);
    QTest::qWait(150);
    // The offscreen platform does not hand activation back after a dialog
    // closes (a desktop window manager does); shortcuts need it.
    window.activateWindow();
    if (!QTest::qWaitForWindowActive(&window)) std::printf("  (window not active)\n");

    auto* canvas = window.findChild<PhysicsCanvas*>();
    check(canvas != nullptr, "sandbox opened");
    if (!canvas) return 1;
    World& w = canvas->world;
    check(w.objects.size() == 6, "starting balls spawned");
    check(canvas->width() > 500 && canvas->height() > 400, "3D view has a sensible size");
    QTest::mouseClick(button(window, "Clear All"), Qt::LeftButton);
    check(w.objects.empty(), "Clear All empties the room");

    // A click puts the object where the cursor's ray meets the vertical
    // plane through the middle of the room (z = 250 with the default camera).
    auto pixel = [&](const Vec3& p) {
        QPointF at;
        canvas->screen_point(p, at);
        return at.toPoint();
    };
    auto place = [&](const char* tool, const Vec3& p) {
        QTest::mouseClick(button(window, tool), Qt::LeftButton);
        QTest::mouseClick(canvas, Qt::LeftButton, Qt::NoModifier, pixel(p));
    };

    // ---- Build a scene through the UI ----
    place("Pin (Static)", Vec3(860, 500, 250));
    check(w.objects.size() == 1, "a click spawns one pin");
    const Vec3 pin = w.objects.back()->position;
    check(std::abs(pin.x - 860) < 4 && std::abs(pin.y - 500) < 4 && std::abs(pin.z - 250) < 4,
          "it appears under the cursor, at mid depth");

    place("Water Tank", Vec3(150, 100, 250));
    check(w.objects.size() == 6, "tank is five glass walls");
    place("Water Drop", Vec3(150, 120, 250));
    check(w.objects.size() == 6 + 512, "clicking through the glass pours water into the tank");
    place("Container", Vec3(720, 150, 250));
    check(w.containers.size() == 1, "vessel spawned");
    place("Heater", Vec3(720, 80, 250));          // inside the vessel
    place("Rope Chain", Vec3(580, 540, 250));
    place("Car", Vec3(420, 60, 250));
    check(w.springs.size() == 12 && w.axles.size() == 4, "car spawned with suspension and axles");
    place("Box", Vec3(330, 300, 250));
    place("Wheel", Vec3(560, 200, 250));
    place("Gas Cloud", Vec3(820, 420, 250));

    // Spring between two balls drawn with the link tool.
    place("Ball", Vec3(280, 480, 250));
    PhysicsObject* left = w.objects.back().get();
    place("Ball", Vec3(360, 480, 250));
    PhysicsObject* right = w.objects.back().get();
    QTest::mouseClick(button(window, "Bouncy Spring"), Qt::LeftButton);
    QTest::mousePress(canvas, Qt::LeftButton, Qt::NoModifier, pixel(left->position));
    QTest::mouseRelease(canvas, Qt::LeftButton, Qt::NoModifier, pixel(right->position));
    check(w.springs.size() == 13, "spring linked with the link tool");

    // ---- Undo / redo ----
    const std::size_t before = w.objects.size();
    place("Ball", Vec3(860, 300, 250));
    check(w.objects.size() == before + 1, "click spawns a ball");
    QTest::keyClick(&window, Qt::Key_Z, Qt::ControlModifier);
    check(w.objects.size() == before, "Ctrl+Z removes it");
    QTest::keyClick(&window, Qt::Key_Z, Qt::ControlModifier | Qt::ShiftModifier);
    check(w.objects.size() == before + 1, "Ctrl+Shift+Z brings it back");
    w.objects.back()->velocity = Vec3();      // let it drop straight down, to be picked up below

    // ---- Let it run ----
    QTest::qWait(2500);
    check(w.ambient_temperature > 20.0, "heater warms the room");
    check(w.containers[0].gas_particles == World::kVesselAirParticles, "vessel keeps its gas");

    // ---- Drag and throw the last ball (it has fallen to the floor by now) ----
    PhysicsObject* ball = w.objects.back().get();
    check(ball->shape_type == ShapeType::SPHERE && ball->radius == 18.0 && ball->position.y < 60.0,
          "found the ball on the floor");
    {
        QPoint p = pixel(ball->position);
        QTest::mousePress(canvas, Qt::LeftButton, Qt::NoModifier, p);
        check(w.dragged() == ball, "pressing on a ball grabs it");
        for (int i = 0; i < 6; ++i) {
            p += QPoint(0, -25);
            QTest::mouseMove(canvas, p, 8);          // 8 ms apart (event timestamps)
        }
        QTest::mouseRelease(canvas, Qt::LeftButton, Qt::NoModifier, p, 1);
        check(ball->velocity.y > 5.0, "a quick flick throws the ball upwards");
    }

    // ---- Camera: right-drag orbits, right-click removes ----
    const std::size_t count = w.objects.size();
    const double yaw = canvas->camera.yaw;
    QPoint c(canvas->width() / 2, 60);
    QTest::mousePress(canvas, Qt::RightButton, Qt::NoModifier, c);
    for (int i = 0; i < 5; ++i) { c += QPoint(20, 0); QTest::mouseMove(canvas, c, 5); }
    QTest::mouseRelease(canvas, Qt::RightButton, Qt::NoModifier, c);
    check(std::abs(canvas->camera.yaw - yaw) > 0.3, "right-drag orbits the camera");
    check(w.objects.size() == count, "orbiting removes nothing");
    QTest::mouseClick(button(window, "Reset View"), Qt::LeftButton);
    check(std::abs(canvas->camera.yaw) < 1e-9, "Reset View puts the camera back");
    QTest::mouseClick(canvas, Qt::RightButton, Qt::NoModifier, pixel(pin));
    check(w.objects.size() == count - 1, "right-click removes the pin");

    // ---- Save / load ----
    canvas->set_paused(true);
    const std::string scene = canvas->scene_text();
    QString error;
    check(canvas->load_scene(scene, &error), "scene reloads from its own text");
    check(canvas->scene_text() == scene, "reload is lossless");
    check(!canvas->load_scene("not a scene", &error), "garbage is rejected with an error");
    canvas->set_paused(false);

    QTest::qWait(1200);
    // Hover the vessel piston for the readout in the screenshot.
    if (!w.containers.empty() && w.containers[0].piston)
        QTest::mouseMove(canvas, pixel(w.containers[0].piston->position), 30);
    QTest::qWait(120);
    window.screenshot().save(shot);
    std::printf("screenshot: %s\n", qPrintable(shot));
    std::printf("%s\n", failures == 0 ? "UI smoke test passed" : "UI smoke test FAILED");
    return failures == 0 ? 0 : 1;
}
