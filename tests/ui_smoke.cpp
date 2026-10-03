// End-to-end smoke test of the Qt app: drives the real window with synthetic
// mouse/keyboard input, checks undo/redo, dragging, hover and save/load, and
// writes a screenshot.  Run headless with QT_QPA_PLATFORM=offscreen.
//
//   ./sapphire_ui_smoke [screenshot.png]

#include "main_window.h"
#include "physics_canvas.h"

#include <QApplication>
#include <QInputDialog>
#include <QPushButton>
#include <QTest>
#include <QTimer>

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
    QApplication app(argc, argv);
    const QString shot = argc > 1 ? QString(argv[1]) : QString("sapphire_screenshot.png");

    MainWindow window;
    window.resize(1200, 900);
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
    check(w.width > 500 && w.height > 400, "world sized to the canvas");

    auto place = [&](const char* tool, int x, int y) {
        QTest::mouseClick(button(window, tool), Qt::LeftButton);
        QTest::mouseClick(canvas, Qt::LeftButton, Qt::NoModifier, QPoint(x, y));
    };

    // ---- Build a scene through the UI ----
    place("Water Drop", 60, 330);
    place("Water Drop", 200, 330);
    place("Container", 640, 520);
    place("Heater", 640, 660);
    place("Rope Chain", 470, 40);
    place("Car", 300, 120);
    place("Box", 120, 200);
    place("Wheel", 230, 220);
    place("Gas Cloud", 880, 300);
    check(!w.containers.empty(), "vessel spawned");
    check(w.springs.size() == 5, "car spawned where clicked");

    // Spring between two balls drawn with the link tool.
    QTest::mouseClick(button(window, "Ball"), Qt::LeftButton);
    QTest::mouseClick(canvas, Qt::LeftButton, Qt::NoModifier, QPoint(820, 120));
    QTest::mouseClick(canvas, Qt::LeftButton, Qt::NoModifier, QPoint(900, 120));
    PhysicsObject* left = w.objects[w.objects.size() - 2].get();
    PhysicsObject* right = w.objects.back().get();
    QTest::mouseClick(button(window, "Bouncy Spring"), Qt::LeftButton);
    QTest::mousePress(canvas, Qt::LeftButton, Qt::NoModifier,
                      QPoint(int(left->position.x), int(left->position.y)));
    QTest::mouseRelease(canvas, Qt::LeftButton, Qt::NoModifier,
                        QPoint(int(right->position.x), int(right->position.y)));
    check(w.springs.size() == 6, "spring linked with the link tool");

    // ---- Undo / redo ----
    const std::size_t before = w.objects.size();
    QTest::mouseClick(button(window, "Ball"), Qt::LeftButton);
    QTest::mouseClick(canvas, Qt::LeftButton, Qt::NoModifier, QPoint(1000, 80));
    check(w.objects.size() == before + 1, "click spawns a ball");
    QTest::keyClick(&window, Qt::Key_Z, Qt::ControlModifier);
    check(w.objects.size() == before, "Ctrl+Z removes it");
    QTest::keyClick(&window, Qt::Key_Z, Qt::ControlModifier | Qt::ShiftModifier);
    check(w.objects.size() == before + 1, "Ctrl+Shift+Z brings it back");
    QTest::keyClick(&window, Qt::Key_Z, Qt::ControlModifier);

    // ---- Let it run ----
    QTest::qWait(2500);
    check(w.ambient_temperature > 20.0, "heater warms the room");

    // ---- Drag and throw a ball ----
    PhysicsObject* ball = nullptr;
    for (const auto& o : w.objects)
        if (!o->is_fluid && !o->is_gas && !o->is_static && o->shape_type == ShapeType::CIRCLE && o->radius == 18.0 &&
            o->group == 0 && o->position.y > 300) { ball = o.get(); break; }
    check(ball != nullptr, "found a ball to throw");
    if (ball) {
        QPoint p(int(ball->position.x), int(ball->position.y));
        QTest::mousePress(canvas, Qt::LeftButton, Qt::NoModifier, p);
        for (int i = 0; i < 6; ++i) {
            p += QPoint(0, -25);
            QTest::mouseMove(canvas, p, 8);          // 8 ms apart (event timestamps)
        }
        QTest::mouseRelease(canvas, Qt::LeftButton, Qt::NoModifier, p, 1);
        check(ball->velocity.y < -5.0, "a quick flick throws the ball upwards");
    }

    // ---- Save / load ----
    const std::string scene = canvas->scene_text();
    QString error;
    check(canvas->load_scene(scene, &error), "scene reloads from its own text");
    check(canvas->scene_text() == scene, "reload is lossless");
    check(!canvas->load_scene("not a scene", &error), "garbage is rejected with an error");

    QTest::qWait(1200);
    // Hover the vessel piston for the readout in the screenshot.
    if (!w.containers.empty()) {
        const GasContainer& c = w.containers[0];
        QTest::mouseMove(canvas, QPoint(int(c.left + 40), int(c.top())));
    }
    QTest::qWait(60);
    window.grab().save(shot);
    std::printf("screenshot: %s\n", qPrintable(shot));
    std::printf("%s\n", failures == 0 ? "UI smoke test passed" : "UI smoke test FAILED");
    return failures == 0 ? 0 : 1;
}
