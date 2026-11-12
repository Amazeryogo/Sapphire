#include "main_window.h"

#include <QApplication>
#include <QCommandLineParser>
#include <QSurfaceFormat>
#include <QTimer>

#include <cstdio>

int main(int argc, char* argv[]) {
    // The 3D view needs an OpenGL 3.3 core context; on macOS that has to be
    // the default format before the application object exists.
    QSurfaceFormat format;
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setSamples(4);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication app(argc, argv);
    QApplication::setApplicationName("Sapphire");

    QCommandLineParser parser;
    parser.setApplicationDescription("3D physics and thermodynamics sandbox");
    parser.addHelpOption();
    const QCommandLineOption demo({"d", "demo"}, "Open straight into the demo scene.");
    const QCommandLineOption screenshot("screenshot", "Run for a while, save a picture of the window and quit.", "file.png");
    const QCommandLineOption frames("frames", "How many frames to simulate before --screenshot (default 240).", "n", "240");
    parser.addOptions({demo, screenshot, frames});
    parser.process(app);

    MainWindow window;
    window.show();
    if (parser.isSet(demo) || parser.isSet(screenshot)) {
        window.enter_sandbox(0);
        window.canvas()->load_demo();
    }
    if (parser.isSet(screenshot)) {
        const QString path = parser.value(screenshot);
        const int count = std::max(1, parser.value(frames).toInt());
        QTimer::singleShot(300, &window, [&window, path, count]() {
            PhysicsCanvas* canvas = window.canvas();
            canvas->set_paused(true);
            for (int i = 0; i < count; ++i) canvas->world.step();
            canvas->update();
            QTimer::singleShot(200, &window, [&window, path]() {
                const bool ok = window.screenshot().save(path);
                std::printf("%s %s\n", ok ? "saved" : "could not save", qPrintable(path));
                QApplication::quit();
            });
        });
    }
    return app.exec();
}
