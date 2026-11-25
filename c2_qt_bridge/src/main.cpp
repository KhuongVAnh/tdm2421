#include "MainWindow.h"
#include <QApplication>

/// Điểm khởi đầu của ứng dụng TDM2421 C2 Bridge (Qt 6 Widgets).
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("TDM2421 C2 Bridge"));
    app.setOrganizationName(QStringLiteral("TDLOGY")); // Dùng chung cho QSettings lưu cấu hình người dùng.

    MainWindow window;
    window.show();
    return app.exec();
}
