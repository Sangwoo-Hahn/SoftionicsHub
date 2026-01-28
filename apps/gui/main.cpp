#include <QApplication>
#include <QStyleFactory>
#include <QFont>
#include "MainWindow.h"

static QString win10StyleSheet() {
    return R"(
QWidget { background: #F3F3F3; color: #202020; }
QMainWindow { background: #F3F3F3; }

QGroupBox {
  background: #FFFFFF;
  border: 1px solid #DADADA;
  border-radius: 8px;
  margin-top: 14px;
}
QGroupBox::title {
  subcontrol-origin: margin;
  subcontrol-position: top left;
  padding: 0 6px;
  left: 10px;
  color: #404040;
}

QListWidget {
  background: #FFFFFF;
  border: 1px solid #DADADA;
  border-radius: 8px;
  padding: 4px;
}
QListWidget::item {
  padding: 8px 10px;
  border-radius: 6px;
}
QListWidget::item:hover {
  background: #F2F8FF;
}
QListWidget::item:selected {
  background: #E6F1FF;
}

QPushButton {
  background: #0078D7;
  color: #FFFFFF;
  border: 1px solid #0078D7;
  border-radius: 6px;
  padding: 8px 10px;
}
QPushButton:hover { background: #0A84E3; border-color: #0A84E3; }
QPushButton:pressed { background: #006ABC; border-color: #006ABC; }
QPushButton:disabled { background: #B8B8B8; border-color: #B8B8B8; }

QCheckBox { spacing: 8px; }
QLineEdit {
  background: #FFFFFF;
  border: 1px solid #DADADA;
  border-radius: 6px;
  padding: 6px 8px;
}
QDoubleSpinBox, QSpinBox {
  background: #FFFFFF;
  border: 1px solid #DADADA;
  border-radius: 6px;
  padding: 4px 6px;
}

QSplitter::handle { background: #E6E6E6; }
QLabel#StatusLabel {
  background: #FFFFFF;
  border: 1px solid #DADADA;
  border-radius: 8px;
  padding: 8px 10px;
}
    )";
}

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    app.setStyle(QStyleFactory::create("Fusion"));
    app.setFont(QFont("Segoe UI", 9));
    app.setStyleSheet(win10StyleSheet());

    MainWindow w;
    w.show();
    return app.exec();
}
