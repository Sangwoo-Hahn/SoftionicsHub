#ifndef SOFTIONICS_GUI_FORMATDOUBLESPINBOX_H
#define SOFTIONICS_GUI_FORMATDOUBLESPINBOX_H

#include <QDoubleSpinBox>
#include <QLocale>
#include <QString>
#include <QValidator>
#include <algorithm>
#include <cmath>

class FormatDoubleSpinBox : public QDoubleSpinBox {
public:
    enum class Mode {
        Fixed,
        Scientific
    };

    explicit FormatDoubleSpinBox(QWidget* parent = nullptr)
        : QDoubleSpinBox(parent) {
        setLocale(QLocale::c());
        setKeyboardTracking(false);
        QDoubleSpinBox::setDecimals(18); // 기본적으로 작은 값도 0으로 안 죽게
    }

    void setMode(Mode m) { mode_ = m; update(); }

    // Fixed 표시 + 내부 반올림(decimals)도 같이 맞춤
    void setFixedDecimals(int d) {
        fixedDecimals_ = std::max(0, d);
        QDoubleSpinBox::setDecimals(fixedDecimals_);
        update();
    }

    // Scientific 표시 자릿수(표시용) + 내부 decimals(저장용)
    void setSciDigits(int displayDigits, int internalDecimals) {
        sciDigits_ = std::max(0, displayDigits);
        QDoubleSpinBox::setDecimals(std::max(0, internalDecimals));
        update();
    }

protected:
    QString textFromValue(double v) const override {
        if (mode_ == Mode::Scientific) {
            return QLocale::c().toString(v, 'e', sciDigits_);
        } else {
            return QLocale::c().toString(v, 'f', fixedDecimals_);
        }
    }

    double valueFromText(const QString& text) const override {
        bool ok = false;
        double v = QLocale::c().toDouble(text.trimmed(), &ok);
        if (!ok) return 0.0;
        return v;
    }

    QValidator::State validate(QString& text, int& pos) const override {
        Q_UNUSED(pos);
        const QString t = text.trimmed();
        if (t.isEmpty() || t == "-" || t == "+" || t == "." || t == "-." || t == "+.")
            return QValidator::Intermediate;

        bool ok = false;
        double v = QLocale::c().toDouble(t, &ok);
        if (!ok) return QValidator::Invalid;

        if (v < minimum() || v > maximum())
            return QValidator::Intermediate;

        return QValidator::Acceptable;
    }

    // Scientific 모드: 한 step = ×10 / ÷10
    void stepBy(int steps) override {
        if (mode_ != Mode::Scientific) {
            QDoubleSpinBox::stepBy(steps);
            return;
        }

        double v = value();
        if (v == 0.0) v = (steps > 0) ? 1.0 : 0.1;

        const double factor = std::pow(10.0, (double)steps);
        v *= factor;

        v = std::min(std::max(v, minimum()), maximum());
        setValue(v);
    }

private:
    Mode mode_ = Mode::Fixed;
    int fixedDecimals_ = 6;
    int sciDigits_ = 6;
};

#endif
