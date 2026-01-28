#include "BF16Window.h"
#include "BleWorker.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSplitter>
#include <QGroupBox>
#include <QCloseEvent>
#include <QPainter>
#include <QRectF>
#include <algorithm>

#include "hub/model/BF16.h"

BF16Window::BF16Window(BleWorker* worker, QWidget* parent)
    : QMainWindow(parent), worker_(worker) {

    buildUi();

    connect(worker_, &BleWorker::poseReady, this, &BF16Window::onPose);

    timer_ = new QTimer(this);
    timer_->setInterval(16); // 60fps
    connect(timer_, &QTimer::timeout, this, &BF16Window::onTick);
    timer_->start();

    QMetaObject::invokeMethod(worker_, [w = worker_]() { w->setBF16Enabled(true); }, Qt::QueuedConnection);
    onParamsChanged();
}

BF16Window::~BF16Window() {
    if (worker_) {
        QMetaObject::invokeMethod(worker_, [w = worker_]() { w->setBF16Enabled(false); }, Qt::QueuedConnection);
    }
}

void BF16Window::closeEvent(QCloseEvent* e) {
    if (worker_) {
        QMetaObject::invokeMethod(worker_, [w = worker_]() { w->setBF16Enabled(false); }, Qt::QueuedConnection);
    }
    e->accept();
}

void BF16Window::buildUi() {
    setWindowTitle("BF16 Viewer");
    resize(1100, 750);

    auto* central = new QWidget(this);
    auto* root = new QHBoxLayout(central);

    auto* split = new QSplitter(Qt::Horizontal, central);
    split->setChildrenCollapsible(false);

    auto* plotW = new QWidget(split);
    auto* plotL = new QVBoxLayout(plotW);

    chart_ = new QChart();
    chart_->legend()->hide();

    axX_ = new QValueAxis();
    axY_ = new QValueAxis();
    axX_->setLabelFormat("%.3f");
    axY_->setLabelFormat("%.3f");

    chart_->addAxis(axX_, Qt::AlignBottom);
    chart_->addAxis(axY_, Qt::AlignLeft);

    sensors_ = new QScatterSeries(chart_);
    sensors_->setMarkerSize(6.0);
    chart_->addSeries(sensors_);
    sensors_->attachAxis(axX_);
    sensors_->attachAxis(axY_);

    {
        auto sens = hub::BF16Solver::sensor_positions();
        for (int i = 0; i < 16; ++i) sensors_->append(sens[i].x, sens[i].y);
    }

    path_ = new QLineSeries(chart_);
    chart_->addSeries(path_);
    path_->attachAxis(axX_);
    path_->attachAxis(axY_);
    {
        auto ppen = path_->pen();
        ppen.setWidthF(2.0);
        path_->setPen(ppen);
    }

    cur_ = new QScatterSeries(chart_);
    cur_->setMarkerSize(12.0);
    chart_->addSeries(cur_);
    cur_->attachAxis(axX_);
    cur_->attachAxis(axY_);

    view_ = new QChartView(chart_, plotW);
    view_->setRenderHint(QPainter::Antialiasing, true);
    plotL->addWidget(view_, 1);

    lbStats_ = new QLabel("Pose: waiting...", plotW);
    lbStats_->setObjectName("StatusLabel");
    lbStats_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    plotL->addWidget(lbStats_);

    connect(chart_, &QChart::plotAreaChanged, this, [this](const QRectF&) {
        updateAxesAndDraw();
    });

    split->addWidget(plotW);

    auto* ctrlW = new QWidget(split);
    ctrlW->setMinimumWidth(330);
    auto* ctrlL = new QVBoxLayout(ctrlW);

    auto* g = new QGroupBox("BF16 Params", ctrlW);
    auto* gL = new QVBoxLayout(g);

    auto mkRow = [&](const QString& name, QWidget* w) {
        auto* row = new QWidget(g);
        auto* rl = new QHBoxLayout(row);
        rl->addWidget(new QLabel(name, row));
        rl->addWidget(w);
        rl->addStretch(1);
        gL->addWidget(row);
    };

    spR_ = new QDoubleSpinBox(g);
    spR_->setRange(1e3, 1e14);
    spR_->setDecimals(0);
    spR_->setValue(1e8);
    mkRow("RC_R (Ohm)", spR_);

    spC_ = new QDoubleSpinBox(g);
    spC_->setRange(1e-18, 1e-3);
    spC_->setDecimals(18);
    spC_->setSingleStep(1e-10);
    spC_->setValue(5e-10);
    mkRow("RC_C (F)", spC_);

    spAlpha_ = new QDoubleSpinBox(g);
    spAlpha_->setRange(0.0, 1.0);
    spAlpha_->setDecimals(4);
    spAlpha_->setSingleStep(0.01);
    spAlpha_->setValue(0.2);
    mkRow("EMA alpha", spAlpha_);

    spQuiet_ = new QDoubleSpinBox(g);
    spQuiet_->setRange(0.0, 1e9);
    spQuiet_->setDecimals(6);
    spQuiet_->setSingleStep(0.05);
    spQuiet_->setValue(0.3);
    mkRow("Quiet err thresh", spQuiet_);

    spMaxPoints_ = new QSpinBox(g);
    spMaxPoints_->setRange(1, 2000);
    spMaxPoints_->setValue(40);
    mkRow("Path points", spMaxPoints_);

    btnReset_ = new QPushButton("Reset BF16 State", g);
    btnClear_ = new QPushButton("Clear Path", g);
    gL->addWidget(btnReset_);
    gL->addWidget(btnClear_);

    ctrlL->addWidget(g);
    ctrlL->addStretch(1);

    connect(spR_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &BF16Window::onParamsChanged);
    connect(spC_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &BF16Window::onParamsChanged);
    connect(spAlpha_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &BF16Window::onParamsChanged);
    connect(spQuiet_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &BF16Window::onParamsChanged);

    connect(btnReset_, &QPushButton::clicked, this, &BF16Window::onResetSolver);
    connect(btnClear_, &QPushButton::clicked, this, &BF16Window::onClearPath);

    split->addWidget(ctrlW);

    split->setStretchFactor(0, 10);
    split->setStretchFactor(1, 3);

    root->addWidget(split);
    setCentralWidget(central);
}

void BF16Window::onParamsChanged() {
    if (!worker_) return;
    double rc_r = spR_->value();
    double rc_c = spC_->value();
    double a = spAlpha_->value();
    double q = spQuiet_->value();

    QMetaObject::invokeMethod(worker_, [w = worker_, rc_r, rc_c, a, q]() {
        w->setBF16Params(rc_r, rc_c, a, q);
    }, Qt::QueuedConnection);
}

void BF16Window::onResetSolver() {
    if (!worker_) return;
    QMetaObject::invokeMethod(worker_, [w = worker_]() { w->resetBF16(); }, Qt::QueuedConnection);
    last_ = {false,false,0,0,0,0,0,0};
    pending_.clear();
    pathBuf_.clear();
    updateAxesAndDraw();
}

void BF16Window::onClearPath() {
    pathBuf_.clear();
    updateAxesAndDraw();
}

void BF16Window::onPose(double x, double y, double z, double q1, double q2, double err, bool quiet, bool hasPose) {
    pending_.push_back(PosePkt{hasPose, quiet, x, y, z, q1, q2, err});
}

void BF16Window::onTick() {
    if (!pending_.isEmpty()) {
        auto local = pending_;
        pending_.clear();

        for (const auto& p : local) {
            if (!p.hasPose) continue;
            last_ = p;

            if (p.quiet) {
                if (!pathBuf_.isEmpty()) pathBuf_.removeFirst();
            } else {
                pathBuf_.append(QPointF(p.x, p.y));
                while (pathBuf_.size() > spMaxPoints_->value()) pathBuf_.removeFirst();
            }
        }
    }
    updateAxesAndDraw();
}

void BF16Window::updateAxesAndDraw() {
    if (last_.hasPose) {
        QVector<QPointF> cur;
        cur.push_back(QPointF(last_.x, last_.y));
        cur_->replace(cur);
    } else {
        cur_->replace(QVector<QPointF>());
    }
    path_->replace(pathBuf_);

    if (last_.hasPose) {
        double xmm = last_.x * 1000.0;
        double ymm = last_.y * 1000.0;
        double zmm = last_.z * 1000.0;
        lbStats_->setText(QString("x=%1 mm, y=%2 mm, z=%3 mm | q1=%4 q2=%5 | err=%6 | %7")
            .arg(xmm, 0, 'f', 1)
            .arg(ymm, 0, 'f', 1)
            .arg(zmm, 0, 'f', 1)
            .arg(last_.q1, 0, 'g', 6)
            .arg(last_.q2, 0, 'g', 6)
            .arg(last_.err, 0, 'g', 6)
            .arg(last_.quiet ? "QUIET" : "ACTIVE"));
    } else {
        lbStats_->setText("Pose: waiting...");
    }

    auto sens = hub::BF16Solver::sensor_positions();
    double minx = sens[0].x, maxx = sens[0].x;
    double miny = sens[0].y, maxy = sens[0].y;

    for (int i = 1; i < 16; ++i) {
        minx = std::min(minx, sens[i].x); maxx = std::max(maxx, sens[i].x);
        miny = std::min(miny, sens[i].y); maxy = std::max(maxy, sens[i].y);
    }
    for (const auto& p : pathBuf_) {
        minx = std::min(minx, p.x()); maxx = std::max(maxx, p.x());
        miny = std::min(miny, p.y()); maxy = std::max(maxy, p.y());
    }
    if (last_.hasPose) {
        minx = std::min(minx, last_.x); maxx = std::max(maxx, last_.x);
        miny = std::min(miny, last_.y); maxy = std::max(maxy, last_.y);
    }

    double cx = 0.5 * (minx + maxx);
    double cy = 0.5 * (miny + maxy);

    double rx = std::max(1e-6, maxx - minx);
    double ry = std::max(1e-6, maxy - miny);

    QRectF pa = chart_->plotArea();
    double w = std::max(1.0, pa.width());
    double h = std::max(1.0, pa.height());
    double aspect = w / h;

    double ySpan = std::max(ry, rx / aspect);
    double xSpan = ySpan * aspect;

    xSpan *= 1.15;
    ySpan *= 1.15;

    axX_->setRange(cx - 0.5 * xSpan, cx + 0.5 * xSpan);
    axY_->setRange(cy - 0.5 * ySpan, cy + 0.5 * ySpan);
}
