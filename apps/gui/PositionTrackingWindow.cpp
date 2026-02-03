#include "PositionTrackingWindow.h"
#include "BleWorker.h"
#include "PositionTrackingEngine.h"
#include "hub/model/BruteForce_16x2.h"
#include "FormatDoubleSpinBox.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSplitter>
#include <QGroupBox>
#include <QPainter>
#include <QRectF>
#include <QSizePolicy>
#include <QtCore/QOverload>
#include <algorithm>
#include <cmath>

PositionTrackingWindow::PositionTrackingWindow(BleWorker* worker, QWidget* parent)
    : QMainWindow(parent), worker_(worker) {

    engine_ = new PositionTrackingEngine();
    engine_->moveToThread(&engineThread_);
    engineThread_.start();

    connect(engine_, &PositionTrackingEngine::outputReady, this, &PositionTrackingWindow::onEngineOut, Qt::QueuedConnection);
    connect(engine_, &PositionTrackingEngine::statusReady, this, &PositionTrackingWindow::onEngineStatus, Qt::QueuedConnection);

    buildUi();

    paramTimer_ = new QTimer(this);
    paramTimer_->setSingleShot(true);
    paramTimer_->setInterval(80);
    connect(paramTimer_, &QTimer::timeout, this, &PositionTrackingWindow::onApplyParams);

    timer_ = new QTimer(this);
    timer_->setInterval(16);
    connect(timer_, &QTimer::timeout, this, &PositionTrackingWindow::onTick);
    timer_->start();

    auto algos = hub::pt::list_algorithms();
    for (auto& a : algos) cbAlgo_->addItem(QString::fromStdString(a.id));

    if (cbAlgo_->count() > 0) {
        cbAlgo_->setCurrentIndex(0);
        onAlgoChanged(0);
    }
}

PositionTrackingWindow::~PositionTrackingWindow() {
    if (connected_) {
        QObject::disconnect(worker_, &BleWorker::frameReady, engine_, &PositionTrackingEngine::onSample);
        connected_ = false;
    }
    engineThread_.quit();
    engineThread_.wait();
    delete engine_;
    engine_ = nullptr;
}

void PositionTrackingWindow::showEvent(QShowEvent* e) {
    QMainWindow::showEvent(e);
    if (!connected_) {
        QObject::connect(worker_, &BleWorker::frameReady, engine_, &PositionTrackingEngine::onSample, Qt::QueuedConnection);
        connected_ = true;
    }
    if (timer_ && !timer_->isActive()) timer_->start();
}

void PositionTrackingWindow::hideEvent(QHideEvent* e) {
    QMainWindow::hideEvent(e);
    if (connected_) {
        QObject::disconnect(worker_, &BleWorker::frameReady, engine_, &PositionTrackingEngine::onSample);
        connected_ = false;
    }
    if (timer_ && timer_->isActive()) timer_->stop();
}

void PositionTrackingWindow::closeEvent(QCloseEvent* e) {
    this->hide();
    e->ignore();
}

void PositionTrackingWindow::buildUi() {
    setWindowTitle("PositionTracking");
    resize(1350, 980);

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

    path_ = new QLineSeries(chart_);
    chart_->addSeries(path_);
    path_->attachAxis(axX_);
    path_->attachAxis(axY_);

    cur_ = new QScatterSeries(chart_);
    cur_->setMarkerSize(12.0);
    chart_->addSeries(cur_);
    cur_->attachAxis(axX_);
    cur_->attachAxis(axY_);

    view_ = new QChartView(chart_, plotW);
    view_->setRenderHint(QPainter::Antialiasing, true);
    plotL->addWidget(view_, 1);

    lbStats_ = new QLabel("waiting...", plotW);
    lbStats_->setObjectName("StatusLabel");
    lbStats_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    plotL->addWidget(lbStats_);

    connect(chart_, &QChart::plotAreaChanged, this, [this](const QRectF&) { updateAxesAndDraw(); });

    split->addWidget(plotW);

    auto* ctrlW = new QWidget(split);
    ctrlW->setMinimumWidth(520);
    auto* ctrlL = new QVBoxLayout(ctrlW);

    auto* gSel = new QGroupBox("Model", ctrlW);
    auto* selL = new QVBoxLayout(gSel);

    cbAlgo_ = new QComboBox(gSel);
    lbAlgoInfo_ = new QLabel("-", gSel);
    lbAlgoInfo_->setObjectName("StatusLabel");

    selL->addWidget(cbAlgo_);
    selL->addWidget(lbAlgoInfo_);
    ctrlL->addWidget(gSel, 0);

    auto* gParams = new QGroupBox("Params", ctrlW);
    gParams->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* gPL = new QVBoxLayout(gParams);

    auto* scroll = new QScrollArea(gParams);
    scroll->setWidgetResizable(true);
    scroll->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    scroll->setMinimumHeight(720);

    paramBox_ = new QWidget(scroll);
    paramForm_ = new QFormLayout(paramBox_);
    paramForm_->setVerticalSpacing(10);
    paramForm_->setHorizontalSpacing(12);
    paramForm_->setFormAlignment(Qt::AlignTop);
    paramForm_->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    paramBox_->setLayout(paramForm_);
    scroll->setWidget(paramBox_);

    gPL->addWidget(scroll, 1);

    btnApply_ = new QPushButton("Apply", gParams);
    gPL->addWidget(btnApply_, 0);

    ctrlL->addWidget(gParams, 1);

    auto* gTools = new QGroupBox("Tools", ctrlW);
    auto* tL = new QVBoxLayout(gTools);

    spPathLen_ = new QSpinBox(gTools);
    spPathLen_->setRange(1, 5000);
    spPathLen_->setValue(40);

    btnReset_ = new QPushButton("Reset", gTools);
    btnClear_ = new QPushButton("Clear Path", gTools);

    tL->addWidget(new QLabel("Path points", gTools));
    tL->addWidget(spPathLen_);
    tL->addWidget(btnReset_);
    tL->addWidget(btnClear_);

    ctrlL->addWidget(gTools, 0);
    ctrlL->addStretch(1);

    connect(cbAlgo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &PositionTrackingWindow::onAlgoChanged);
    connect(btnApply_, &QPushButton::clicked, this, &PositionTrackingWindow::onApplyParams);
    connect(btnReset_, &QPushButton::clicked, this, &PositionTrackingWindow::onResetAlgo);
    connect(btnClear_, &QPushButton::clicked, this, &PositionTrackingWindow::onClearPath);

    split->addWidget(ctrlW);

    split->setStretchFactor(0, 12);
    split->setStretchFactor(1, 6);

    root->addWidget(split);
    setCentralWidget(central);
}

void PositionTrackingWindow::onAlgoChanged(int idx) {
    if (idx < 0) return;

    std::string id = cbAlgo_->itemText(idx).toStdString();
    curInfo_ = hub::pt::get_algorithm_info(id);

    lbAlgoInfo_->setText(QString("id=%1  N=%2  M=%3")
        .arg(QString::fromStdString(curInfo_.id))
        .arg(curInfo_.N)
        .arg(curInfo_.M));

    rebuildParamUi(curInfo_);

    QMetaObject::invokeMethod(engine_, "setAlgorithm", Qt::QueuedConnection, Q_ARG(QString, QString::fromStdString(id)));
    QMetaObject::invokeMethod(engine_, "setParams", Qt::QueuedConnection, Q_ARG(QVector<double>, collectParams()));
    QMetaObject::invokeMethod(engine_, "reset", Qt::QueuedConnection);

    pending_.clear();
    pathBuf_.clear();
    last_ = {false, false, 0, 0, 0, 0, 0, 0, 0};
    engineStatusText_.clear();

    sensors_->clear();
    if (curInfo_.N == 16) {
        auto sens = hub::BruteForce_16x2Solver::sensor_positions();
        for (int i = 0; i < 16; ++i) sensors_->append(sens[i].x, sens[i].y);
    }

    updateAxesAndDraw();
}

void PositionTrackingWindow::rebuildParamUi(const hub::pt::AlgoInfo& info) {
    while (paramForm_->rowCount() > 0) paramForm_->removeRow(0);
    paramSpins_.clear();

    for (size_t i = 0; i < info.params.size(); ++i) {
        const auto& p = info.params[i];

        auto* sp = new FormatDoubleSpinBox(paramBox_);
        sp->blockSignals(true);
        sp->setRange(p.minv, p.maxv);
        double v0 = p.defv;
        if (i < info.defaults.size()) v0 = info.defaults[i];
        sp->setValue(v0);
        sp->setMinimumWidth(260);
        sp->setMinimumHeight(28);

        if (p.scientific) {
            sp->setMode(FormatDoubleSpinBox::Mode::Scientific);
            int internalDecimals = (p.decimals >= 0) ? p.decimals : 18;
            sp->setSciDigits(6, internalDecimals);
        } else {
            sp->setMode(FormatDoubleSpinBox::Mode::Fixed);
            int d = (p.decimals >= 0) ? p.decimals : 6;
            sp->setFixedDecimals(d);
            if (p.step > 0.0) sp->setSingleStep(p.step);
        }

        sp->blockSignals(false);

        connect(sp, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
            if (paramTimer_) paramTimer_->start();
        });

        paramForm_->addRow(QString::fromStdString(p.label), sp);
        paramSpins_.push_back(sp);
    }
}

QVector<double> PositionTrackingWindow::collectParams() const {
    QVector<double> v;
    v.reserve(paramSpins_.size());
    for (auto* sp : paramSpins_) v.push_back(sp->value());
    return v;
}

void PositionTrackingWindow::onApplyParams() {
    QMetaObject::invokeMethod(engine_, "setParams", Qt::QueuedConnection, Q_ARG(QVector<double>, collectParams()));
}

void PositionTrackingWindow::onResetAlgo() {
    QMetaObject::invokeMethod(engine_, "reset", Qt::QueuedConnection);
    pending_.clear();
    pathBuf_.clear();
    last_ = {false, false, 0, 0, 0, 0, 0, 0, 0};
    engineStatusText_.clear();
    updateAxesAndDraw();
}

void PositionTrackingWindow::onClearPath() {
    pathBuf_.clear();
    updateAxesAndDraw();
}

void PositionTrackingWindow::onEngineOut(double x, double y, double z, double confidence, double q1, double q2, double err, bool quiet, bool valid) {
    engineStatusText_.clear();
    pending_.push_back(OutPkt{valid, quiet, x, y, z, confidence, q1, q2, err});
}

void PositionTrackingWindow::onEngineStatus(QString text) {
    engineStatusText_ = text;
}

void PositionTrackingWindow::onTick() {
    if (!pending_.isEmpty()) {
        auto local = pending_;
        pending_.clear();

        for (const auto& p : local) {
            last_ = p;
            if (!p.valid) continue;

            if (p.quiet) {
                if (!pathBuf_.isEmpty()) pathBuf_.removeFirst();
            } else {
                pathBuf_.append(QPointF(p.x, p.y));
                while (pathBuf_.size() > spPathLen_->value()) pathBuf_.removeFirst();
            }
        }
    }
    updateAxesAndDraw();
}

void PositionTrackingWindow::updateAxesAndDraw() {
    if (last_.valid) {
        QVector<QPointF> cur;
        cur.push_back(QPointF(last_.x, last_.y));
        cur_->setMarkerSize(6.0 + 12.0 * std::clamp(last_.confidence, 0.0, 1.0));
        cur_->replace(cur);
    } else {
        cur_->setMarkerSize(12.0);
        cur_->clear();
    }
    path_->replace(pathBuf_);

    if (last_.valid) {
        lbStats_->setText(QString("x=%1  y=%2  z=%3  conf=%4  err=%5  %6")
            .arg(last_.x, 0, 'g', 6)
            .arg(last_.y, 0, 'g', 6)
            .arg(last_.z, 0, 'g', 6)
            .arg(last_.confidence, 0, 'g', 6)
            .arg(last_.err, 0, 'g', 6)
            .arg(last_.quiet ? "QUIET" : "ACTIVE"));
    } else if (!engineStatusText_.isEmpty()) {
        lbStats_->setText(engineStatusText_);
    } else {
        lbStats_->setText("waiting...");
    }

    double minx = -0.03, maxx = 0.03;
    double miny = -0.03, maxy = 0.03;

    if (curInfo_.N == 16) {
        auto sens = hub::BruteForce_16x2Solver::sensor_positions();
        minx = maxx = sens[0].x;
        miny = maxy = sens[0].y;
        for (int i = 1; i < 16; ++i) {
            minx = std::min(minx, sens[i].x);
            maxx = std::max(maxx, sens[i].x);
            miny = std::min(miny, sens[i].y);
            maxy = std::max(maxy, sens[i].y);
        }
    }

    for (const auto& p : pathBuf_) {
        minx = std::min(minx, p.x());
        maxx = std::max(maxx, p.x());
        miny = std::min(miny, p.y());
        maxy = std::max(maxy, p.y());
    }

    if (last_.valid) {
        minx = std::min(minx, last_.x);
        maxx = std::max(maxx, last_.x);
        miny = std::min(miny, last_.y);
        maxy = std::max(maxy, last_.y);
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
