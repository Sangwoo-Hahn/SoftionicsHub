#include "MainWindow.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSplitter>
#include <QGroupBox>
#include <QFileDialog>
#include <QFont>
#include <QPainter>
#include <algorithm>
#include <cmath>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    worker_ = new BleWorker();
    worker_->moveToThread(&workerThread_);
    workerThread_.start();

    connect(worker_, &BleWorker::scanUpdated, this, &MainWindow::onScanUpdated);
    connect(worker_, &BleWorker::statusText, this, &MainWindow::onStatus);
    connect(worker_, &BleWorker::connected, this, &MainWindow::onConnected);
    connect(worker_, &BleWorker::disconnected, this, &MainWindow::onDisconnected);
    connect(worker_, &BleWorker::frameReady, this, &MainWindow::onFrame);
    connect(worker_, &BleWorker::statsUpdated, this, &MainWindow::onStats);

    buildUi();

    plotTimer_ = new QTimer(this);
    plotTimer_->setInterval(50);
    connect(plotTimer_, &QTimer::timeout, this, &MainWindow::onPlotTick);
    plotTimer_->start();

    applyTimer_ = new QTimer(this);
    applyTimer_->setSingleShot(true);
    applyTimer_->setInterval(120);
    connect(applyTimer_, &QTimer::timeout, this, &MainWindow::applyPipelineNow);

    QMetaObject::invokeMethod(worker_, [w = worker_]() { w->startAuto("Softionics"); }, Qt::QueuedConnection);

    applyPipelineNow();
}

MainWindow::~MainWindow() {
    if (worker_) {
        QMetaObject::invokeMethod(worker_, "disconnectDevice", Qt::BlockingQueuedConnection);
    }
    workerThread_.quit();
    workerThread_.wait();
    delete worker_;
}

void MainWindow::buildUi() {
    auto* central = new QWidget(this);
    auto* root = new QHBoxLayout(central);

    auto* split = new QSplitter(Qt::Horizontal, central);

    auto* devPanel = new QWidget(split);
    auto* devL = new QVBoxLayout(devPanel);

    auto* devTitle = new QLabel("Devices (double-click to connect)", devPanel);
    QFont titleFont = devTitle->font();
    titleFont.setPointSize(titleFont.pointSize() + 1);
    titleFont.setBold(true);
    devTitle->setFont(titleFont);

    list_ = new QListWidget(devPanel);
    connect(list_, &QListWidget::itemDoubleClicked, this, &MainWindow::onDeviceDoubleClicked);

    status_ = new QLabel("Scanning...", devPanel);
    status_->setObjectName("StatusLabel");
    conn_ = new QLabel("-", devPanel);
    conn_->setObjectName("StatusLabel");
    stats_ = new QLabel("-", devPanel);
    stats_->setObjectName("StatusLabel");

    devL->addWidget(devTitle);
    devL->addWidget(list_, 1);
    devL->addWidget(status_);
    devL->addWidget(conn_);
    devL->addWidget(stats_);

    split->addWidget(devPanel);

    auto* chartPanel = new QWidget(split);
    auto* chartL = new QVBoxLayout(chartPanel);

    chart_ = new QChart();
    chart_->legend()->hide();

    axX_ = new QValueAxis();
    axY_ = new QValueAxis();
    axX_->setRange(0.0, 5.0);
    axY_->setRange(-1.0, 1.0);

    chart_->addAxis(axX_, Qt::AlignBottom);
    chart_->addAxis(axY_, Qt::AlignLeft);

    chartView_ = new QChartView(chart_, chartPanel);
    chartView_->setRenderHint(QPainter::Antialiasing, true);
    chartL->addWidget(chartView_, 1);

    split->addWidget(chartPanel);

    auto* ctrlPanel = new QWidget(split);
    auto* ctrlL = new QVBoxLayout(ctrlPanel);

    auto* gPlot = new QGroupBox("Plot", ctrlPanel);
    auto* pL = new QVBoxLayout(gPlot);

    auto* rowX = new QWidget(gPlot);
    auto* rowXL = new QHBoxLayout(rowX);
    rowXL->addWidget(new QLabel("X window (s)", rowX));
    sp_xwin_ = new QDoubleSpinBox(rowX);
    sp_xwin_->setRange(0.5, 120.0);
    sp_xwin_->setDecimals(2);
    sp_xwin_->setValue(5.0);
    rowXL->addWidget(sp_xwin_);
    rowXL->addStretch(1);
    pL->addWidget(rowX);

    auto* rowYc = new QWidget(gPlot);
    auto* rowYcL = new QHBoxLayout(rowYc);
    rowYcL->addWidget(new QLabel("Y center", rowYc));
    sp_ycenter_ = new QDoubleSpinBox(rowYc);
    sp_ycenter_->setRange(-1e9, 1e9);
    sp_ycenter_->setDecimals(6);
    sp_ycenter_->setValue(0.0);
    rowYcL->addWidget(sp_ycenter_);
    rowYcL->addStretch(1);
    pL->addWidget(rowYc);

    auto* rowYa = new QWidget(gPlot);
    auto* rowYaL = new QHBoxLayout(rowYa);
    cb_yauto_ = new QCheckBox("Auto Y", rowYa);
    cb_yauto_->setChecked(true);
    rowYaL->addWidget(cb_yauto_);
    rowYaL->addWidget(new QLabel("Y abs", rowYa));
    sp_yabs_ = new QDoubleSpinBox(rowYa);
    sp_yabs_->setRange(1e-12, 1e9);
    sp_yabs_->setDecimals(6);
    sp_yabs_->setValue(1.0);
    sp_yabs_->setEnabled(false);
    rowYaL->addWidget(sp_yabs_);
    rowYaL->addStretch(1);
    pL->addWidget(rowYa);

    connect(cb_yauto_, &QCheckBox::toggled, this, [this](bool on) {
        if (sp_yabs_) sp_yabs_->setEnabled(!on);
    });

    ctrlL->addWidget(gPlot);

    auto* gFilters = new QGroupBox("Filters", ctrlPanel);
    auto* fL = new QVBoxLayout(gFilters);

    auto* rowMA = new QWidget(gFilters);
    auto* rowMAL = new QHBoxLayout(rowMA);
    cb_ma_ = new QCheckBox("MA", rowMA);
    sp_ma_ = new QSpinBox(rowMA);
    sp_ma_->setRange(1, 20000);
    sp_ma_->setValue(5);
    rowMAL->addWidget(cb_ma_);
    rowMAL->addWidget(sp_ma_);
    rowMAL->addStretch(1);
    fL->addWidget(rowMA);

    auto* rowEMA = new QWidget(gFilters);
    auto* rowEMAL = new QHBoxLayout(rowEMA);
    cb_ema_ = new QCheckBox("EMA", rowEMA);
    cb_ema_->setChecked(true);
    sp_alpha_ = new QDoubleSpinBox(rowEMA);
    sp_alpha_->setRange(0.0, 1.0);
    sp_alpha_->setDecimals(4);
    sp_alpha_->setValue(0.2);
    rowEMAL->addWidget(cb_ema_);
    rowEMAL->addWidget(sp_alpha_);
    rowEMAL->addStretch(1);
    fL->addWidget(rowEMA);

    auto* rowNotch = new QWidget(gFilters);
    auto* rowNotchL = new QHBoxLayout(rowNotch);
    cb_notch_ = new QCheckBox("Notch", rowNotch);
    sp_fs_ = new QDoubleSpinBox(rowNotch);
    sp_fs_->setRange(10, 50000);
    sp_fs_->setDecimals(2);
    sp_fs_->setValue(200.0);
    sp_f0_ = new QDoubleSpinBox(rowNotch);
    sp_f0_->setRange(1, 5000);
    sp_f0_->setDecimals(2);
    sp_f0_->setValue(60.0);
    sp_q_ = new QDoubleSpinBox(rowNotch);
    sp_q_->setRange(0.1, 2000);
    sp_q_->setDecimals(2);
    sp_q_->setValue(30.0);
    rowNotchL->addWidget(cb_notch_);
    rowNotchL->addWidget(sp_fs_);
    rowNotchL->addWidget(sp_f0_);
    rowNotchL->addWidget(sp_q_);
    rowNotchL->addStretch(1);
    fL->addWidget(rowNotch);

    ctrlL->addWidget(gFilters);

    auto* gBias = new QGroupBox("Bias", ctrlPanel);
    auto* bL = new QVBoxLayout(gBias);

    auto* rowBias = new QWidget(gBias);
    auto* rowBiasL = new QHBoxLayout(rowBias);
    cb_bias_ = new QCheckBox("Apply", rowBias);
    sp_bias_frames_ = new QSpinBox(rowBias);
    sp_bias_frames_->setRange(1, 2000000);
    sp_bias_frames_->setValue(200);
    btn_bias_cap_ = new QPushButton("Capture", rowBias);
    connect(btn_bias_cap_, &QPushButton::clicked, this, &MainWindow::onBiasCapture);
    rowBiasL->addWidget(cb_bias_);
    rowBiasL->addWidget(sp_bias_frames_);
    rowBiasL->addWidget(btn_bias_cap_);
    rowBiasL->addStretch(1);
    bL->addWidget(rowBias);

    ctrlL->addWidget(gBias);

    auto* gModel = new QGroupBox("Model", ctrlPanel);
    auto* mL = new QVBoxLayout(gModel);

    auto* rowModel = new QWidget(gModel);
    auto* rowModelL = new QHBoxLayout(rowModel);
    cb_model_ = new QCheckBox("Enable", rowModel);
    cb_model_->setChecked(true);
    sp_model_bias_ = new QDoubleSpinBox(rowModel);
    sp_model_bias_->setRange(-1e9, 1e9);
    sp_model_bias_->setDecimals(6);
    sp_model_bias_->setValue(0.0);
    btn_load_weights_ = new QPushButton("Weights", rowModel);
    connect(btn_load_weights_, &QPushButton::clicked, this, &MainWindow::onLoadWeights);
    rowModelL->addWidget(cb_model_);
    rowModelL->addWidget(sp_model_bias_);
    rowModelL->addWidget(btn_load_weights_);
    rowModelL->addStretch(1);
    mL->addWidget(rowModel);

    modelOut_ = new QLabel("y = -", gModel);
    mL->addWidget(modelOut_);

    ctrlL->addWidget(gModel);

    auto* gRec = new QGroupBox("Record (CSV)", ctrlPanel);
    auto* rL = new QVBoxLayout(gRec);

    auto* rowRec = new QWidget(gRec);
    auto* rowRecL = new QHBoxLayout(rowRec);
    cb_record_ = new QCheckBox("Record", rowRec);
    ed_csv_path_ = new QLineEdit(rowRec);
    ed_csv_path_->setReadOnly(true);
    btn_browse_csv_ = new QPushButton("Browse", rowRec);
    connect(btn_browse_csv_, &QPushButton::clicked, this, &MainWindow::onBrowseCsv);
    connect(cb_record_, &QCheckBox::toggled, this, &MainWindow::onToggleRecord);
    rowRecL->addWidget(cb_record_);
    rowRecL->addWidget(ed_csv_path_, 1);
    rowRecL->addWidget(btn_browse_csv_);
    rL->addWidget(rowRec);

    ctrlL->addWidget(gRec);

    ctrlL->addStretch(1);

    auto applyHook = [this]() { onAnyControlChanged(); };
    connect(cb_ma_, &QCheckBox::toggled, this, applyHook);
    connect(sp_ma_, &QSpinBox::valueChanged, this, applyHook);
    connect(cb_ema_, &QCheckBox::toggled, this, applyHook);
    connect(sp_alpha_, &QDoubleSpinBox::valueChanged, this, applyHook);
    connect(cb_notch_, &QCheckBox::toggled, this, applyHook);
    connect(sp_fs_, &QDoubleSpinBox::valueChanged, this, applyHook);
    connect(sp_f0_, &QDoubleSpinBox::valueChanged, this, applyHook);
    connect(sp_q_, &QDoubleSpinBox::valueChanged, this, applyHook);
    connect(cb_bias_, &QCheckBox::toggled, this, applyHook);
    connect(cb_model_, &QCheckBox::toggled, this, applyHook);
    connect(sp_model_bias_, &QDoubleSpinBox::valueChanged, this, applyHook);

    split->addWidget(ctrlPanel);

    split->setStretchFactor(0, 1);
    split->setStretchFactor(1, 3);
    split->setStretchFactor(2, 1);

    root->addWidget(split);
    setCentralWidget(central);

    setWindowTitle("SoftionicsHub");
    resize(1500, 850);
}

hub::PipelineConfig MainWindow::readCfgFromUi() const {
    hub::PipelineConfig cfg;
    cfg.enable_ma = cb_ma_->isChecked();
    cfg.ma_win = (size_t)sp_ma_->value();
    cfg.enable_ema = cb_ema_->isChecked();
    cfg.ema_alpha = (float)sp_alpha_->value();
    cfg.enable_notch = cb_notch_->isChecked();
    cfg.fs_hz = sp_fs_->value();
    cfg.notch_f0 = sp_f0_->value();
    cfg.notch_q = sp_q_->value();
    cfg.enable_bias = cb_bias_->isChecked();
    cfg.enable_model = cb_model_->isChecked();
    cfg.model_bias = (float)sp_model_bias_->value();
    return cfg;
}

void MainWindow::onAnyControlChanged() {
    if (applyTimer_) applyTimer_->start();
}

void MainWindow::applyPipelineNow() {
    auto cfg = readCfgFromUi();
    QMetaObject::invokeMethod(worker_, [w = worker_, cfg]() { w->setPipelineConfig(cfg); }, Qt::QueuedConnection);
}

void MainWindow::onBiasCapture() {
    int frames = sp_bias_frames_->value();
    QMetaObject::invokeMethod(worker_, [w = worker_, frames]() { w->startBiasCapture(frames); }, Qt::QueuedConnection);
}

void MainWindow::onBrowseCsv() {
    QString path = QFileDialog::getSaveFileName(this, "Save CSV", "", "CSV (*.csv)");
    if (path.isEmpty()) return;
    ed_csv_path_->setText(path);
}

void MainWindow::onToggleRecord(bool on) {
    if (on) {
        if (ed_csv_path_->text().isEmpty()) {
            cb_record_->setChecked(false);
            onBrowseCsv();
            if (ed_csv_path_->text().isEmpty()) return;
            cb_record_->setChecked(true);
            return;
        }
        QString path = ed_csv_path_->text();
        QMetaObject::invokeMethod(worker_, [w = worker_, path]() { w->startCsv(path); }, Qt::QueuedConnection);
    } else {
        QMetaObject::invokeMethod(worker_, [w = worker_]() { w->stopCsv(); }, Qt::QueuedConnection);
    }
}

void MainWindow::onLoadWeights() {
    QString path = QFileDialog::getOpenFileName(this, "Load Weights CSV", "", "CSV (*.csv)");
    if (path.isEmpty()) return;
    QMetaObject::invokeMethod(worker_, [w = worker_, path]() { w->loadWeights(path); }, Qt::QueuedConnection);
}

void MainWindow::onDeviceDoubleClicked(QListWidgetItem*) {
    int idx = list_->currentRow();
    if (idx < 0) return;
    QMetaObject::invokeMethod(worker_, [w = worker_, idx]() { w->connectToIndex(idx); }, Qt::QueuedConnection);
}

void MainWindow::onScanUpdated(QVector<DeviceInfo> devices) {
    devices_ = devices;
    list_->clear();
    for (int i = 0; i < devices_.size(); ++i) {
        auto& d = devices_[i];
        QString s = QString("%1  (%2)  rssi=%3").arg(d.name).arg(d.address).arg(d.rssi);
        auto* item = new QListWidgetItem(s);
        item->setData(Qt::UserRole, d.address);
        list_->addItem(item);
    }
    updateDeviceListDecor();
}

void MainWindow::updateDeviceListDecor() {
    for (int i = 0; i < list_->count(); ++i) {
        auto* it = list_->item(i);
        QString addr = it->data(Qt::UserRole).toString();
        QFont f = it->font();
        f.setBold(!connectedAddr_.isEmpty() && addr == connectedAddr_);
        it->setFont(f);
    }
}

void MainWindow::onStatus(QString text) { status_->setText(text); }

void MainWindow::onConnected(QString name, QString addr) {
    connectedAddr_ = addr;
    conn_->setText(QString("Connected: %1  %2").arg(name).arg(addr));
    updateDeviceListDecor();
}

void MainWindow::onDisconnected() {
    connectedAddr_.clear();
    conn_->setText("-");
    updateDeviceListDecor();
}

void MainWindow::onStats(qulonglong ok, qulonglong bad) {
    stats_->setText(QString("ok=%1 bad=%2").arg(ok).arg(bad));
}

void MainWindow::rebuildPlot(int n_ch) {
    for (auto* s : series_) {
        chart_->removeSeries(s);
        delete s;
    }
    series_.clear();
    buffers_.clear();

    for (int i = 0; i < n_ch; ++i) {
        auto* s = new QLineSeries(chart_);
        chart_->addSeries(s);
        s->attachAxis(axX_);
        s->attachAxis(axY_);
        auto pen = s->pen();
        pen.setWidthF(1.0);
        auto c = pen.color();
        c.setAlpha(140);
        pen.setColor(c);
        s->setPen(pen);
        series_.push_back(s);
        buffers_.push_back(QList<QPointF>());
    }
}

void MainWindow::onFrame(qulonglong t_ns, QVector<float> x, bool modelValid, float modelOut) {
    lastTsNs_ = t_ns;
    lastFrame_ = x;
    if (modelValid) modelOut_->setText(QString("y = %1").arg(modelOut, 0, 'g', 10));
}

void MainWindow::onPlotTick() {
    if (lastFrame_.isEmpty()) return;
    if (lastTsNs_ == 0) return;

    if (t0Ns_ == 0) t0Ns_ = lastTsNs_;
    double t = (double)(lastTsNs_ - t0Ns_) * 1e-9;

    if (series_.size() != lastFrame_.size()) rebuildPlot(lastFrame_.size());

    double xwin = sp_xwin_ ? sp_xwin_->value() : 5.0;
    if (xwin < 0.5) xwin = 0.5;

    double xMin = t - xwin;
    if (xMin < 0.0) xMin = 0.0;
    double xMax = xMin + xwin;

    axX_->setRange(xMin, xMax);

    for (int i = 0; i < lastFrame_.size(); ++i) {
        buffers_[i].append(QPointF(t, lastFrame_[i]));
        while (!buffers_[i].isEmpty() && buffers_[i].first().x() < xMin) buffers_[i].removeFirst();
    }

    double yCenter = sp_ycenter_ ? sp_ycenter_->value() : 0.0;
    bool yAuto = cb_yauto_ ? cb_yauto_->isChecked() : true;

    double yAbs = 1.0;
    if (yAuto) {
        double maxAbs = 0.0;
        for (int i = 0; i < buffers_.size(); ++i) {
            for (const auto& pt : buffers_[i]) {
                double a = std::abs(pt.y() - yCenter);
                if (a > maxAbs) maxAbs = a;
            }
        }
        if (maxAbs < 1e-12) maxAbs = 1.0;
        yAbs = maxAbs * 1.05;
    } else {
        double v = sp_yabs_ ? sp_yabs_->value() : 1.0;
        if (v < 1e-12) v = 1.0;
        yAbs = v;
    }

    axY_->setRange(yCenter - yAbs, yCenter + yAbs);

    for (int i = 0; i < series_.size(); ++i) {
        series_[i]->replace(buffers_[i]);
    }
}
