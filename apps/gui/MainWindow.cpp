#include "MainWindow.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSplitter>
#include <QGroupBox>
#include <QFileDialog>
#include <QFont>
#include <QPainter>
#include <QAbstractItemView>
#include <cmath>
#include "hub/model/BF16.h"

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
    connect(worker_, &BleWorker::biasStateChanged, this, &MainWindow::onBiasState);
    connect(worker_, &BleWorker::streamStats, this, &MainWindow::onStreamStats);
    connect(worker_, &BleWorker::poseReady, this, &MainWindow::onPoseReady);

    buildUi();

    plotTimer_ = new QTimer(this);
    plotTimer_->setInterval(100);
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
    split->setChildrenCollapsible(false);

    auto* devPanel = new QWidget(split);
    devPanel->setMinimumWidth(280);
    auto* devL = new QVBoxLayout(devPanel);

    auto* devTitle = new QLabel("Devices (click to connect)", devPanel);
    QFont titleFont = devTitle->font();
    titleFont.setBold(true);
    devTitle->setFont(titleFont);

    list_ = new QListWidget(devPanel);
    list_->setSelectionMode(QAbstractItemView::NoSelection);
    list_->setFocusPolicy(Qt::NoFocus);
    list_->setMouseTracking(true);
    connect(list_, &QListWidget::itemClicked, this, &MainWindow::onDeviceClicked);

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
    chartPanel->setMinimumWidth(900);
    auto* chartL = new QVBoxLayout(chartPanel);

    timeChart_ = new QChart();
    timeChart_->legend()->hide();

    timeAxX_ = new QValueAxis();
    timeAxY_ = new QValueAxis();

    timeAxX_->setRange(0.0, 1.0);
    timeAxY_->setRange(-1.0, 1.0);

    timeAxX_->setLabelFormat("%.3f");
    timeAxY_->setLabelFormat("%.6g");

    timeChart_->addAxis(timeAxX_, Qt::AlignBottom);
    timeChart_->addAxis(timeAxY_, Qt::AlignLeft);

    timeCenterLine_ = new QLineSeries(timeChart_);
    timeChart_->addSeries(timeCenterLine_);
    timeCenterLine_->attachAxis(timeAxX_);
    timeCenterLine_->attachAxis(timeAxY_);
    auto pen = timeCenterLine_->pen();
    pen.setWidthF(1.0);
    pen.setStyle(Qt::DashLine);
    auto c = pen.color();
    c.setAlpha(140);
    pen.setColor(c);
    timeCenterLine_->setPen(pen);

    timeView_ = new QChartView(timeChart_, chartPanel);
    timeView_->setRenderHint(QPainter::Antialiasing, true);
    chartL->addWidget(timeView_, 1);

    lb_stream_stats_ = new QLabel("Total: 0 | Time: 0.000 s | 1s: 0 | dt: 0.000 ms", chartPanel);
    lb_stream_stats_->setObjectName("StatusLabel");
    lb_stream_stats_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    chartL->addWidget(lb_stream_stats_);

    lb_pose_stats_ = new QLabel("Pose: N/A (need 16ch)", chartPanel);
    lb_pose_stats_->setObjectName("StatusLabel");
    lb_pose_stats_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    chartL->addWidget(lb_pose_stats_);

    xyChart_ = new QChart();
    xyChart_->legend()->hide();

    xyAxX_ = new QValueAxis();
    xyAxY_ = new QValueAxis();
    xyAxX_->setRange(-0.08, 0.08);
    xyAxY_->setRange(-0.08, 0.08);
    xyAxX_->setLabelFormat("%.3f");
    xyAxY_->setLabelFormat("%.3f");

    xyChart_->addAxis(xyAxX_, Qt::AlignBottom);
    xyChart_->addAxis(xyAxY_, Qt::AlignLeft);

    xySensors_ = new QScatterSeries(xyChart_);
    xySensors_->setMarkerSize(6.0);
    xyChart_->addSeries(xySensors_);
    xySensors_->attachAxis(xyAxX_);
    xySensors_->attachAxis(xyAxY_);

    auto sens = hub::BF16Solver::sensor_positions();
    for (int i = 0; i < 16; ++i) {
        xySensors_->append(sens[i].x, sens[i].y);
    }

    xyPath_ = new QLineSeries(xyChart_);
    xyChart_->addSeries(xyPath_);
    xyPath_->attachAxis(xyAxX_);
    xyPath_->attachAxis(xyAxY_);
    auto ppen = xyPath_->pen();
    ppen.setWidthF(2.0);
    xyPath_->setPen(ppen);

    xyCurrent_ = new QScatterSeries(xyChart_);
    xyCurrent_->setMarkerSize(10.0);
    xyChart_->addSeries(xyCurrent_);
    xyCurrent_->attachAxis(xyAxX_);
    xyCurrent_->attachAxis(xyAxY_);

    xyView_ = new QChartView(xyChart_, chartPanel);
    xyView_->setRenderHint(QPainter::Antialiasing, true);
    xyView_->setMinimumHeight(260);
    xyView_->setMaximumHeight(260);
    chartL->addWidget(xyView_);

    split->addWidget(chartPanel);

    auto* ctrlPanel = new QWidget(split);
    ctrlPanel->setMinimumWidth(420);
    auto* ctrlL = new QVBoxLayout(ctrlPanel);

    auto* gPlot = new QGroupBox("Plot", ctrlPanel);
    auto* pL = new QVBoxLayout(gPlot);

    auto* rowX = new QWidget(gPlot);
    auto* rowXL = new QHBoxLayout(rowX);
    rowXL->addWidget(new QLabel("X window (s)", rowX));
    sp_xwin_ = new QDoubleSpinBox(rowX);
    sp_xwin_->setRange(0.5, 120.0);
    sp_xwin_->setDecimals(2);
    sp_xwin_->setValue(1.0);
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
    cb_ma_->setChecked(false);
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
    cb_ema_->setChecked(false);
    sp_alpha_ = new QDoubleSpinBox(rowEMA);
    sp_alpha_->setRange(0.0, 1.0);
    sp_alpha_->setDecimals(4);
    sp_alpha_->setValue(0.2);
    rowEMAL->addWidget(cb_ema_);
    rowEMAL->addWidget(sp_alpha_);
    rowEMAL->addStretch(1);
    fL->addWidget(rowEMA);

    auto* rowNotchCheck = new QWidget(gFilters);
    auto* rowNotchCheckL = new QHBoxLayout(rowNotchCheck);
    cb_notch_ = new QCheckBox("Notch", rowNotchCheck);
    cb_notch_->setChecked(false);
    rowNotchCheckL->addWidget(cb_notch_);
    rowNotchCheckL->addStretch(1);
    fL->addWidget(rowNotchCheck);

    auto* rowNotchParams = new QWidget(gFilters);
    auto* rowNotchParamsL = new QHBoxLayout(rowNotchParams);

    rowNotchParamsL->addWidget(new QLabel("Sampling rate (Hz)", rowNotchParams));
    sp_fs_ = new QDoubleSpinBox(rowNotchParams);
    sp_fs_->setRange(10, 50000);
    sp_fs_->setDecimals(2);
    sp_fs_->setValue(200.0);
    rowNotchParamsL->addWidget(sp_fs_);

    rowNotchParamsL->addWidget(new QLabel("Notch f0 (Hz)", rowNotchParams));
    sp_f0_ = new QDoubleSpinBox(rowNotchParams);
    sp_f0_->setRange(1, 5000);
    sp_f0_->setDecimals(2);
    sp_f0_->setValue(60.0);
    rowNotchParamsL->addWidget(sp_f0_);

    rowNotchParamsL->addWidget(new QLabel("Q", rowNotchParams));
    sp_q_ = new QDoubleSpinBox(rowNotchParams);
    sp_q_->setRange(0.1, 2000);
    sp_q_->setDecimals(2);
    sp_q_->setValue(30.0);
    rowNotchParamsL->addWidget(sp_q_);

    rowNotchParamsL->addStretch(1);
    fL->addWidget(rowNotchParams);

    ctrlL->addWidget(gFilters);

    auto* gBias = new QGroupBox("Bias", ctrlPanel);
    auto* bL = new QVBoxLayout(gBias);

    auto* rowBias = new QWidget(gBias);
    auto* rowBiasL = new QHBoxLayout(rowBias);
    cb_bias_apply_ = new QCheckBox("Apply stored bias", rowBias);
    cb_bias_apply_->setChecked(false);
    sp_bias_frames_ = new QSpinBox(rowBias);
    sp_bias_frames_->setRange(1, 2000000);
    sp_bias_frames_->setValue(200);
    btn_bias_cap_ = new QPushButton("Capture", rowBias);
    connect(btn_bias_cap_, &QPushButton::clicked, this, &MainWindow::onBiasCapture);
    btn_bias_save_ = new QPushButton("Save CSV", rowBias);
    connect(btn_bias_save_, &QPushButton::clicked, this, &MainWindow::onBiasSave);

    rowBiasL->addWidget(cb_bias_apply_);
    rowBiasL->addWidget(sp_bias_frames_);
    rowBiasL->addWidget(btn_bias_cap_);
    rowBiasL->addWidget(btn_bias_save_);
    rowBiasL->addStretch(1);
    bL->addWidget(rowBias);

    lb_bias_state_ = new QLabel("State: None", gBias);
    bL->addWidget(lb_bias_state_);

    ctrlL->addWidget(gBias);

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

    connect(sp_xwin_, &QDoubleSpinBox::valueChanged, this, [this]() { clearTimePlotData(); });
    connect(sp_ycenter_, &QDoubleSpinBox::valueChanged, this, applyHook);
    connect(sp_yabs_, &QDoubleSpinBox::valueChanged, this, applyHook);
    connect(cb_yauto_, &QCheckBox::toggled, this, applyHook);

    connect(cb_ma_, &QCheckBox::toggled, this, applyHook);
    connect(sp_ma_, &QSpinBox::valueChanged, this, applyHook);
    connect(cb_ema_, &QCheckBox::toggled, this, applyHook);
    connect(sp_alpha_, &QDoubleSpinBox::valueChanged, this, applyHook);

    connect(cb_notch_, &QCheckBox::toggled, this, applyHook);
    connect(sp_fs_, &QDoubleSpinBox::valueChanged, this, [this]() {
        clearTimePlotData();
        onAnyControlChanged();
    });
    connect(sp_f0_, &QDoubleSpinBox::valueChanged, this, applyHook);
    connect(sp_q_, &QDoubleSpinBox::valueChanged, this, applyHook);

    connect(cb_bias_apply_, &QCheckBox::toggled, this, applyHook);

    split->addWidget(ctrlPanel);

    split->setStretchFactor(0, 2);
    split->setStretchFactor(1, 14);
    split->setStretchFactor(2, 3);

    root->addWidget(split);
    setCentralWidget(central);

    setWindowTitle("SoftionicsHub");
    resize(1750, 1050);
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
    cfg.enable_bias = cb_bias_apply_->isChecked();
    cfg.enable_model = false;
    cfg.model_bias = 0.0f;
    return cfg;
}

void MainWindow::clearTimePlotData() {
    pending_samples_.clear();
    plotSampleIndex_ = 0;
    plotFsUsed_ = 0.0;

    for (auto& b : timeBuffers_) b.clear();
    for (auto* s : timeSeries_) s->replace(QList<QPointF>());

    if (timeCenterLine_) timeCenterLine_->replace(QList<QPointF>());

    double xwin = sp_xwin_ ? sp_xwin_->value() : 1.0;
    if (xwin < 0.5) xwin = 0.5;
    timeAxX_->setRange(0.0, xwin);
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

void MainWindow::onBiasSave() {
    QString path = QFileDialog::getSaveFileName(this, "Save Bias CSV", "", "CSV (*.csv)");
    if (path.isEmpty()) return;
    QMetaObject::invokeMethod(worker_, [w = worker_, path]() { w->saveBiasCsv(path); }, Qt::QueuedConnection);
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

void MainWindow::onDeviceClicked(QListWidgetItem* item) {
    if (!item) return;
    int idx = list_->row(item);
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

    fsAutoSetDone_ = false;
    clearTimePlotData();

    pending_pose_.clear();
    xyPathBuf_.clear();
    xyPath_->replace(QList<QPointF>());
    xyCurrent_->replace(QList<QPointF>());
    lastPose_ = {false,false,0,0,0,0,0,0};
    lb_pose_stats_->setText("Pose: waiting (need 2 frames)");
}

void MainWindow::onDisconnected() {
    connectedAddr_.clear();
    conn_->setText("-");
    updateDeviceListDecor();

    fsAutoSetDone_ = false;
    clearTimePlotData();

    pending_pose_.clear();
    xyPathBuf_.clear();
    xyPath_->replace(QList<QPointF>());
    xyCurrent_->replace(QList<QPointF>());
    lastPose_ = {false,false,0,0,0,0,0,0};
    lb_pose_stats_->setText("Pose: N/A (need 16ch)");
}

void MainWindow::onStats(qulonglong ok, qulonglong bad) {
    stats_->setText(QString("ok=%1 bad=%2").arg(ok).arg(bad));
}

void MainWindow::onBiasState(bool hasBias, bool capturing) {
    if (!lb_bias_state_) return;
    if (capturing) lb_bias_state_->setText("State: Capturing");
    else if (hasBias) lb_bias_state_->setText("State: Stored");
    else lb_bias_state_->setText("State: None");
}

void MainWindow::onStreamStats(qulonglong totalSamples, double totalTimeSec, qulonglong last1sSamples, double lastDtSec) {
    if (lb_stream_stats_) {
        double dt_ms = lastDtSec * 1000.0;
        QString s = QString("Total: %1 | Time: %2 s | 1s: %3 | dt: %4 ms")
            .arg(totalSamples)
            .arg(totalTimeSec, 0, 'f', 3)
            .arg(last1sSamples)
            .arg(dt_ms, 0, 'f', 3);
        lb_stream_stats_->setText(s);
    }

    if (!fsAutoSetDone_ && totalTimeSec >= 1.0 && last1sSamples > 0) {
        fsAutoSetDone_ = true;
        if (sp_fs_) sp_fs_->setValue((double)last1sSamples);
        status_->setText(QString("Sampling rate auto-set: %1 Hz").arg(last1sSamples));
    }
}

void MainWindow::rebuildTimePlot(int n_ch) {
    for (auto* s : timeSeries_) {
        timeChart_->removeSeries(s);
        delete s;
    }
    timeSeries_.clear();
    timeBuffers_.clear();

    for (int i = 0; i < n_ch; ++i) {
        auto* s = new QLineSeries(timeChart_);
        timeChart_->addSeries(s);
        s->attachAxis(timeAxX_);
        s->attachAxis(timeAxY_);
        auto pen = s->pen();
        pen.setWidthF(1.0);
        auto c = pen.color();
        c.setAlpha(120);
        pen.setColor(c);
        s->setPen(pen);
        timeSeries_.push_back(s);
        timeBuffers_.push_back(QList<QPointF>());
    }
}

void MainWindow::onFrame(qulonglong, QVector<float> x, bool, float) {
    pending_samples_.emplace_back(std::move(x));
}

void MainWindow::onPoseReady(double x, double y, double z, double q1, double q2, double err, bool quiet, bool hasPose) {
    pending_pose_.push_back(PosePkt{hasPose, quiet, x, y, z, q1, q2, err});
}

void MainWindow::updateXYPlot() {
    if (!lastPose_.hasPose) return;

    QList<QPointF> cur;
    cur.append(QPointF(lastPose_.x, lastPose_.y));
    xyCurrent_->replace(cur);
    xyPath_->replace(xyPathBuf_);

    double xmm = lastPose_.x * 1000.0;
    double ymm = lastPose_.y * 1000.0;
    double zmm = lastPose_.z * 1000.0;

    QString s = QString("Pose: x=%1 mm, y=%2 mm, z=%3 mm | q1=%4 q2=%5 | err=%6 | %7")
        .arg(xmm, 0, 'f', 1)
        .arg(ymm, 0, 'f', 1)
        .arg(zmm, 0, 'f', 1)
        .arg(lastPose_.q1, 0, 'g', 6)
        .arg(lastPose_.q2, 0, 'g', 6)
        .arg(lastPose_.err, 0, 'g', 6)
        .arg(lastPose_.quiet ? "QUIET" : "ACTIVE");

    lb_pose_stats_->setText(s);
}

void MainWindow::onPlotTick() {
    if (!pending_samples_.empty()) {
        std::vector<QVector<float>> local;
        local.swap(pending_samples_);

        double fs = sp_fs_ ? sp_fs_->value() : 200.0;
        if (fs < 1.0) fs = 1.0;

        if (plotFsUsed_ == 0.0) plotFsUsed_ = fs;
        if (std::abs(fs - plotFsUsed_) > 1e-9) {
            plotFsUsed_ = fs;
            plotSampleIndex_ = 0;
            for (auto& b : timeBuffers_) b.clear();
            for (auto* s : timeSeries_) s->replace(QList<QPointF>());
        }

        const double dt = 1.0 / plotFsUsed_;

        int n_ch = (int)local.front().size();
        if (n_ch > 0 && timeSeries_.size() != n_ch) rebuildTimePlot(n_ch);

        double t_end = 0.0;
        for (const auto& sample : local) {
            if ((int)sample.size() != n_ch) continue;
            double t = (double)plotSampleIndex_ * dt;
            plotSampleIndex_++;
            t_end = t;
            for (int i = 0; i < n_ch; ++i) {
                timeBuffers_[i].append(QPointF(t, sample[i]));
            }
        }

        double xwin = sp_xwin_ ? sp_xwin_->value() : 1.0;
        if (xwin < 0.5) xwin = 0.5;

        double xMin = t_end - xwin;
        if (xMin < 0.0) xMin = 0.0;
        double xMax = xMin + xwin;

        timeAxX_->setRange(xMin, xMax);

        for (int i = 0; i < n_ch; ++i) {
            while (!timeBuffers_[i].isEmpty() && timeBuffers_[i].first().x() < xMin) timeBuffers_[i].removeFirst();
        }

        double yCenter = sp_ycenter_ ? sp_ycenter_->value() : 0.0;
        bool yAuto = cb_yauto_ ? cb_yauto_->isChecked() : true;

        double yAbs = 1.0;
        if (yAuto) {
            double maxAbs = 0.0;
            for (int i = 0; i < n_ch; ++i) {
                for (const auto& pt : timeBuffers_[i]) {
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
        timeAxY_->setRange(yCenter - yAbs, yCenter + yAbs);

        if (timeCenterLine_) {
            QList<QPointF> pts;
            pts.append(QPointF(xMin, yCenter));
            pts.append(QPointF(xMax, yCenter));
            timeCenterLine_->replace(pts);
        }

        for (int i = 0; i < n_ch; ++i) {
            timeSeries_[i]->replace(timeBuffers_[i]);
        }
    }

    if (!pending_pose_.empty()) {
        std::vector<PosePkt> localPose;
        localPose.swap(pending_pose_);

        for (const auto& p : localPose) {
            if (!p.hasPose) continue;

            lastPose_ = p;

            if (p.quiet) {
                if (!xyPathBuf_.isEmpty()) xyPathBuf_.removeFirst();
            } else {
                xyPathBuf_.append(QPointF(p.x, p.y));
                while (xyPathBuf_.size() > xyMaxPoints_) xyPathBuf_.removeFirst();
            }
        }

        updateXYPlot();
    }
}
