#include "MainWindow.h"
#include "PositionTrackingWindow.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSplitter>
#include <QGroupBox>
#include <QFileDialog>
#include <QFont>
#include <QPainter>
#include <QAbstractItemView>
#include <QtCore/QOverload>
#include <QStyledItemDelegate>
#include <QStyleOptionViewItem>
#include <QColor>
#include <QBrush>

#include <algorithm>
#include <cmath>

static constexpr int ROLE_ADDR       = Qt::UserRole;
static constexpr int ROLE_STATE      = Qt::UserRole + 1;   // 0 normal, 1 connected, 2 connecting
static constexpr int ROLE_SCAN_INDEX = Qt::UserRole + 2;   // -1 pinned
static constexpr int ROLE_NAME       = Qt::UserRole + 3;

class DeviceItemDelegate : public QStyledItemDelegate {
public:
    using QStyledItemDelegate::QStyledItemDelegate;

    void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override {
        QStyleOptionViewItem opt = option;
        initStyleOption(&opt, index);
        QStyledItemDelegate::paint(painter, opt, index);

        int st = index.data(ROLE_STATE).toInt();
        if (st == 1) {
            painter->save();
            QPen pen(QColor(0, 120, 215));
            pen.setWidth(2);
            painter->setPen(pen);
            QRect r = option.rect.adjusted(2, 2, -2, -2);
            painter->drawRoundedRect(r, 6, 6);
            painter->restore();
        }
    }
};

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

    buildUi();

    plotTimer_ = new QTimer(this);
    plotTimer_->setInterval(33); // 30fps redraw
    connect(plotTimer_, &QTimer::timeout, this, &MainWindow::onPlotTick);
    plotTimer_->start();

    applyTimer_ = new QTimer(this);
    applyTimer_->setSingleShot(true);
    applyTimer_->setInterval(120);
    connect(applyTimer_, &QTimer::timeout, this, &MainWindow::applyPipelineNow);

    connectTimeout_ = new QTimer(this);
    connectTimeout_->setSingleShot(true);
    connectTimeout_->setInterval(8000);
    connect(connectTimeout_, &QTimer::timeout, this, [this]() {
        if (!connecting_) return;
        status_->setText("Connect timeout");
        endConnecting();
    });

    QMetaObject::invokeMethod(worker_, [w = worker_]() { w->startAuto("Softionics"); }, Qt::QueuedConnection);
    applyPipelineNow();

    clearPlotData();
}

MainWindow::~MainWindow() {
    if (ptWin_) {
        ptWin_->close();
        ptWin_ = nullptr;
    }

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

    // Left: Devices
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
    list_->setItemDelegate(new DeviceItemDelegate(list_));
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

    // Middle: Chart
    auto* chartPanel = new QWidget(split);
    chartPanel->setMinimumWidth(900);
    auto* chartL = new QVBoxLayout(chartPanel);

    chart_ = new QChart();
    chart_->legend()->hide();

    axX_ = new QValueAxis();
    axY_ = new QValueAxis();
    axX_->setRange(0.0, 1.0);
    axY_->setRange(-1.0, 1.0);
    axX_->setLabelFormat("%.3f");
    axY_->setLabelFormat("%10.5f");
    chart_->addAxis(axX_, Qt::AlignBottom);
    chart_->addAxis(axY_, Qt::AlignLeft);

    centerLine_ = new QLineSeries(chart_);
    chart_->addSeries(centerLine_);
    centerLine_->attachAxis(axX_);
    centerLine_->attachAxis(axY_);
    {
        auto pen = centerLine_->pen();
        pen.setWidthF(1.0);
        pen.setStyle(Qt::DashLine);
        auto c = pen.color();
        c.setAlpha(140);
        pen.setColor(c);
        centerLine_->setPen(pen);
    }

    chartView_ = new QChartView(chart_, chartPanel);
    chartView_->setRenderHint(QPainter::Antialiasing, true);
    chartL->addWidget(chartView_, 1);

    lb_stream_stats_ = new QLabel("Total: 0 | Time: 0.000 s | 1s: 0 | dt: 0.000 ms", chartPanel);
    lb_stream_stats_->setObjectName("StatusLabel");
    lb_stream_stats_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    chartL->addWidget(lb_stream_stats_);

    split->addWidget(chartPanel);

    // Right: Controls (간단히: 기존대로 유지)
    auto* ctrlPanel = new QWidget(split);
    ctrlPanel->setMinimumWidth(420);
    auto* ctrlL = new QVBoxLayout(ctrlPanel);

    auto* gPT = new QGroupBox("PositionTracking", ctrlPanel);
    auto* ptL = new QVBoxLayout(gPT);
    btn_pt_ = new QPushButton("Open PositionTracking Window", gPT);
    connect(btn_pt_, &QPushButton::clicked, this, &MainWindow::onOpenPositionTracking);
    ptL->addWidget(btn_pt_);
    ctrlL->addWidget(gPT);

    auto* gPlot = new QGroupBox("Plot", ctrlPanel);
    auto* pL = new QVBoxLayout(gPlot);

    sp_xwin_ = new QDoubleSpinBox(gPlot);
    sp_xwin_->setRange(0.5, 120.0);
    sp_xwin_->setDecimals(2);
    sp_xwin_->setValue(1.0);

    sp_ycenter_ = new QDoubleSpinBox(gPlot);
    sp_ycenter_->setRange(-1e9, 1e9);
    sp_ycenter_->setDecimals(6);
    sp_ycenter_->setValue(0.0);

    cb_yauto_ = new QCheckBox("Auto Y", gPlot);
    cb_yauto_->setChecked(true);

    sp_yabs_ = new QDoubleSpinBox(gPlot);
    sp_yabs_->setRange(1e-12, 1e9);
    sp_yabs_->setDecimals(6);
    sp_yabs_->setValue(1.0);
    sp_yabs_->setEnabled(false);

    connect(cb_yauto_, &QCheckBox::toggled, this, [this](bool on) {
        sp_yabs_->setEnabled(!on);
    });

    auto* row1 = new QWidget(gPlot);
    auto* r1 = new QHBoxLayout(row1);
    r1->addWidget(new QLabel("X window (s)"));
    r1->addWidget(sp_xwin_);
    pL->addWidget(row1);

    auto* row2 = new QWidget(gPlot);
    auto* r2 = new QHBoxLayout(row2);
    r2->addWidget(new QLabel("Y center"));
    r2->addWidget(sp_ycenter_);
    pL->addWidget(row2);

    auto* row3 = new QWidget(gPlot);
    auto* r3 = new QHBoxLayout(row3);
    r3->addWidget(cb_yauto_);
    r3->addWidget(new QLabel("Y abs"));
    r3->addWidget(sp_yabs_);
    pL->addWidget(row3);

    ctrlL->addWidget(gPlot);

    auto* gFilters = new QGroupBox("Filters", ctrlPanel);
    auto* fL = new QVBoxLayout(gFilters);

    cb_ma_ = new QCheckBox("MA", gFilters);
    cb_ma_->setChecked(false);
    sp_ma_ = new QSpinBox(gFilters);
    sp_ma_->setRange(1, 20000);
    sp_ma_->setValue(5);

    auto* maRow = new QWidget(gFilters);
    auto* maL = new QHBoxLayout(maRow);
    maL->addWidget(cb_ma_);
    maL->addWidget(sp_ma_);
    maL->addStretch(1);
    fL->addWidget(maRow);

    cb_ema_ = new QCheckBox("EMA", gFilters);
    cb_ema_->setChecked(false);
    sp_alpha_ = new QDoubleSpinBox(gFilters);
    sp_alpha_->setRange(0.0, 1.0);
    sp_alpha_->setDecimals(4);
    sp_alpha_->setValue(0.2);

    auto* emaRow = new QWidget(gFilters);
    auto* emaL = new QHBoxLayout(emaRow);
    emaL->addWidget(cb_ema_);
    emaL->addWidget(sp_alpha_);
    emaL->addStretch(1);
    fL->addWidget(emaRow);

    cb_notch_ = new QCheckBox("Notch", gFilters);
    cb_notch_->setChecked(false);

    sp_fs_ = new QDoubleSpinBox(gFilters);
    sp_fs_->setRange(10, 50000);
    sp_fs_->setDecimals(2);
    sp_fs_->setValue(200.0);

    sp_f0_ = new QDoubleSpinBox(gFilters);
    sp_f0_->setRange(1, 5000);
    sp_f0_->setDecimals(2);
    sp_f0_->setValue(60.0);

    sp_q_ = new QDoubleSpinBox(gFilters);
    sp_q_->setRange(0.1, 2000);
    sp_q_->setDecimals(2);
    sp_q_->setValue(30.0);

    auto* notchRow = new QWidget(gFilters);
    auto* notchL = new QHBoxLayout(notchRow);
    notchL->addWidget(cb_notch_);
    notchL->addWidget(new QLabel("fs"));
    notchL->addWidget(sp_fs_);
    notchL->addWidget(new QLabel("f0"));
    notchL->addWidget(sp_f0_);
    notchL->addWidget(new QLabel("Q"));
    notchL->addWidget(sp_q_);
    fL->addWidget(notchRow);

    ctrlL->addWidget(gFilters);

    cb_bias_apply_ = new QCheckBox("Apply stored bias", ctrlPanel);
    cb_bias_apply_->setChecked(false);
    sp_bias_frames_ = new QSpinBox(ctrlPanel);
    sp_bias_frames_->setRange(1, 2000000);
    sp_bias_frames_->setValue(200);

    btn_bias_cap_ = new QPushButton("Capture", ctrlPanel);
    connect(btn_bias_cap_, &QPushButton::clicked, this, &MainWindow::onBiasCapture);

    btn_bias_save_ = new QPushButton("Save CSV", ctrlPanel);
    connect(btn_bias_save_, &QPushButton::clicked, this, &MainWindow::onBiasSave);

    lb_bias_state_ = new QLabel("State: None", ctrlPanel);
    lb_bias_state_->setObjectName("StatusLabel");

    auto* biasRow = new QWidget(ctrlPanel);
    auto* biasL = new QHBoxLayout(biasRow);
    biasL->addWidget(cb_bias_apply_);
    biasL->addWidget(sp_bias_frames_);
    biasL->addWidget(btn_bias_cap_);
    biasL->addWidget(btn_bias_save_);
    ctrlL->addWidget(biasRow);
    ctrlL->addWidget(lb_bias_state_);

    cb_record_ = new QCheckBox("Record", ctrlPanel);
    ed_csv_path_ = new QLineEdit(ctrlPanel);
    ed_csv_path_->setReadOnly(true);
    btn_browse_csv_ = new QPushButton("Browse", ctrlPanel);
    connect(btn_browse_csv_, &QPushButton::clicked, this, &MainWindow::onBrowseCsv);
    connect(cb_record_, &QCheckBox::toggled, this, &MainWindow::onToggleRecord);

    auto* recRow = new QWidget(ctrlPanel);
    auto* recL = new QHBoxLayout(recRow);
    recL->addWidget(cb_record_);
    recL->addWidget(ed_csv_path_, 1);
    recL->addWidget(btn_browse_csv_);
    ctrlL->addWidget(recRow);

    ctrlL->addStretch(1);

    auto applyHook = [this]() { onAnyControlChanged(); };

    connect(sp_xwin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) { clearPlotData(); });

    connect(sp_ycenter_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, applyHook);
    connect(sp_yabs_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, applyHook);
    connect(cb_yauto_, &QCheckBox::toggled, this, applyHook);

    connect(cb_ma_, &QCheckBox::toggled, this, applyHook);
    connect(sp_ma_, QOverload<int>::of(&QSpinBox::valueChanged), this, applyHook);

    connect(cb_ema_, &QCheckBox::toggled, this, applyHook);
    connect(sp_alpha_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, applyHook);

    connect(cb_notch_, &QCheckBox::toggled, this, applyHook);
    connect(sp_fs_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, applyHook);
    connect(sp_f0_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, applyHook);
    connect(sp_q_,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, applyHook);

    connect(cb_bias_apply_, &QCheckBox::toggled, this, applyHook);

    split->addWidget(ctrlPanel);
    split->setStretchFactor(0, 2);
    split->setStretchFactor(1, 16);
    split->setStretchFactor(2, 3);

    root->addWidget(split);
    setCentralWidget(central);

    setWindowTitle("SoftionicsHub");
    resize(1850, 980);
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
    cfg.notch_q  = sp_q_->value();

    cfg.enable_bias = cb_bias_apply_->isChecked();
    return cfg;
}

void MainWindow::rescalePlotTime(double ratio) {
    for (auto& buf : buffers_) {
        for (auto& pt : buf) {
            pt.setX(pt.x() * ratio);
        }
    }
    for (auto& pf : pending_) {
        pf.t *= ratio;
    }
}

void MainWindow::clearPlotData() {
    pending_.clear();

    sampleIndex_ = 0;

    plotFs_ = sp_fs_ ? sp_fs_->value() : 200.0;
    if (plotFs_ < 1.0) plotFs_ = 1.0;
    dtPlot_ = 1.0 / plotFs_;

    for (auto& b : buffers_) b.clear();
    for (auto* s : series_) s->replace(QList<QPointF>());

    if (centerLine_) centerLine_->replace(QList<QPointF>());

    double xwin = sp_xwin_ ? sp_xwin_->value() : 1.0;
    if (xwin < 0.5) xwin = 0.5;
    axX_->setRange(0.0, xwin);
}

void MainWindow::beginConnecting(const QString& addr, const QString& name) {
    connecting_ = true;
    connectingAddr_ = addr;
    connectingName_ = name;
    if (connectTimeout_) connectTimeout_->start();
    updateDeviceListDecor();
}

void MainWindow::endConnecting() {
    connecting_ = false;
    connectingAddr_.clear();
    connectingName_.clear();
    if (connectTimeout_) connectTimeout_->stop();
    updateDeviceListDecor();
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

void MainWindow::onOpenPositionTracking() {
    if (!ptWin_) ptWin_ = new PositionTrackingWindow(worker_, this);
    ptWin_->show();
    ptWin_->raise();
    ptWin_->activateWindow();
}

void MainWindow::onScanUpdated(QVector<DeviceInfo> devices) {
    devices_ = devices;

    list_->clear();

    bool foundConnected = false;
    bool foundConnecting = false;

    for (int i = 0; i < devices_.size(); ++i) {
        const auto& d = devices_[i];

        QString text = QString("%1  (%2)  rssi=%3").arg(d.name).arg(d.address).arg(d.rssi);
        auto* item = new QListWidgetItem(text);
        item->setData(ROLE_ADDR, d.address);
        item->setData(ROLE_NAME, d.name);
        item->setData(ROLE_SCAN_INDEX, i);
        list_->addItem(item);

        if (!connectedAddr_.isEmpty() && d.address == connectedAddr_) foundConnected = true;
        if (connecting_ && d.address == connectingAddr_) foundConnecting = true;
    }

    if (!connectedAddr_.isEmpty() && !foundConnected) {
        QString nm = connectedName_.isEmpty() ? "CONNECTED" : connectedName_;
        QString text = QString("%1  (%2)  rssi=--").arg(nm).arg(connectedAddr_);
        auto* item = new QListWidgetItem(text);
        item->setData(ROLE_ADDR, connectedAddr_);
        item->setData(ROLE_NAME, nm);
        item->setData(ROLE_SCAN_INDEX, -1);
        list_->insertItem(0, item);
    }

    if (connecting_ && !connectingAddr_.isEmpty() && !foundConnecting) {
        QString nm = connectingName_.isEmpty() ? "CONNECTING" : connectingName_;
        QString text = QString("%1  (%2)  rssi=--").arg(nm).arg(connectingAddr_);
        auto* item = new QListWidgetItem(text);
        item->setData(ROLE_ADDR, connectingAddr_);
        item->setData(ROLE_NAME, nm);
        item->setData(ROLE_SCAN_INDEX, -1);
        list_->insertItem(0, item);
    }

    updateDeviceListDecor();
}

void MainWindow::updateDeviceListDecor() {
    for (int i = 0; i < list_->count(); ++i) {
        auto* it = list_->item(i);
        QString addr = it->data(ROLE_ADDR).toString();

        int st = 0;
        if (!connectedAddr_.isEmpty() && addr == connectedAddr_) st = 1;
        if (connecting_ && addr == connectingAddr_) st = 2;

        it->setData(ROLE_STATE, st);

        auto flags = it->flags();
        if (st == 2) flags &= ~Qt::ItemIsEnabled;
        else flags |= Qt::ItemIsEnabled;
        it->setFlags(flags);

        if (st == 2) it->setForeground(QBrush(QColor(150, 150, 150)));
        else it->setForeground(QBrush());
    }
    list_->viewport()->update();
}

void MainWindow::onDeviceClicked(QListWidgetItem* item) {
    if (!item) return;
    if (connecting_) return;

    QString addr = item->data(ROLE_ADDR).toString();
    QString name = item->data(ROLE_NAME).toString();
    int scanIndex = item->data(ROLE_SCAN_INDEX).toInt();

    if (!connectedAddr_.isEmpty() && addr == connectedAddr_) return;
    if (scanIndex < 0) return;

    beginConnecting(addr, name);
    QMetaObject::invokeMethod(worker_, [w = worker_, scanIndex]() { w->connectToIndex(scanIndex); }, Qt::QueuedConnection);
}

void MainWindow::onStatus(QString text) {
    status_->setText(text);

    if (connecting_) {
        QString t = text.toLower();
        if (t.contains("connect failed") || t.contains("no notify") || t.contains("no bluetooth")) {
            endConnecting();
        }
    }
}

void MainWindow::onConnected(QString name, QString addr) {
    connectedAddr_ = addr;
    connectedName_ = name;
    conn_->setText(QString("Connected: %1  %2").arg(name).arg(addr));

    endConnecting();
    updateDeviceListDecor();

    fsAutoSetDone_ = false;
    clearPlotData();
}

void MainWindow::onDisconnected() {
    connectedAddr_.clear();
    connectedName_.clear();
    conn_->setText("-");

    clearPlotData();

    if (!connecting_) endConnecting();
    updateDeviceListDecor();
}

void MainWindow::onStats(qulonglong ok, qulonglong bad) {
    stats_->setText(QString("ok=%1 bad=%2").arg(ok).arg(bad));
}

void MainWindow::onBiasState(bool hasBias, bool capturing) {
    if (capturing) lb_bias_state_->setText("State: Capturing");
    else if (hasBias) lb_bias_state_->setText("State: Stored");
    else lb_bias_state_->setText("State: None");
}

void MainWindow::onStreamStats(qulonglong totalSamples, double totalTimeSec, qulonglong last1sSamples, double lastDtSec) {
    double dt_ms = lastDtSec * 1000.0;
    lb_stream_stats_->setText(QString("Total: %1 | Time: %2 s | 1s: %3 | dt: %4 ms")
        .arg(totalSamples)
        .arg(totalTimeSec, 0, 'f', 3)
        .arg(last1sSamples)
        .arg(dt_ms, 0, 'f', 3));

    // ✅ plot x간격은 uniform이지만, fs를 더 정확히 만들고 싶으면 "한 번만" rescale
    if (!fsAutoSetDone_ && totalTimeSec >= 1.0 && last1sSamples > 0) {
        fsAutoSetDone_ = true;

        double newFs = (double)last1sSamples;
        if (newFs < 1.0) newFs = 1.0;

        double oldFs = plotFs_;
        plotFs_ = newFs;
        dtPlot_ = 1.0 / plotFs_;

        // x축 전체를 리셋하지 않고, 기존 점들 x만 스케일 보정
        // x = k*(1/oldFs) -> k*(1/newFs) => x *= oldFs/newFs
        double ratio = oldFs / newFs;
        rescalePlotTime(ratio);

        sp_fs_->setValue(newFs);
        status_->setText(QString("Sampling rate auto-set: %1 Hz").arg((int)newFs));
    }
}

void MainWindow::onFrame(qulonglong, QVector<float> x, bool, float) {
    double t = (double)sampleIndex_ * dtPlot_;
    sampleIndex_++;
    pending_.push_back(PendingFrame{t, std::move(x)});
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
        c.setAlpha(120);
        pen.setColor(c);
        s->setPen(pen);
        series_.push_back(s);
        buffers_.push_back(QList<QPointF>());
    }
}

void MainWindow::onPlotTick() {
    if (pending_.empty()) return;

    std::vector<PendingFrame> local;
    local.swap(pending_);

    int n_ch = (int)local.front().x.size();
    if (n_ch <= 0) return;
    if (series_.size() != n_ch) rebuildPlot(n_ch);

    double t_end = local.back().t;

    for (const auto& f : local) {
        if ((int)f.x.size() != n_ch) continue;
        for (int i = 0; i < n_ch; ++i) {
            buffers_[i].append(QPointF(f.t, f.x[i]));
        }
    }

    double xwin = sp_xwin_ ? sp_xwin_->value() : 1.0;
    if (xwin < 0.5) xwin = 0.5;

    double xMin = t_end - xwin;
    if (xMin < 0.0) xMin = 0.0;
    double xMax = xMin + xwin;
    axX_->setRange(xMin, xMax);

    for (int i = 0; i < n_ch; ++i) {
        while (!buffers_[i].isEmpty() && buffers_[i].first().x() < xMin) buffers_[i].removeFirst();
    }

    double yCenter = sp_ycenter_ ? sp_ycenter_->value() : 0.0;
    bool yAuto = cb_yauto_ ? cb_yauto_->isChecked() : true;

    double yAbs = 1.0;
    if (yAuto) {
        double maxAbs = 0.0;
        for (int i = 0; i < n_ch; ++i) {
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

    if (centerLine_) {
        QList<QPointF> pts;
        pts.append(QPointF(xMin, yCenter));
        pts.append(QPointF(xMax, yCenter));
        centerLine_->replace(pts);
    }

    for (int i = 0; i < n_ch; ++i) {
        series_[i]->replace(buffers_[i]);
    }
}
