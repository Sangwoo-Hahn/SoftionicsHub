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
#include <QToolButton>
#include <QButtonGroup>

#include <algorithm>
#include <cmath>
#include <complex>

#include <QSignalBlocker>

static constexpr int ROLE_ADDR       = Qt::UserRole;
static constexpr int ROLE_STATE      = Qt::UserRole + 1;   // 0 normal, 1 connected, 2 connecting
static constexpr int ROLE_SCAN_INDEX = Qt::UserRole + 2;   // -1 pinned
static constexpr int ROLE_NAME       = Qt::UserRole + 3;

static inline int snap_pow2(int v, int min_v = 32, int max_v = 16384) {
    if (v < min_v) v = min_v;
    if (v > max_v) v = max_v;

    int p = 1;
    while (p < v && p < max_v) p <<= 1;

    int hi = p;
    int lo = p >> 1;
    if (lo < min_v) lo = min_v;

    // choose nearest power-of-two
    int snapped = hi;
    if (v - lo <= hi - v) snapped = lo;

    // clamp again (safety)
    if (snapped < min_v) snapped = min_v;
    if (snapped > max_v) snapped = max_v;
    return snapped;
}

static inline void fft_inplace(std::vector<std::complex<float>>& a) {
    const size_t n = a.size();
    if (n <= 1) return;

    // bit-reversal permutation
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }

    constexpr float PI = 3.14159265358979323846f;

    for (size_t len = 2; len <= n; len <<= 1) {
        const float ang = -2.0f * PI / (float)len;
        const std::complex<float> wlen(std::cos(ang), std::sin(ang));

        for (size_t i = 0; i < n; i += len) {
            std::complex<float> w(1.0f, 0.0f);
            const size_t half = len >> 1;

            for (size_t j = 0; j < half; ++j) {
                const std::complex<float> u = a[i + j];
                const std::complex<float> v = a[i + j + half] * w;
                a[i + j] = u + v;
                a[i + j + half] = u - v;
                w *= wlen;
            }
        }
    }
}


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

    fftClock_.start();

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

    auto* devTitle = new QLabel("Devices (BLE / COM, click to connect)", devPanel);
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

    // --- Serial input format toggle (AUTO / DEC / HEX / BIN / I16) ---
    // 센서가 HEX/BIN 등으로 들어오는 경우를 위해, 그래프 위에 작은 토글을 둔다.
    auto* fmtRow = new QWidget(chartPanel);
    fmtRow->setMaximumHeight(28);
    auto* fmtL = new QHBoxLayout(fmtRow);
    fmtL->setContentsMargins(0, 0, 0, 0);
    fmtL->setSpacing(6);
    fmtL->addWidget(new QLabel("Input", fmtRow));

    auto* fmtGroup = new QButtonGroup(fmtRow);
    fmtGroup->setExclusive(true);

    auto makeBtn = [&](const QString& text, int id) {
        auto* b = new QToolButton(fmtRow);
        b->setText(text);
        b->setCheckable(true);
        b->setAutoRaise(true);
        b->setToolButtonStyle(Qt::ToolButtonTextOnly);
        fmtGroup->addButton(b, id);
        fmtL->addWidget(b);
        return b;
    };

    auto* bAuto = makeBtn("AUTO", 0);
    makeBtn("DEC", 1);
    makeBtn("HEX", 2);
    makeBtn("BIN", 3);
    makeBtn("I16", 4);
    bAuto->setChecked(true);

    fmtL->addStretch(1);
    chartL->addWidget(fmtRow, 0);

    connect(fmtGroup, &QButtonGroup::idClicked, this, [this](int id) {
        if (!worker_) return;
        QMetaObject::invokeMethod(worker_, "setInputFormat", Qt::QueuedConnection, Q_ARG(int, id));
    });

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

    // Moving Average
    cb_ma_ = new QCheckBox("Moving Average", gFilters);
    cb_ma_->setChecked(false);

    sp_ma_ = new QSpinBox(gFilters);
    sp_ma_->setRange(1, 20000);
    sp_ma_->setValue(5);

    sp_ma_order_ = new QSpinBox(gFilters);
    sp_ma_order_->setRange(1, 16);
    sp_ma_order_->setValue(1);

    auto* maRow = new QWidget(gFilters);
    auto* maL = new QHBoxLayout(maRow);
    maL->addWidget(cb_ma_);
    maL->addWidget(new QLabel("N"));
    maL->addWidget(sp_ma_);
    maL->addWidget(new QLabel("Order"));
    maL->addWidget(sp_ma_order_);
    maL->addStretch(1);
    fL->addWidget(maRow);

    // Exponential Moving Average
    cb_ema_ = new QCheckBox("Exponential Moving Average", gFilters);
    cb_ema_->setChecked(false);

    sp_alpha_ = new QDoubleSpinBox(gFilters);
    sp_alpha_->setRange(0.0, 1.0);
    sp_alpha_->setDecimals(4);
    sp_alpha_->setValue(0.2);

    sp_ema_order_ = new QSpinBox(gFilters);
    sp_ema_order_->setRange(1, 16);
    sp_ema_order_->setValue(1);

    auto* emaRow = new QWidget(gFilters);
    auto* emaL = new QHBoxLayout(emaRow);
    emaL->addWidget(cb_ema_);
    emaL->addWidget(new QLabel("Alpha"));
    emaL->addWidget(sp_alpha_);
    emaL->addWidget(new QLabel("Order"));
    emaL->addWidget(sp_ema_order_);
    emaL->addStretch(1);
    fL->addWidget(emaRow);

    // Notch
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

    sp_notch_order_ = new QSpinBox(gFilters);
    sp_notch_order_->setRange(1, 16);
    sp_notch_order_->setValue(1);

    auto* notchRow = new QWidget(gFilters);
    auto* notchL = new QHBoxLayout(notchRow);
    notchL->addWidget(cb_notch_);
    notchL->addWidget(new QLabel("fs"));
    notchL->addWidget(sp_fs_);
    notchL->addWidget(new QLabel("f0"));
    notchL->addWidget(sp_f0_);
    notchL->addWidget(new QLabel("Q"));
    notchL->addWidget(sp_q_);
    notchL->addWidget(new QLabel("Order"));
    notchL->addWidget(sp_notch_order_);
    fL->addWidget(notchRow);

    // V/RC + dV/dt
    cb_vrc_ = new QCheckBox("V/RC + dV/dt", gFilters);
    cb_vrc_->setChecked(false);

    sp_vrc_rc_ = new QDoubleSpinBox(gFilters);
    sp_vrc_rc_->setRange(1e-6, 1e6);
    sp_vrc_rc_->setDecimals(6);
    sp_vrc_rc_->setValue(0.05);

    sp_vrc_n_ = new QSpinBox(gFilters);
    sp_vrc_n_->setRange(2, 2000);
    sp_vrc_n_->setValue(5);

    sp_vrc_order_ = new QSpinBox(gFilters);
    sp_vrc_order_->setRange(1, 16);
    sp_vrc_order_->setValue(1);

    auto* vrcRow = new QWidget(gFilters);
    auto* vrcL = new QHBoxLayout(vrcRow);
    vrcL->addWidget(cb_vrc_);
    vrcL->addWidget(new QLabel("RC"));
    vrcL->addWidget(sp_vrc_rc_);
    vrcL->addWidget(new QLabel("n"));
    vrcL->addWidget(sp_vrc_n_);
    vrcL->addWidget(new QLabel("Order"));
    vrcL->addWidget(sp_vrc_order_);
    vrcL->addStretch(1);
    fL->addWidget(vrcRow);

    // FFT (display)
    cb_fft_ = new QCheckBox("FFT (Spectrum)", gFilters);
    cb_fft_->setChecked(false);

    sp_fft_n_ = new QSpinBox(gFilters);
    sp_fft_n_->setRange(32, 16384);
    sp_fft_n_->setSingleStep(32);
    sp_fft_n_->setValue(1024);

    sp_fft_avg_ = new QSpinBox(gFilters);
    sp_fft_avg_->setRange(1, 64);
    sp_fft_avg_->setValue(1);

    auto* fftRow = new QWidget(gFilters);
    auto* fftL = new QHBoxLayout(fftRow);
    fftL->addWidget(cb_fft_);
    fftL->addWidget(new QLabel("N"));
    fftL->addWidget(sp_fft_n_);
    fftL->addWidget(new QLabel("Order"));
    fftL->addWidget(sp_fft_avg_);
    fftL->addStretch(1);
    fL->addWidget(fftRow);

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
    connect(sp_ma_order_, QOverload<int>::of(&QSpinBox::valueChanged), this, applyHook);

    connect(cb_ema_, &QCheckBox::toggled, this, applyHook);
    connect(sp_alpha_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, applyHook);
    connect(sp_ema_order_, QOverload<int>::of(&QSpinBox::valueChanged), this, applyHook);

    connect(cb_notch_, &QCheckBox::toggled, this, applyHook);
    connect(sp_fs_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, applyHook);
    connect(sp_f0_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, applyHook);
    connect(sp_q_,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, applyHook);
    connect(sp_notch_order_, QOverload<int>::of(&QSpinBox::valueChanged), this, applyHook);

    connect(cb_vrc_, &QCheckBox::toggled, this, applyHook);
    connect(sp_vrc_rc_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, applyHook);
    connect(sp_vrc_n_, QOverload<int>::of(&QSpinBox::valueChanged), this, applyHook);
    connect(sp_vrc_order_, QOverload<int>::of(&QSpinBox::valueChanged), this, applyHook);

    // FFT (display mode)
    connect(cb_fft_, &QCheckBox::toggled, this, &MainWindow::onFftControlsChanged);
    connect(sp_fft_n_, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::onFftControlsChanged);
    connect(sp_fft_avg_, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::onFftControlsChanged);

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
    cfg.ma_order = (size_t)(sp_ma_order_ ? sp_ma_order_->value() : 1);

    cfg.enable_ema = cb_ema_->isChecked();
    cfg.ema_alpha = (float)sp_alpha_->value();
    cfg.ema_order = (size_t)(sp_ema_order_ ? sp_ema_order_->value() : 1);

    cfg.enable_notch = cb_notch_->isChecked();
    cfg.fs_hz = sp_fs_->value();
    cfg.notch_f0 = sp_f0_->value();
    cfg.notch_q  = sp_q_->value();
    cfg.notch_order = (size_t)(sp_notch_order_ ? sp_notch_order_->value() : 1);

    cfg.enable_vrc = cb_vrc_ ? cb_vrc_->isChecked() : false;
    cfg.vrc_rc = sp_vrc_rc_ ? sp_vrc_rc_->value() : 0.05;
    cfg.vrc_n = (size_t)(sp_vrc_n_ ? sp_vrc_n_->value() : 5);
    cfg.vrc_order = (size_t)(sp_vrc_order_ ? sp_vrc_order_->value() : 1);

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

    // reset FFT state
    lastFftMs_ = 0;
    fftPos_ = 0;
    fftCount_ = 0;
    fftWindow_.clear();
    fftRing_.clear();

    fftAvgPos_ = 0;
    fftAvgCount_ = 0;
    fftAvgRing_.clear();
    fftAvgSum_.clear();

    const bool fftOn = (cb_fft_ && cb_fft_->isChecked());

    if (fftOn) {
        axX_->setTitleText("Frequency (Hz)");
        axX_->setLabelFormat("%.1f");
        axX_->setRange(0.0, plotFs_ * 0.5);
    } else {
        axX_->setTitleText("Time (s)");
        axX_->setLabelFormat("%.3f");

        double xwin = sp_xwin_ ? sp_xwin_->value() : 1.0;
        if (xwin < 0.5) xwin = 0.5;
        axX_->setRange(0.0, xwin);
    }
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

    auto isSerialId = [](const QString& s) {
        return s.startsWith("COM", Qt::CaseInsensitive) || s.startsWith("/dev/");
    };

    bool foundConnected = false;
    bool foundConnecting = false;

    for (int i = 0; i < devices_.size(); ++i) {
        const auto& d = devices_[i];

        QString text;
        if (d.kind == DeviceKind::Ble) {
            text = QString("%1  (%2)  rssi=%3").arg(d.name).arg(d.address).arg(d.rssi);
        } else {
            text = QString("[COM] %1").arg(d.name);
        }
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
        QString text;
        if (isSerialId(connectedAddr_)) text = QString("[COM] %1  (%2)").arg(nm).arg(connectedAddr_);
        else text = QString("%1  (%2)  rssi=--").arg(nm).arg(connectedAddr_);
        auto* item = new QListWidgetItem(text);
        item->setData(ROLE_ADDR, connectedAddr_);
        item->setData(ROLE_NAME, nm);
        item->setData(ROLE_SCAN_INDEX, -1);
        list_->insertItem(0, item);
    }

    if (connecting_ && !connectingAddr_.isEmpty() && !foundConnecting) {
        QString nm = connectingName_.isEmpty() ? "CONNECTING" : connectingName_;
        QString text;
        if (isSerialId(connectingAddr_)) text = QString("[COM] %1  (%2)").arg(nm).arg(connectingAddr_);
        else text = QString("%1  (%2)  rssi=--").arg(nm).arg(connectingAddr_);
        auto* item = new QListWidgetItem(text);
        item->setData(ROLE_ADDR, connectingAddr_);
        item->setData(ROLE_NAME, nm);
        item->setData(ROLE_SCAN_INDEX, -1);
        list_->insertItem(0, item);
    }

    updateDeviceListDecor();

}

void MainWindow::onFftControlsChanged() {
    // Snap N to a power-of-two so the FFT stays fast.
    if (sp_fft_n_) {
        int snapped = snap_pow2(sp_fft_n_->value());
        if (snapped != sp_fft_n_->value()) {
            QSignalBlocker b(sp_fft_n_);
            sp_fft_n_->setValue(snapped);
        }
    }

    if (sp_fft_avg_) {
        int k = sp_fft_avg_->value();
        if (k < 1) k = 1;
        if (k > 64) k = 64;
        if (k != sp_fft_avg_->value()) {
            QSignalBlocker b(sp_fft_avg_);
            sp_fft_avg_->setValue(k);
        }
    }

    clearPlotData();
}

void MainWindow::ensureFftState(int n_ch) {
    if (!cb_fft_ || !cb_fft_->isChecked()) return;
    if (n_ch <= 0) return;

    const int N = sp_fft_n_ ? snap_pow2(sp_fft_n_->value()) : 1024;
    if (sp_fft_n_ && sp_fft_n_->value() != N) {
        QSignalBlocker b(sp_fft_n_);
        sp_fft_n_->setValue(N);
    }

    int K = sp_fft_avg_ ? sp_fft_avg_->value() : 1;
    if (K < 1) K = 1;
    if (K > 64) K = 64;

    const int binCount = (N / 2) + 1;

    const bool needReinit =
        (fftCh_ != n_ch) ||
        (fftN_ != N) ||
        (fftBinCount_ != binCount) ||
        (fftAvgK_ != K) ||
        ((int)fftWindow_.size() != N) ||
        (fftRing_.size() != (size_t)N * (size_t)n_ch) ||
        (fftAvgRing_.size() != (size_t)K * (size_t)n_ch * (size_t)binCount) ||
        (fftAvgSum_.size() != (size_t)n_ch * (size_t)binCount);

    if (!needReinit) return;

    fftCh_ = n_ch;
    fftN_ = N;
    fftBinCount_ = binCount;

    fftPos_ = 0;
    fftCount_ = 0;

    // Hann window
    fftWindow_.assign((size_t)N, 1.0f);
    if (N > 1) {
        constexpr double PI = 3.14159265358979323846;
        for (int i = 0; i < N; ++i) {
            const double w = 0.5 - 0.5 * std::cos(2.0 * PI * (double)i / (double)(N - 1));
            fftWindow_[(size_t)i] = (float)w;
        }
    }

    // Ring of recent samples: [pos][ch]
    fftRing_.assign((size_t)N * (size_t)n_ch, 0.0f);

    // Spectrum moving-average ("Order")
    fftAvgK_ = K;
    fftAvgPos_ = 0;
    fftAvgCount_ = 0;
    fftAvgRing_.assign((size_t)K * (size_t)n_ch * (size_t)binCount, 0.0f);
    fftAvgSum_.assign((size_t)n_ch * (size_t)binCount, 0.0);

    // Clear plot buffers
    for (auto& b : buffers_) b.clear();
    for (auto* s : series_) s->replace(QList<QPointF>());
    if (centerLine_) centerLine_->replace(QList<QPointF>());
}

void MainWindow::renderFftPlot(int n_ch) {
    if (n_ch <= 0) return;
    if (!cb_fft_ || !cb_fft_->isChecked()) return;
    if (fftN_ <= 0 || fftBinCount_ <= 0) return;
    if (fftCount_ < (size_t)fftN_) return;

    const double fs = plotFs_;
    const double fMax = fs * 0.5;

    const double xMin = 0.0;
    const double xMax = fMax;
    axX_->setRange(xMin, xMax);

    const double yCenter = sp_ycenter_ ? sp_ycenter_->value() : 0.0;
    const bool yAuto = cb_yauto_ ? cb_yauto_->isChecked() : true;

    double maxAbs = 0.0;

    std::vector<std::complex<float>> a((size_t)fftN_);
    std::vector<float> mag((size_t)fftBinCount_);

    // Oldest sample index in ring
    const size_t start = (fftCount_ == (size_t)fftN_) ? fftPos_ : 0;

    // 1) compute current spectrum for each channel, update moving-average ring/sum
    for (int ch = 0; ch < n_ch; ++ch) {
        for (int i = 0; i < fftN_; ++i) {
            const size_t pos = (start + (size_t)i) % (size_t)fftN_;
            const float v = fftRing_[pos * (size_t)n_ch + (size_t)ch];
            a[(size_t)i] = std::complex<float>(v * fftWindow_[(size_t)i], 0.0f);
        }

        fft_inplace(a);

        for (int k = 0; k < fftBinCount_; ++k) {
            mag[(size_t)k] = std::abs(a[(size_t)k]) / (float)fftN_;
        }

        float* ringSlot = fftAvgRing_.data() + ((size_t)(fftAvgPos_ * n_ch + ch) * (size_t)fftBinCount_);
        double* sum = fftAvgSum_.data() + ((size_t)ch * (size_t)fftBinCount_);

        for (int k = 0; k < fftBinCount_; ++k) {
            const float oldv = ringSlot[(size_t)k];
            const float newv = mag[(size_t)k];
            ringSlot[(size_t)k] = newv;
            sum[(size_t)k] += (double)newv - (double)oldv;
        }
    }

    // advance average ring
    fftAvgPos_++;
    if (fftAvgPos_ >= fftAvgK_) fftAvgPos_ = 0;
    if (fftAvgCount_ < fftAvgK_) fftAvgCount_++;

    // 2) render averaged spectrum to chart
    for (int ch = 0; ch < n_ch; ++ch) {
        const double* sum = fftAvgSum_.data() + ((size_t)ch * (size_t)fftBinCount_);
        const double inv = 1.0 / (double)std::max(fftAvgCount_, 1);

        QList<QPointF> pts;
        pts.reserve(fftBinCount_);

        for (int k = 0; k < fftBinCount_; ++k) {
            const double f = ((double)k * fs) / (double)fftN_;
            const double v = sum[(size_t)k] * inv;
            pts.append(QPointF(f, v));

            const double a = std::abs(v - yCenter);
            if (a > maxAbs) maxAbs = a;
        }

        buffers_[ch] = pts;
        series_[ch]->replace(buffers_[ch]);
    }

    double yAbs = 1.0;
    if (yAuto) {
        yAbs = (maxAbs > 1e-12) ? (maxAbs * 1.15) : 1.0;
    } else {
        double v = sp_yabs_ ? sp_yabs_->value() : 1.0;
        if (v < 1e-12) v = 1.0;
        yAbs = v;
    }

    axY_->setRange(yCenter - yAbs, yCenter + yAbs);

    if (centerLine_) {
        QList<QPointF> c;
        c.append(QPointF(xMin, yCenter));
        c.append(QPointF(xMax, yCenter));
        centerLine_->replace(c);
    }
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

        sp_fs_->setValue(newFs);

        // FFT 모드에서는 x축이 시간축이 아니므로(주파수축), 기존 점들을 스케일 보정하지 않고 리셋합니다.
        if (cb_fft_ && cb_fft_->isChecked()) clearPlotData();
        else rescalePlotTime(ratio);

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

    const bool fftOn = (cb_fft_ && cb_fft_->isChecked());

    if (fftOn) {
        ensureFftState(n_ch);

        // Push new samples into ring (frame ring: [pos][ch])
        for (const auto& f : local) {
            if ((int)f.x.size() != n_ch) continue;

            if (fftRing_.size() != (size_t)fftN_ * (size_t)n_ch) continue;

            for (int ch = 0; ch < n_ch; ++ch) {
                fftRing_[fftPos_ * (size_t)n_ch + (size_t)ch] = f.x[ch];
            }

            fftPos_++;
            if (fftPos_ >= (size_t)fftN_) fftPos_ = 0;
            if (fftCount_ < (size_t)fftN_) fftCount_++;
        }

        // Rate-limit heavy FFT work so we don't overwhelm the UI thread
        const qint64 ms = fftClock_.elapsed();
        int intervalMs = 100;
        if (fftN_ >= 8192) intervalMs = 300;
        else if (fftN_ >= 4096) intervalMs = 200;
        else if (fftN_ >= 2048) intervalMs = 140;

        if (ms - lastFftMs_ < intervalMs) return;
        lastFftMs_ = ms;

        renderFftPlot(n_ch);
        return;
    }

    // ---- time-domain plot ----

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
        yAbs = maxAbs * 1.1;
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
