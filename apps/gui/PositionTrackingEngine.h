#ifndef SOFTIONICS_GUI_POSITIONTRACKINGENGINE_H
#define SOFTIONICS_GUI_POSITIONTRACKINGENGINE_H

#include <QObject>
#include <QVector>
#include <QString>
#include <memory>
#include <vector>

#include "hub/model/PositionTrackingRegistry.h"

class PositionTrackingEngine : public QObject {
    Q_OBJECT
public:
    explicit PositionTrackingEngine(QObject* parent = nullptr);

public slots:
    void setAlgorithm(QString id);
    void setParams(QVector<double> params);
    void reset();
    void onSample(qulonglong t_ns, QVector<float> x, bool modelValid, float modelOut);

signals:
    void outputReady(double x, double y, double z, double confidence, double q1, double q2, double err, bool quiet, bool valid);
    void statusReady(QString text);

private:
    std::unique_ptr<hub::pt::IAlgorithm> algo_;
    std::string algoId_;
    std::vector<double> params_;
    std::vector<float> sampleBuf_;
    qulonglong lastStatusEmitNs_ = 0;
};

#endif
