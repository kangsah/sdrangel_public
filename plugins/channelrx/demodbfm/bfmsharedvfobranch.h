#ifndef INCLUDE_BFMSHAREDVFOBRANCH_H
#define INCLUDE_BFMSHAREDVFOBRANCH_H

#include <memory>

#include <QHash>
#include <QMutex>
#include <QString>
#include <QThread>

#include "dsp/basebandsamplesink.h"
#include "bfmearlyifworker.h"

class BFMDemodBaseband;
class DSPDeviceSourceEngine;

class BFMSharedVFOBranch : public BasebandSampleSink
{
public:
    static std::shared_ptr<BFMSharedVFOBranch> acquire(
        const QString& key,
        int inputSampleRate,
        int outputSampleRate,
        unsigned int decimation,
        qint64 frequencyOffset,
        DSPDeviceSourceEngine* sourceEngine,
        BFMDemodBaseband* output);

    ~BFMSharedVFOBranch();

    void start() override {}
    void stop() override {}
    void feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end, bool positiveOnly) override;
    void pushMessage(Message *msg) override { delete msg; }
    QString getSinkName() override { return "BFMSharedVFO"; }

    BFMEarlyIFWorker* worker() { return m_worker; }
    void configure(int inputSampleRate, int outputSampleRate, unsigned int decimation, qint64 frequencyOffset);
    void removeOutput(BFMDemodBaseband* output);

private:
    static QMutex s_mutex;
    static QHash<QString, std::weak_ptr<BFMSharedVFOBranch>> s_branches;

    QThread* m_thread;
    BFMEarlyIFWorker* m_worker;
    QString m_key;
    DSPDeviceSourceEngine* m_sourceEngine;
    QMutex m_feedMutex;
    SampleVector::const_iterator m_lastBegin;
    SampleVector::const_iterator m_lastEnd;
    bool m_haveLastBlock;

    BFMSharedVFOBranch(
        const QString& key,
        int inputSampleRate,
        int outputSampleRate,
        unsigned int decimation,
        qint64 frequencyOffset,
        DSPDeviceSourceEngine* sourceEngine,
        BFMDemodBaseband* output);
};

#endif // INCLUDE_BFMSHAREDVFOBRANCH_H
