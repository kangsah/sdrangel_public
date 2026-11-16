#ifndef INCLUDE_BFMSHAREDVFOBRANCH_H
#define INCLUDE_BFMSHAREDVFOBRANCH_H

#include <memory>
#include <atomic>
#include <condition_variable>
#include <thread>

#include <QHash>
#include <QMutex>
#include <QString>
#include <QThread>
#include <QElapsedTimer>

#include "dsp/basebandsamplesink.h"
#include "dsp/decimators.h"
#include "dsp/samplesinkfifo.h"
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
    SampleSinkFifo m_rawFifo;
    std::thread m_preThread;
    std::mutex m_preWaitMutex;
    std::condition_variable m_preWaitCondition;
    std::atomic<bool> m_preStop;
    std::atomic<quint64> m_preDroppedSamples;
    std::atomic<unsigned int> m_preHighWater;
    bool m_directDecimation;
    unsigned int m_decimation;
    SampleVector m_directBuffer;
    SampleVector m_directIntermediate;
    Decimators<qint32, qint32, SDR_RX_SAMP_SZ, SDR_RX_SAMP_SZ, true> m_decimators1;
    Decimators<qint32, qint32, SDR_RX_SAMP_SZ, SDR_RX_SAMP_SZ, true> m_decimators2;
    quint64 m_directInputSamples;
    quint64 m_directOutputSamples;
    quint64 m_directProcessingNsecs;
    QElapsedTimer m_directReportTimer;
    SampleVector::const_iterator m_lastBegin;
    SampleVector::const_iterator m_lastEnd;
    bool m_haveLastBlock;

    void processDirect();

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
