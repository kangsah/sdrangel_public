#ifndef INCLUDE_BFMSHAREDVFOBRANCH_H
#define INCLUDE_BFMSHAREDVFOBRANCH_H

#include <memory>

#include <QHash>
#include <QMutex>
#include <QString>
#include <QThread>

#include "bfmearlyifworker.h"

class BFMDemodBaseband;

class BFMSharedVFOBranch
{
public:
    static std::shared_ptr<BFMSharedVFOBranch> acquire(
        const QString& key,
        int inputSampleRate,
        int outputSampleRate,
        unsigned int decimation,
        qint64 frequencyOffset,
        BFMDemodBaseband* output);

    ~BFMSharedVFOBranch();

    BFMEarlyIFWorker* worker() { return m_worker; }
    void feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end);
    void configure(int inputSampleRate, int outputSampleRate, unsigned int decimation, qint64 frequencyOffset);
    void removeOutput(BFMDemodBaseband* output);

private:
    static QMutex s_mutex;
    static QHash<QString, std::weak_ptr<BFMSharedVFOBranch>> s_branches;

    QThread* m_thread;
    BFMEarlyIFWorker* m_worker;
    QString m_key;
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
        BFMDemodBaseband* output);
};

#endif // INCLUDE_BFMSHAREDVFOBRANCH_H
